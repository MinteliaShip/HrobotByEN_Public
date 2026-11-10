import asyncio
import threading
import os
import sys
import shutil
import logging
import subprocess
import math
import json
import re
import struct
from collections import deque

try:
    from bleak import BleakScanner, BleakClient
except ImportError:  # フレーム生成だけ使う場合（テスト等）
    BleakScanner = BleakClient = None

# ==============================================================
# ファイルの置き場所
#   APP_DIR: 設定・ポーズライブラリ(json)を保存する場所。main.py（exe化したときは exe）と同じフォルダ。
#   RES_DIR: 画面(index.html)の場所。exe化したときは同梱フォルダ（PyInstaller の _MEIPASS）。
# ==============================================================
if getattr(sys, "frozen", False):
    APP_DIR = os.path.dirname(os.path.abspath(sys.executable))
    RES_DIR = getattr(sys, "_MEIPASS", APP_DIR)
else:
    APP_DIR = RES_DIR = os.path.dirname(os.path.abspath(__file__))

APP_NAME = "RobotMotionStudio"
APP_JSON_FILES = ("pose_library.json", "settings.json")
logger = logging.getLogger(APP_NAME)


def _writable_dir(d):
    """ フォルダを作れて、ファイルを書き込めるか（OneDrive・書き込み禁止フォルダの確認用） """
    try:
        os.makedirs(d, exist_ok=True)
        test = os.path.join(d, f".write_test_{os.getpid()}")
        with open(test, 'w', encoding='utf-8') as f:
            f.write("ok")
        os.remove(test)
        return True
    except OSError:
        return False


def _resolve_data_dir():
    r""" DATA_DIR: 設定・ポーズライブラリ・ログの保存先。
        基本は APP_DIR。書き込めないときだけ %APPDATA%\RobotMotionStudio（初回は APP_DIR の旧ファイルを引き継ぐ）。 """
    if _writable_dir(APP_DIR):
        return APP_DIR
    base = os.environ.get("APPDATA") or os.path.join(os.path.expanduser("~"), ".config")
    d = os.path.join(base, APP_NAME)
    if not _writable_dir(d):
        return APP_DIR                       # どこにも書けない: 保存は失敗するが起動はする（ログに出る）
    for name in APP_JSON_FILES:
        src, dst = os.path.join(APP_DIR, name), os.path.join(d, name)
        try:
            if os.path.exists(src) and not os.path.exists(dst):
                shutil.copy2(src, dst)
        except OSError:
            pass
    return d


DATA_DIR = _resolve_data_dir()
LOG_PATH = os.path.join(DATA_DIR, "robot_motion_studio.log")


def _ble_hint(e):
    """ BLEのエラーに、利用者向けの原因と対処を添える（表示だけ。処理は変えない） """
    if BleakScanner is None:
        return "（Bluetooth 用の部品 bleak が入っていません。setup.bat を実行し直してください）"
    s = str(e).lower()
    if "turned off" in s or "powered off" in s or "poweredoff" in s or ("radio" in s and "off" in s):
        return "（Bluetooth がオフのようです。Windows の「設定 → Bluetooth とデバイス」で Bluetooth をオンにしてください）"
    if "adapter" in s or "no bluetooth" in s or "not available" in s or "unavailable" in s:
        return "（Bluetooth アダプタが見つかりません。PC に Bluetooth が無い場合は、BLE 対応の USB Bluetooth アダプタを挿してください）"
    return "（Bluetooth がオンか、ロボットの電源が入っていて近くにあるかを確認してください）"

# ==============================================================
# BLEプロトコル v2（ファームの Protocol.h と一致させること）
#   すべてのパケット: [type][seq][payload]
#   角度はすべて論理ICS（7500=校正済み中立、0=FREE）
# ==============================================================
PROTO_VERSION = 7
# PC -> ESP32
HELLO, SET_MODE, HEARTBEAT, ESTOP, ESTOP_CLEAR = 0x01, 0x02, 0x03, 0x04, 0x05
AXIS_CMD, POSE_CMD, NEUTRAL_REQ = 0x10, 0x11, 0x12
TEACH_FRAME, TEACH_POSE = 0x13, 0x14   # リアルタイム再生: ACK付き・順番どおり全部実行
UP_BEGIN, UP_DATA, UP_COMMIT, UP_START = 0x20, 0x21, 0x22, 0x23
CAL_ZERO_HERE, CAL_SET_FOOTDEF, CAL_DEFAULTS = 0x32, 0x33, 0x35
CAL_SET_OFFSET, CAL_SET_ALL, CAL_ZERO_ALL = 0x34, 0x36, 0x37
# ESP32 -> PC
ACK, HELLO_R, STATE, FB, POSE_ECHO, EVENT, CALIB, LINK = 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87
STATUS_TEXT = {0: "OK", 1: "拒否", 2: "状態不正", 3: "不正な値", 4: "保存失敗", 5: "処理中"}

FRAME_BYTES = 61
MAX_FRAMES = 100
KF_SNAP, KF_POSE_L, KF_POSE_R, KF_IK_L, KF_IK_R = 0x01, 0x02, 0x04, 0x08, 0x10
FL_UPLOAD_READY = 0x10
CHUNK = 16            # MTUが小さいときの書き込みデータ量
UP_DATA_MAX = 48     # 書き込みデータ量の上限（BLE MTU 64 で送れる量。ファームの UP_DATA_MAX と合わせる）
UP_WINDOW = 4        # 書き込みで同時に待つ件数（ファームは受信順に処理するので、offset は昇順に送る）


def build_frame(kf):
    """ キーフレーム(JS形式) → 61byte
        {DurationMs, Mode(0/1), Angles[19](論理,-1=無視,-2=空白,0=FREE),
         Poses: {left|right: {x,y,z,roll,yaw,ik}} (任意。指定した脚は座標で再生)} """
    duration = max(20, min(65535, int(kf.get('DurationMs', 500))))
    flags = KF_SNAP if kf.get('Mode') == 1 else 0
    poses = kf.get('Poses') or {}
    for key, fp, fik in (('left', KF_POSE_L, KF_IK_L), ('right', KF_POSE_R, KF_IK_R)):
        p = poses.get(key)
        if p:
            flags |= fp
            if int(p.get('ik', 0)):
                flags |= fik
    frame = bytearray(FRAME_BYTES)
    struct.pack_into('<HB', frame, 0, duration, flags)
    angles = kf.get('Angles', [])
    for axis in range(19):
        val = int(angles[axis]) if axis < len(angles) else -1
        struct.pack_into('<h', frame, 3 + axis * 2, max(-2, min(32767, val)))   # -2 = 空白（トラックに参加して保持）
    for f, key in enumerate(('left', 'right')):
        p = poses.get(key)
        if not p:
            continue
        vals = (round(float(p['x']) * 10), round(float(p['y']) * 10), round(float(p['z']) * 10),
                round(float(p.get('roll', 0)) * 100), round(float(p.get('yaw', 0)) * 100))
        struct.pack_into('<5h', frame, 41 + f * 10, *[max(-32768, min(32767, v)) for v in vals])
    return bytes(frame)


# ==============================================================
# ファーム用の書き出しは2種類のファイル。
#   - ExportData.h : モーションだけ（モーション名の namespace）。Motion.h で DECLARE_DATA_MOTION して使う。
#                    値は論理ICS。1行 = 1制御周期（1000/FPS ms）。-1 = その軸は触らない、0 = FREE。
#   - CalibData.h  : 校正値だけ（calib_data）。機体ごとの値。ファームとPCアプリの両方がこの形式を読む。
#   アプリデータ（.json）の保存とは別物。ここはファームに組み込むための書き出し。
# ==============================================================
EXPORT_FILE_NAME = "ExportData.h"
CALIB_FILE_NAME = "CalibData.h"
CALIB_FORMAT = 1          # CalibData.h の FORMAT（firmware の src/CalibData.h と合わせる）
OFFSET_MIN, OFFSET_MAX = 3500, 11500
# モーション名はそのまま C++ の namespace 名になるので、使える文字と予約語を確認する
MOTION_NAME_RE = re.compile(r'^[A-Za-z_][A-Za-z0-9_]{0,62}$')
CPP_KEYWORDS = set("""alignas alignof and and_eq asm auto bitand bitor bool break case catch char char8_t char16_t char32_t class
compl concept const consteval constexpr constinit const_cast continue co_await co_return co_yield decltype default delete do
double dynamic_cast else enum explicit export extern false float for friend goto if inline int long mutable namespace new
noexcept not not_eq nullptr operator or or_eq private protected public register reinterpret_cast requires return short signed
sizeof static static_assert static_cast struct switch template this thread_local throw true try typedef typeid typename union
unsigned using virtual void volatile wchar_t while xor xor_eq""".split())
# ファーム側で既に使っている名前（衝突するとビルドできない）
RESERVED_NAMES = {"calib_data", "export_data", "ExportFrame", "motion", "motionList", "proto", "ctrl", "calib", "servoICS",
                  "servoBus", "Config", "std", "Serial", "Serial1", "Wire", "Dualshock4", "ServoArray", "leftFoot", "rightFoot",
                  "setup", "loop", "loop2",
                  # Motion.h の namespace motion にあるクラス名（DECLARE_DATA_MOTION の中では、こちらが先に見つかってしまう）
                  "walk1", "walkBack", "walkY", "turn", "attack_Medium_left",
                  "attack_Medium_right", "attack_Heavy_1", "attack_Heavy_2", "zero", "nop", "LOCK_DebugMode", "pose",
                  "getUp", "getUp_supine", "getUp_prone", "sit_stand", "lowerPos", "hip"}
PART_INFO = [("hip", "腰", "HIP_BIT", (0,)), ("l_arm", "左腕", "LEFT_ARM_BIT", (1, 2, 3, 4)), ("r_arm", "右腕", "RIGHT_ARM_BIT", (5, 6, 7, 8)),
             ("l_leg", "左脚", "LEFT_FOOT_BIT", (9, 10, 11, 12, 13)), ("r_leg", "右脚", "RIGHT_FOOT_BIT", (14, 15, 16, 17, 18))]


def fnv1a32(data):
    h = 2166136261
    for b in data:
        h = ((h ^ b) * 16777619) & 0xFFFFFFFF
    return h


def hash16(data):
    """ ファームの uploadHash16() と同じ（ACKのtokenと照合する） """
    h = fnv1a32(data)
    return (h ^ (h >> 16)) & 0xFFFF


def motion_name_error(name):
    """ C++ の名前として使えなければ理由を返す（使えれば None） """
    if not isinstance(name, str) or not name:
        return "名前が空です。"
    if not MOTION_NAME_RE.match(name):
        return "英字・数字・_ だけで、先頭は英字か _ にしてください（63文字まで）。"
    if name in CPP_KEYWORDS:
        return f"「{name}」は C++ の予約語なので使えません。"
    if name in RESERVED_NAMES:
        return f"「{name}」はファームで使っている名前なので使えません。"
    if name.startswith("__") or re.match(r'^_[A-Z]', name):
        return "「__」や「_大文字」で始まる名前は C++ で予約されています。"
    return None


def _check_calib(offsets, foot_l, foot_r):
    offsets = [int(v) for v in offsets]
    if len(offsets) != 19:
        raise ValueError("オフセットは19軸分必要です。")
    for i, v in enumerate(offsets):
        if not (OFFSET_MIN <= v <= OFFSET_MAX):
            raise ValueError(f"軸{i}のオフセット {v} が範囲外です ({OFFSET_MIN}〜{OFFSET_MAX})。")
    foot_l, foot_r = float(foot_l), float(foot_r)
    for v in (foot_l, foot_r):
        if not math.isfinite(v) or abs(v) > 100:
            raise ValueError("左右差補正は -100〜100 mm にしてください。")
    return offsets, foot_l, foot_r


def _cfloat(v):
    t = repr(float(v))
    return t + "f"


def generate_export_header(fps, motions):
    """ fps: 制御周期, motions: [{"name": C++名, "desc": 説明文, "rows": [[19]...]}] """
    fps = int(fps)
    if not (1 <= fps <= 1000):
        raise ValueError("FPS が範囲外です。")
    names = set()
    blocks = []
    for m in motions:
        name = m.get("name", "")
        err = motion_name_error(name)
        if err:
            raise ValueError(f"モーション名「{name}」: {err}")
        if name in names:
            raise ValueError(f"モーション名「{name}」が重複しています。")
        names.add(name)
        rows = m.get("rows") or []
        if not (1 <= len(rows) <= 65535):
            raise ValueError(f"「{name}」のフレーム数が範囲外です ({len(rows)})。")
        used = set()
        lines = []
        for r in rows:
            if len(r) != 19:
                raise ValueError(f"「{name}」: 1行は19軸分必要です。")
            vals = [int(v) for v in r]
            for i, v in enumerate(vals):
                if not (v in (-1, 0) or OFFSET_MIN <= v <= OFFSET_MAX):
                    raise ValueError(f"「{name}」: 軸{i}の値 {v} が範囲外です。")
                if v != -1:
                    used.add(i)
            lines.append("    {{" + ", ".join(f"{v:5d}" for v in vals) + "}},")
        parts = [(label, bit) for _, label, bit, axes in PART_INFO if any(a in used for a in axes)]
        sec = len(rows) / fps
        desc_lines = [l.strip() for l in str(m.get("desc", "")).splitlines() if l.strip()] or ["（なし）"]
        head = [f"// <モーション名> {name}", f"// <説明文> {desc_lines[0]}"] + [f"//          {l}" for l in desc_lines[1:]]
        bits = "ALL_BIT" if len(parts) == len(PART_INFO) else (" | ".join(p[1] for p in parts) or "NON_BIT")
        head.append(f"// 部位: {'、'.join(p[0] for p in parts) or 'なし'}（部位ビットの例: {bits}）"
                    f" / {sec:.2f} 秒 = {len(rows)} フレーム")
        blocks.append("\n".join(head) + f"\nnamespace {name} {{\ninline const ExportFrame flameData[] = {{\n" + "\n".join(lines) +
                      f"\n}};\ninline const uint16_t flameNum = {len(rows)};\n}} // namespace {name}")
    motion_text = ("// 使い方（Motion.h）: DECLARE_DATA_MOTION(クラス名, モーション名, 部位ビット)\n\n" + "\n\n".join(blocks)) if blocks \
        else "// （まだありません）"
    return f"""#pragma once
// ==============================================================
// ファーム用エクスポートデータ（PCアプリの「エクスポート (.h)」がこのファイルを丸ごと書き換える）
//  - モーション（モーション名の namespace）だけ。校正値は CalibData.h（別ファイル）。
//  - モーションは Motion.h で DECLARE_DATA_MOTION(クラス名, モーション名, 部位ビット) と宣言して使う。
//  - モーションの値は論理ICS（7500 = 校正済みの中立, 0 = FREE, -1 = その軸は触らない）。
//    1行 = 1制御周期（1000 / FPS ミリ秒）。
// ==============================================================
#include <stdint.h>

struct ExportFrame {{ int16_t v[19]; }};

namespace export_data {{
inline constexpr uint16_t FPS = {fps};   // 書き出したときの制御周期（Config の G_fps と同じにすること）
}}

// ---- モーション ----
{motion_text}
"""


def generate_calib_header(calib):
    """ 校正データ（CalibData.h）。ファームの src/CalibData.h と同じ形式。
        calib: {"offsets":[19], "footL":, "footR":} """
    offsets, foot_l, foot_r = _check_calib(calib["offsets"], calib["footL"], calib["footR"])
    def g(a, b):
        return ", ".join(str(v) for v in offsets[a:b])
    return f"""#pragma once
// ==============================================================
// 校正データ（PCアプリの「校正を書き出し (CalibData.h)」がこのファイルを丸ごと書き換える）
//  - 機体ごとの値。ファームとPCアプリの両方がこの形式を読む（PCアプリは「CalibData.h を開く」で読み込む）。
//  - 手で編集しない。値の範囲: OFFSET は {OFFSET_MIN}〜{OFFSET_MAX}、FOOTDEF は -100〜100 [mm]。
//  - FORMAT を変えたら、PCアプリ（PC_App/main.py の CALIB_FORMAT）も合わせること。
// ==============================================================
#include <stdint.h>

namespace calib_data {{
inline constexpr int FORMAT = {CALIB_FORMAT};
// サーボごとのオフセット（生ICSで論理7500=中立になる値。範囲 {OFFSET_MIN}〜{OFFSET_MAX}）
inline constexpr int16_t OFFSET[19] = {{
    {g(0, 1)},                           // [0] 腰
    {g(1, 5)},         // [1..4] 左腕
    {g(5, 9)},         // [5..8] 右腕
    {g(9, 14)},   // [9..13] 左脚
    {g(14, 19)}    // [14..18] 右脚
}};
// 左右の脚の高さ補正 [mm]
inline constexpr float FOOTDEF_L = {_cfloat(foot_l)};
inline constexpr float FOOTDEF_R = {_cfloat(foot_r)};
}} // namespace calib_data
"""


def parse_calib_from_header(text):
    """ CalibData.h（旧 ExportData.h の calib_data も可）から校正値を読む。読めなければ None """
    fmt = re.search(r'FORMAT\s*=\s*(\d+)', text)
    if fmt and int(fmt.group(1)) != CALIB_FORMAT:
        return None                                # 形式が違うファイルは読まない（値の取り違え防止）
    m = re.search(r'OFFSET\s*\[\s*19\s*\]\s*=\s*\{([^}]*)\}', text)
    l = re.search(r'FOOTDEF_L\s*=\s*([-+0-9.eE]+)f?', text)
    r = re.search(r'FOOTDEF_R\s*=\s*([-+0-9.eE]+)f?', text)
    if not (m and l and r):
        return None
    body = re.sub(r'//[^\n]*', '', m.group(1))
    vals = [int(v) for v in re.findall(r'-?\d+', body)]
    if len(vals) != 19:
        return None
    try:
        offsets, foot_l, foot_r = _check_calib(vals, l.group(1), r.group(1))
    except ValueError:
        return None
    return {"offsets": offsets, "footL": foot_l, "footR": foot_r}


class Api:
    def __init__(self):
        self.client = None
        self.is_connected = False
        self.connected_device_name = ""

        self.RX_UUID = "6e400002b5a3f393e0a9e50e24dcca9e"
        self.TX_UUID = "6e400003b5a3f393e0a9e50e24dcca9e"

        # スレッド間で共有するキュー。STATE/FBは最新値だけを保持（UIが遅れても溜まらない）
        self.event_queue = deque()
        self.latest = {}
        self.latest_lock = threading.Lock()
        self.seq = 0
        self.pending = {}          # (type, seq) -> Future
        self.upload_active = False
        self.realtime_active = False
        self.realtime_stop_flag = False
        self.state_flags = 0       # 直近のSTATEのflags（latestはUIが取り出すと消えるので、別に持つ）
        self.last_upload = None    # 直近にマイコンへ転送した内容 {"hash":int16bit, "frames":n}（接続ごとに忘れる）
        # 接続の世代番号。古い接続からの切断通知・受信データは無視する
        # （旧実装: 接続/切断を繰り返すと、前の接続の「切断」通知が新しい接続の表示を上書きし、
        #   「切断」表示なのに実際は接続中でデータも流れる状態になっていた）
        self.gen = 0
        self.conn_state = "disconnected"   # disconnected / connecting / connected / disconnecting
        self.conn_lock = None
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self._run_loop, daemon=True).start()

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.conn_lock = asyncio.Lock()
        self.loop.run_forever()

    def _push_event(self, event_type, payload):
        self.event_queue.append({"type": event_type, "payload": payload})

    def poll_events(self):
        events = []
        while True:
            try:
                events.append(self.event_queue.popleft())
            except IndexError:
                break
        with self.latest_lock:
            for k, v in self.latest.items():
                events.append({"type": k, "payload": v})
            self.latest.clear()
        return events

    def log(self, message):
        logger.info(message)
        self._push_event("log", message)

    # --- BLE 接続 ---
    def _set_conn(self, state, name=""):
        self.conn_state = state
        self.is_connected = state == "connected"
        self.connected_device_name = name if state == "connected" else ""
        self._push_event("connection_status", {"state": state, "connected": self.is_connected, "name": name})

    def get_connection_state(self):
        """ UIの定期確認用（表示と実際のずれを防ぐ） """
        real = bool(self.client and self.client.is_connected)
        if self.conn_state == "connected" and not real:
            self._set_conn("disconnected")
        elif self.conn_state == "disconnected" and real:
            self._set_conn("connected", self.connected_device_name or "ESP32_Robot")
        return {"state": self.conn_state, "name": self.connected_device_name}

    def connect_target(self, name):
        """ 名前で探して接続（スキャン+接続を1操作で）。接続処理中の二重実行はしない """
        if self.conn_state != "disconnected":
            return False
        self._set_conn("connecting")             # 呼び出し直後から接続中扱い（二度押し・表示のちらつき防止）
        asyncio.run_coroutine_threadsafe(self._connect_by_name(name), self.loop)
        return True

    async def _connect_by_name(self, name):
        self._set_conn("connecting")
        try:
            self.log(f"[BLE] {name} を探しています...")
            dev = await BleakScanner.find_device_by_name(name, timeout=8.0)
        except Exception as e:
            dev = None
            self.log(f"[ERROR] スキャン失敗: {e}{_ble_hint(e)}")
        if dev is None:
            self.log(f"[BLE] {name} が見つかりませんでした。電源と広告状態を確認してください。")
            self._set_conn("disconnected")
            return
        await self._connect(dev.address, dev.name or name)

    def _make_disconnect_cb(self, gen):
        def cb(client):
            # 古い接続（世代が違う）からの通知は無視する
            if gen != self.gen:
                return
            self.loop.call_soon_threadsafe(self._on_lost, gen)
        return cb

    def _on_lost(self, gen):
        if gen != self.gen:
            return
        self.client = None
        for fut in self.pending.values():
            if not fut.done():
                fut.set_exception(ConnectionError("切断"))
        self.pending.clear()
        self.last_upload = None
        self.state_flags = 0
        if self.conn_state != "disconnecting":
            self.log("[BLE] 切断されました。")
        self._set_conn("disconnected")

    async def _connect(self, address, name):
        async with self.conn_lock:
            await self._close_client()             # 残っている接続は必ず先に閉じる
            self.gen += 1
            gen = self.gen
            self._set_conn("connecting")
            client = BleakClient(address, disconnected_callback=self._make_disconnect_cb(gen))
            try:
                await client.connect()
                await client.start_notify(self.TX_UUID, lambda s, d, g=gen: self._notification_handler(s, d, g))
                self.client = client
                self.seq = 0
                self.last_upload = None
                self.state_flags = 0
                self._set_conn("connected", name)
                self.log(f"[SUCCESS] {name} ({address}) に接続しました！")
                await self._send(HELLO, b"", True)   # → HELLO_R + CALIB
            except Exception as e:
                self.log(f"[ERROR] 接続失敗: {e}{_ble_hint(e)}")
                try:
                    await client.disconnect()        # 半端に繋がった接続を残さない
                except Exception:
                    pass
                if gen == self.gen:
                    self.client = None
                    self._set_conn("disconnected")

    async def _close_client(self):
        c = self.client
        self.client = None
        self.gen += 1                            # 以後この接続からの通知は無視
        if c is not None:
            try:
                try:
                    await c.stop_notify(self.TX_UUID)
                except Exception:
                    pass
                await c.disconnect()
            except Exception as e:
                self.log(f"[BLE] 切断処理でエラー: {e}")

    def disconnect_ble(self):
        asyncio.run_coroutine_threadsafe(self._disconnect_task(), self.loop)

    async def _disconnect_task(self):
        async with self.conn_lock:
            self._set_conn("disconnecting")
            await self._close_client()
            for fut in self.pending.values():
                if not fut.done():
                    fut.set_exception(ConnectionError("切断"))
            self.pending.clear()
            self._set_conn("disconnected")
            self.log("[SUCCESS] BLE 切断完了。")

    # --- 送信の共通処理 ---
    async def _send(self, ptype, payload, response):
        if not (self.client and self.client.is_connected):
            raise ConnectionError("BLEデバイスが接続されていません。")
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFF
        await self.client.write_gatt_char(self.RX_UUID, bytes([ptype, seq]) + bytes(payload), response=response)
        return seq

    async def _request(self, ptype, payload=b"", timeout=3.0):
        """ ACK付きコマンド。(type, seq)で照合したstatusを返す """
        if not (self.client and self.client.is_connected):
            raise ConnectionError("BLEデバイスが接続されていません。")
        seq = self.seq
        fut = self.loop.create_future()
        self.pending[(ptype, seq)] = fut
        try:
            await self._send(ptype, payload, True)
            status, token = await asyncio.wait_for(fut, timeout)
            return status, token
        finally:
            self.pending.pop((ptype, seq), None)

    def _call(self, coro, timeout=6.0):
        return asyncio.run_coroutine_threadsafe(coro, self.loop).result(timeout=timeout)

    def _cmd(self, ptype, payload=b"", timeout=3.0):
        """ JSから呼ぶ同期版。戻り値: {"ok":bool, "status":int, "text":str} """
        try:
            status, _ = self._call(self._request(ptype, payload, timeout), timeout + 1)
            return {"ok": status == 0, "status": status, "text": STATUS_TEXT.get(status, str(status))}
        except Exception as e:
            self.log(f"[TX ERROR] {e}")
            return {"ok": False, "status": -1, "text": str(e)}

    def _fire(self, ptype, payload=b"", reliable=False):
        """ ACK無しの送信（ストリーム系）。reliable=Trueなら書き込み応答ありで確実に届ける """
        try:
            self._call(self._send(ptype, payload, bool(reliable)), 3.0)
            return True
        except Exception as e:
            self.log(f"[TX ERROR] {e}")
            return False

    # --- 受信 ---
    def _notification_handler(self, sender, data, gen=None):
        if gen is not None and gen != self.gen:
            return                               # 古い接続からのデータは捨てる
        data = bytes(data)
        if len(data) < 2:
            return
        t, p = data[0], data[2:]
        if t == ACK and len(p) >= 5:
            fut = self.pending.get((p[0], p[1]))
            if fut and not fut.done():
                fut.set_result((p[2], p[3] | (p[4] << 8)))
        elif t == STATE and len(p) == 43:
            st = {"mode": p[0], "flags": p[1], "err": p[2] | (p[3] << 8) | (p[4] << 16),
                  "cmd": list(struct.unpack_from('<19H', p, 5))}
            self.state_flags = st["flags"]
            with self.latest_lock:
                self.latest["state"] = st
        elif t == FB and len(p) == 41:
            fb = {"valid": p[0] | (p[1] << 8) | (p[2] << 16), "pos": list(struct.unpack_from('<19H', p, 3))}
            with self.latest_lock:
                self.latest["fb"] = fb
        elif t == HELLO_R and len(p) >= 5:
            self._push_event("hello", {"version": p[0], "mode": p[1], "frameBytes": p[2],
                                       "maxFrames": p[3] | (p[4] << 8), "expected": PROTO_VERSION,
                                       "fps": p[5] if len(p) >= 6 else 40})
        elif t == CALIB and len(p) == 47:
            offs = list(struct.unpack_from('<19H', p, 0))
            l, r = struct.unpack_from('<2f', p, 38)
            self._push_event("calib", {"offsets": offs, "footL": l, "footR": r, "dirty": bool(p[46])})
        elif t == POSE_ECHO and len(p) == 25:
            self._push_event("pose_echo", {"foot": p[0], "v": list(struct.unpack_from('<6f', p, 1))})
        elif t == LINK and len(p) == 59:
            axes = [{"rate": p[2 + i * 3], "rttUs": p[3 + i * 3] * 20, "rej": p[4 + i * 3]} for i in range(19)]
            with self.latest_lock:
                self.latest["link"] = {"timeoutUs": p[0] * 100, "late": p[1], "axes": axes}
        elif t == EVENT and len(p) >= 1:
            self._push_event("motion_event", p[0])

    # --- JSから呼ぶAPI ---
    def set_mode(self, tx):
        return self._cmd(SET_MODE, bytes([1 if tx else 0]))

    def heartbeat(self):
        return self._fire(HEARTBEAT)

    def emergency_stop(self, servo_mask):
        m = int(servo_mask) & 0x7FFFF
        return self._cmd(ESTOP, m.to_bytes(3, 'little'))

    def emergency_clear(self):
        return self._cmd(ESTOP_CLEAR)

    def send_teach_frame(self, cmd_mask, fb_mask, angles, reliable=False):
        """ 軸指令 [cmd:3][fb:3][19*u16 論理値]。0=FREE """
        payload = (int(cmd_mask) & 0x7FFFF).to_bytes(3, 'little') + (int(fb_mask) & 0x7FFFF).to_bytes(3, 'little')
        payload += struct.pack('<19H', *[max(0, min(65535, int(angles[i]) if i < len(angles) else 0)) for i in range(19)])
        return self._fire(AXIS_CMD, payload, reliable)

    def send_pose(self, foot, mode, x, y, z, roll_deg, yaw_deg, reliable=False):
        """ 座標(IK)指令。逆運動学はマイコンで計算。X/Y/Z[mm]、角度[deg] """
        payload = struct.pack('<BB6f', int(foot), int(mode), float(x), float(y), float(z),
                              math.radians(float(roll_deg)), 0.0, math.radians(float(yaw_deg)))
        return self._fire(POSE_CMD, payload, reliable)

    def request_neutral(self, foot):
        return self._cmd(NEUTRAL_REQ, bytes([int(foot)]))

    def cal_zero_here(self, axis):
        return self._cmd(CAL_ZERO_HERE, bytes([int(axis)]))

    def cal_set_footdef(self, left, right):
        return self._cmd(CAL_SET_FOOTDEF, struct.pack('<2f', float(left), float(right)))

    def cal_defaults(self):
        return self._cmd(CAL_DEFAULTS)

    def cal_set_offset(self, axis, offset):
        """ 1軸のオフセット（生ICS, 3500〜11500）を直接設定。画面の一覧で数値を打ち込んだときに使う """
        axis, offset = int(axis), int(offset)
        if not (0 <= axis < 19):
            return {"ok": False, "status": -1, "text": "軸番号が範囲外です。"}
        if not (OFFSET_MIN <= offset <= OFFSET_MAX):
            return {"ok": False, "status": -1, "text": f"オフセットは {OFFSET_MIN}〜{OFFSET_MAX} にしてください。"}
        return self._cmd(CAL_SET_OFFSET, struct.pack('<BH', axis, offset))

    def cal_set_all(self, calib):
        """ 全校正値を一括設定（CalibData.h の読み込み）。calib: {"offsets":[19], "footL":, "footR":} """
        try:
            offsets, foot_l, foot_r = _check_calib(calib["offsets"], calib["footL"], calib["footR"])
        except (ValueError, KeyError, TypeError) as e:
            return {"ok": False, "status": -1, "text": str(e)}
        payload = struct.pack('<19H2f', *offsets, foot_l, foot_r)
        return self._cmd(CAL_SET_ALL, payload)

    def cal_zero_all(self):
        """ 全軸を今の姿勢で中立(7500)にする。サーボの応答待ちがあるので時間を長めに取る """
        return self._cmd(CAL_ZERO_ALL, timeout=8.0)

    # --- マイコンへの事前転送 + 再生 ---
    def upload_and_play(self, keyframes):
        if self.upload_active:
            self.log("[MCU] 転送中です。")
            return False
        asyncio.run_coroutine_threadsafe(self._upload_task(keyframes), self.loop)
        return True

    async def _req_ok(self, ptype, payload=b"", timeout=3.0, retries=2):
        """ ACKがOKになるまで（タイムアウト時は同じseq系列で再送） """
        last = None
        for _ in range(retries + 1):
            try:
                status, token = await self._request(ptype, payload, timeout)
                if status == 0:
                    return token
                raise RuntimeError(f"0x{ptype:02X}: {STATUS_TEXT.get(status, status)}")
            except asyncio.TimeoutError as e:
                last = e
        raise RuntimeError(f"0x{ptype:02X}: ACKがありません") from last

    def _mcu_has_upload(self):
        """ マイコンがRAMに再生可能なモーションを持っているか（STATEのフラグ） """
        return bool(self.state_flags & FL_UPLOAD_READY)

    def _upload_chunk_size(self):
        """ BLEのMTUから1回に送れるデータ量を決める（ATTヘッダ3 + type/seq/offset 4 を引く） """
        mtu = getattr(self.client, "mtu_size", None) or 23
        return UP_DATA_MAX if mtu >= UP_DATA_MAX + 7 else CHUNK

    async def _upload_chunks(self, data, chunk):
        """ UP_DATA を UP_WINDOW 件ずつ同時に待つ。マイコンは受信順に処理するので、offset は必ず昇順に送る。
            進み具合は upload_progress で画面に出す。 """
        offsets = list(range(0, len(data), chunk))
        total = len(offsets)
        done = 0
        next_report = 25
        self._push_event("upload_progress", {"pct": 0})
        inflight = []
        try:
            for off in offsets:
                inflight.append(asyncio.ensure_future(self._req_ok(UP_DATA, off.to_bytes(2, 'little') + data[off:off + chunk])))
                if len(inflight) >= UP_WINDOW:
                    await inflight.pop(0)
                    done += 1
                    pct = done * 100 // total
                    self._push_event("upload_progress", {"pct": pct})
                    if pct >= next_report:
                        self.log(f"[MCU] 転送 {pct}%")
                        next_report += 25
            while inflight:
                await inflight.pop(0)
                done += 1
                self._push_event("upload_progress", {"pct": done * 100 // total})
        except BaseException:
            for fut in inflight:
                fut.cancel()
            raise

    async def _upload_task(self, keyframes):
        self.upload_active = True
        self._push_event("upload_state", {"busy": True})
        skipped = False
        ok = False
        try:
            n = len(keyframes)
            if n == 0 or n > MAX_FRAMES:
                raise RuntimeError(f"フレーム数が範囲外です ({n}。1〜{MAX_FRAMES})")
            data = b"".join(build_frame(kf) for kf in keyframes)
            h = hash16(data)
            key = {"hash": h, "frames": n}
            # 前回送った内容と同じで、マイコンにも残っているなら、転送を省略して再生だけ送る
            if self.last_upload == key and self._mcu_has_upload():
                try:
                    await self._req_ok(UP_START, b"", 5.0, 0)
                    skipped = True
                    self.log(f"[MCU] 変更なし: 転送を省略して再生します ({n} フレーム)")
                except RuntimeError:
                    self.last_upload = None        # 残っていなかった → 通常の転送に切り替える
            if not skipped:
                self.last_upload = None
                await self._req_ok(UP_BEGIN, n.to_bytes(2, 'little'))
                await self._upload_chunks(data, self._upload_chunk_size())
                token = await self._req_ok(UP_COMMIT, b"", 10.0, 0)   # 検証(IK含む)。RAMに保持するだけ
                if token != h:
                    raise RuntimeError("マイコンが受け取った内容が送った内容と一致しません。もう一度お試しください。")
                self.last_upload = key
                await self._req_ok(UP_START, b"", 5.0, 0)
                self.log(f"[MCU] 再生開始: {n} フレーム")
            ok = True
        except Exception as e:
            self.log(f"[MCU ERROR] {e}")
        finally:
            self.upload_active = False
            self._push_event("upload_state", {"busy": False, "skipped": skipped, "ok": ok})

    # --- リアルタイム再生（PCから1コマずつ送る） ---
    def realtime_play(self, frames, fps):
        """ frames: [{"axes": [19] 論理ICS（-1=触らない, 0=FREE）, "poses": [{foot, ik, x, y, z, roll, yaw}]}]
            1コマ送って ACK を待ち、次は「1周期後」か「ACKの後」のどちらか遅い方に送る。
            マイコンが追いつかなければ自動的にスローになるが、全コマを順番どおり実行させる。 """
        if self.realtime_active:
            self.log("[RT] 再生中です。")
            return False
        if self.upload_active:
            self.log("[RT] マイコンへ書き込み中のため、リアルタイム再生は開始できません。")
            return False
        fps = int(fps)
        if not (1 <= fps <= 100):
            self.log("[RT] FPS が範囲外です。")
            return False
        asyncio.run_coroutine_threadsafe(self._realtime_task(frames, fps), self.loop)
        return True

    def realtime_stop(self):
        self.realtime_stop_flag = True
        return True

    async def _send_teach(self, ptype, payload):
        """ ACKを待つ。キューが満杯（ST_BUSY）なら少し待って送り直す。それ以外の失敗は例外にする """
        for _ in range(2000):
            status, _token = await self._request(ptype, payload, 2.0)
            if status == 0:
                return
            if status != 5:   # 5 = 処理中（満杯）
                raise RuntimeError(f"0x{ptype:02X}: {STATUS_TEXT.get(status, status)}")
            await asyncio.sleep(0.004)
        raise RuntimeError("マイコンが受け取れない状態が続きました（キュー満杯）")

    @staticmethod
    def _axis_payload(axes):
        """ TEACH_FRAME: [cmd:3][fb:3][19*u16]。-1 の軸は cmd に入れない（触らない） """
        mask = 0
        vals = []
        for i, v in enumerate(axes):
            if v == -1:
                vals.append(0)
            else:
                mask |= 1 << i
                vals.append(max(0, min(65535, int(v))))
        return mask.to_bytes(3, 'little') + (0).to_bytes(3, 'little') + struct.pack('<19H', *vals)

    @staticmethod
    def _pose_payload(p):
        """ TEACH_POSE: [foot][ik][6*f32]（POSE_CMD と同じ形式、角度は度で受け取る） """
        return struct.pack('<BB6f', int(p["foot"]), int(p.get("ik", 0)), float(p["x"]), float(p["y"]), float(p["z"]),
                           math.radians(float(p["roll"])), 0.0, math.radians(float(p["yaw"])))

    async def _realtime_task(self, frames, fps):
        self.realtime_active = True
        self.realtime_stop_flag = False
        total = len(frames)
        sent = 0
        period = 1.0 / fps
        loop = asyncio.get_running_loop()
        last = None
        self._push_event("realtime_state", {"running": True, "sent": 0, "total": total})
        try:
            for fr in frames:
                if self.realtime_stop_flag:
                    break
                if last is not None:
                    wait = last + period - loop.time()
                    if wait > 0:
                        await asyncio.sleep(wait)
                last = loop.time()
                if any(v != -1 for v in fr["axes"]):
                    await self._send_teach(TEACH_FRAME, self._axis_payload(fr["axes"]))
                for p in fr.get("poses", []):
                    await self._send_teach(TEACH_POSE, self._pose_payload(p))
                sent += 1
                if sent % 3 == 0 or sent == total:
                    self._push_event("realtime_state", {"running": True, "sent": sent, "total": total})
        except Exception as e:
            self.log(f"[RT ERROR] {e}")
        finally:
            self.realtime_active = False
            stopped = self.realtime_stop_flag
            self._push_event("realtime_state", {"running": False, "sent": sent, "total": total, "stopped": stopped})
            self.log(f"[RT] {'中断しました' if stopped else '終了'}: {sent}/{total} コマ送信")

    # --- ファーム用エクスポート（ExportData.h = モーション / CalibData.h = 校正値） ---
    def check_motion_name(self, name):
        return motion_name_error(name)

    def _build_export(self, kind, payload):
        """ 書き出す内容を作る。kind: "motions"（ExportData.h）| "calib"（CalibData.h） """
        if kind == "calib":
            calib = payload.get("calib")
            if not calib:
                raise ValueError("校正値がありません。ロボットに接続するか、CalibData.h を開いてください。")
            return generate_calib_header(calib)
        return generate_export_header(payload.get("fps", 40), payload.get("motions") or [])

    def export_header_to(self, path, payload):
        """ ダイアログ無しでファイルへ直接書く版（テストにも使う）。payload: {fps, motions} """
        text = self._build_export("motions", payload)
        old_text = None
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as f:
                old_text = f.read()
        if old_text == text:
            return {"ok": True, "path": path, "result": "unchanged", "text": "OK"}
        if old_text is not None:
            with open(path + ".bak", 'w', encoding='utf-8', newline='\n') as f:   # 1世代だけ退避
                f.write(old_text)
        tmp = path + ".tmp"
        with open(tmp, 'w', encoding='utf-8', newline='\n') as f:
            f.write(text)
        os.replace(tmp, path)                      # 書き込み途中で落ちても壊れたファイルを残さない
        return {"ok": True, "path": path, "result": "written", "text": "OK"}

    def export_header_build(self, payload, existing_text=None, file_name=EXPORT_FILE_NAME):
        """ モーション（ExportData.h）を画面から書き出す版。保存先は画面（ブラウザ）が選び、書き込みも画面が行う。
            existing_text: 保存先の今の内容（新規・空なら None/""）。
            戻り値: {ok, result, content(書く内容), bakPath(元の内容の退避先), text} """
        return self._export_build("motions", payload, existing_text, file_name)

    def export_calib_build(self, payload, existing_text=None, file_name=CALIB_FILE_NAME):
        """ 校正値（CalibData.h）を画面から書き出す版。payload: {calib: {offsets, footL, footR}} """
        return self._export_build("calib", payload, existing_text, file_name)

    def _export_build(self, kind, payload, existing_text, file_name):
        try:
            # 空のファイル = 新規（保存先を選んだ時点でブラウザが空のファイルを作ることがあるため）
            old_text = existing_text if existing_text else None
            text = self._build_export(kind, payload)
            result = "unchanged" if old_text == text else "written"
            bak = None
            if result == "written" and old_text is not None:
                bak = self._save_export_backup(file_name, old_text)
            return {"ok": True, "result": result, "content": text, "bakPath": bak, "text": "OK"}
        except Exception as e:
            self.log(f"[EXPORT] 書き出せませんでした: {e}")
            return {"ok": False, "text": str(e)}

    def calib_parse(self, text):
        """ 画面の「CalibData.h を開く」。ロボットに接続しなくても、ファイルの校正値を画面に出して書き出せる。 """
        calib = parse_calib_from_header(text or "")
        if not calib:
            return {"ok": False, "text": "CalibData.h として読めませんでした（OFFSET・FOOTDEF の形式か、FORMAT を確認してください）。"}
        return {"ok": True, "calib": calib, "text": "OK"}

    def _save_export_backup(self, file_name, old_text):
        """ 上書き前の内容を DATA_DIR/backup/<名前>.bak に1世代だけ退避（ブラウザからは保存先の隣に書けないため） """
        try:
            d = os.path.join(DATA_DIR, "backup")
            os.makedirs(d, exist_ok=True)
            name = os.path.basename(str(file_name or EXPORT_FILE_NAME)) or EXPORT_FILE_NAME
            path = os.path.join(d, name + ".bak")
            tmp = path + ".tmp"
            with open(tmp, 'w', encoding='utf-8', newline='\n') as f:
                f.write(old_text)
            os.replace(tmp, path)
            return path
        except OSError as e:
            self.log(f"[EXPORT] 元の内容を退避できませんでした（書き出しは続けます）: {e}")
            return None

    def open_log_folder(self):
        """ 設定・ログの保存先をエクスプローラーで開く """
        try:
            if hasattr(os, "startfile"):
                os.startfile(DATA_DIR)              # Windows
            else:
                opener = shutil.which("xdg-open") or shutil.which("open")
                if opener:
                    subprocess.Popen([opener, DATA_DIR], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.log(f"[FILE] フォルダを開けませんでした: {e}")
        return DATA_DIR

    # --- アプリ用JSON（ポーズライブラリ・設定）: main.py と同じフォルダ（書けなければ %APPDATA%）に自動保存 ---
    APP_JSON = {"pose_library": "pose_library.json", "settings": "settings.json"}

    def _app_json_path(self, key):
        if key not in self.APP_JSON:
            raise ValueError("unknown json")
        return os.path.join(DATA_DIR, self.APP_JSON[key])

    def load_app_json(self, key):
        try:
            with open(self._app_json_path(key), 'r', encoding='utf-8') as f:
                return json.load(f)
        except FileNotFoundError:
            return None
        except Exception as e:
            self.log(f"[FILE] {self.APP_JSON.get(key, key)} を読めませんでした: {e}")
            return None

    def save_app_json(self, key, data):
        try:
            path = self._app_json_path(key)
            tmp = path + ".tmp"
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=1)
            os.replace(tmp, path)        # 書き込み途中で落ちても壊れたファイルを残さない
            return True
        except Exception as e:
            self.log(f"[FILE] {self.APP_JSON.get(key, key)} を保存できませんでした: {e}")
            return False


if __name__ == '__main__':
    try:
        import server                            # ローカルサーバー + Edge/Chrome のアプリ表示
        sys.exit(server.main(Api))
    except SystemExit:
        raise
    except BaseException as e:                   # 起動時の想定外の失敗は、日本語のメッセージボックスで知らせる
        logger.exception("起動に失敗しました")
        text = f"起動できませんでした。\n\n{type(e).__name__}: {e}\n\nログ: {LOG_PATH}"
        try:
            import ctypes
            ctypes.windll.user32.MessageBoxW(None, text, "Robot Motion Studio", 0x10)
        except Exception:
            if sys.stderr:
                print(text, file=sys.stderr)
        sys.exit(1)
