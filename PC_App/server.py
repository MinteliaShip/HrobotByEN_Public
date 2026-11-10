# ==============================================================
# ローカルサーバー + Edge/Chrome のアプリ表示（旧 pywebview の代わり）
#   - 127.0.0.1 の空きポートで待ち受け、index.html を配る。
#   - 画面からの POST /api/<名前> {"args":[...]} を Api のメソッドに渡し、戻り値を JSON で返す。
#   - 起動ごとのトークン + Host / Origin の確認で、ほかのサイトからロボットを動かされないようにする。
#   - Edge（無ければ Chrome）の --app で専用ウィンドウを開く。閉じたら（/api/_alive が途絶えたら）終了する。
# ==============================================================
import os
import sys
import json
import time
import hmac
import secrets
import shutil
import socket
import logging
import threading
import subprocess
import webbrowser
import urllib.request
import urllib.parse
import asyncio
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from logging.handlers import RotatingFileHandler

# main.py（python main.py / exe の場合は __main__）の設定値（DATA_DIR など）を使う
_m = sys.modules.get("__main__")
if _m is not None and hasattr(_m, "Api") and hasattr(_m, "DATA_DIR"):
    app = _m
else:
    import main as app

logger = logging.getLogger(app.APP_NAME)

# 画面から呼べるメソッド（これ以外は呼べない）
API_METHODS = {
    "poll_events", "get_connection_state", "connect_target", "disconnect_ble",
    "set_mode", "heartbeat", "emergency_stop", "emergency_clear",
    "send_teach_frame", "send_pose", "request_neutral",
    "cal_zero_here", "cal_set_footdef", "cal_defaults", "cal_set_offset", "cal_set_all", "cal_zero_all",
    "upload_and_play", "realtime_play", "realtime_stop", "check_motion_name", "export_header_build", "export_calib_build", "calib_parse",
    "load_app_json", "save_app_json", "open_log_folder",
}
ALIVE_TIMEOUT_S = 25      # 画面からの生存通知がこれだけ途絶えたら終了（バックグラウンドの間引きに耐える余裕）
FIRST_ALIVE_S = 120       # 起動してから画面が一度も来ないまま、この時間が過ぎたら終了
BYE_GRACE_S = 5           # 画面を閉じた通知のあと、再読み込みでなければ終了するまでの待ち
MAX_BODY = 64 * 1024 * 1024
LOCK_NAME = "instance.json"
PREFERRED_PORTS = range(48573, 48578)   # 127.0.0.1 だけで待ち受ける


# --------------------------------------------------------------
# 起動前の確認・ログ・エラー表示
# --------------------------------------------------------------
def _fix_std_streams():
    """ --windowed の exe では sys.stdout / sys.stderr が None。print や logging で落ちないようにする """
    for name in ("stdout", "stderr"):
        if getattr(sys, name) is None:
            setattr(sys, name, open(os.devnull, "w", encoding="utf-8"))


def _setup_logging():
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    try:
        h = RotatingFileHandler(app.LOG_PATH, maxBytes=1_000_000, backupCount=2, encoding="utf-8")
        h.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
        root.addHandler(h)
    except OSError:
        pass                                   # ログが書けなくても起動は続ける


def message_box(text, title="Robot Motion Studio", error=True):
    """ 致命的な失敗を日本語で知らせる（Windows はメッセージボックス、それ以外はログと標準エラー） """
    logger.error(text) if error else logger.info(text)
    if os.name == "nt":
        try:
            import ctypes
            ctypes.windll.user32.MessageBoxW(None, text, title, 0x10 if error else 0x40)
            return
        except Exception:
            pass
    try:
        print(text, file=sys.stderr)
    except Exception:
        pass


def _running_from_zip():
    """ zip をエクスプローラーで開いたまま起動した（一時フォルダ Temp1_xxx.zip などに展開されている）か """
    p = os.path.abspath(app.APP_DIR).lower() + os.sep
    return (".zip" + os.sep) in p or (os.sep + "temp1_") in p


# --------------------------------------------------------------
# ブラウザ
# --------------------------------------------------------------
def _is_ascii(s):
    try:
        s.encode("ascii")
        return True
    except UnicodeEncodeError:
        return False


def browser_profile_dir():
    """ 専用プロファイルの場所。ASCII 以外の文字（日本語のユーザー名など）を含むとブラウザが失敗することがあるので、
        そのときは ASCII だけのパスへ切り替える。 """
    cands = [os.path.join(app.DATA_DIR, "browser_profile")]
    if os.name == "nt":
        cands.append(os.path.join(os.environ.get("ProgramData", r"C:\ProgramData"), app.APP_NAME, "browser_profile"))
        cands.append(os.path.join(os.environ.get("PUBLIC", r"C:\Users\Public"), app.APP_NAME, "browser_profile"))
    for d in cands:
        if _is_ascii(d) and app._writable_dir(d):
            return d
    d = cands[0]                               # 最後の手段（ASCII でなくても使う）
    app._writable_dir(d)
    logger.warning(f"ブラウザ用フォルダに ASCII 以外の文字が含まれます: {d}")
    return d


def find_browser():
    """ Edge → Chrome → （Linux など）chromium の順に探す。見つからなければ None """
    if os.name == "nt":
        roots = [os.environ.get(k) for k in ("ProgramFiles(x86)", "ProgramFiles", "LOCALAPPDATA")]
        for rel in (r"Microsoft\Edge\Application\msedge.exe", r"Google\Chrome\Application\chrome.exe"):
            for r in roots:
                if r and os.path.isfile(os.path.join(r, rel)):
                    return os.path.join(r, rel)
    for name in ("msedge", "microsoft-edge", "microsoft-edge-stable", "google-chrome", "google-chrome-stable",
                 "chromium", "chromium-browser", "chrome"):
        p = shutil.which(name)
        if p:
            return p
    return None


def open_window(url, profile):
    """ 専用ウィンドウ（--app）で開く。戻り値: (起動したプロセス or None, 専用ウィンドウで開けたか) """
    exe = find_browser()
    if exe:
        args = [exe, f"--app={url}", f"--user-data-dir={profile}", "--no-first-run",
                "--no-default-browser-check", "--window-size=1600,900"]
        try:
            proc = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            logger.info(f"ブラウザを起動しました: {exe}")
            return proc, True
        except OSError as e:
            logger.warning(f"ブラウザを起動できませんでした: {exe}: {e}")
    try:
        webbrowser.open(url + "&b=0")         # 既定のブラウザ（Linux は xdg-open など）
    except Exception as e:
        logger.warning(f"既定のブラウザも開けませんでした: {e}")
    return None, False


# --------------------------------------------------------------
# 二重起動: すでに起動中なら、そのウィンドウをもう一度開いて終わる
# --------------------------------------------------------------
def _lock_path():
    return os.path.join(app.DATA_DIR, LOCK_NAME)


def find_running_instance():
    try:
        with open(_lock_path(), "r", encoding="utf-8") as f:
            info = json.load(f)
        port, token = int(info["port"]), str(info["token"])
        req = urllib.request.Request(f"http://127.0.0.1:{port}/api/_ping", data=b"{}", method="POST",
                                     headers={"Content-Type": "application/json", "X-Token": token,
                                              "Origin": f"http://127.0.0.1:{port}"})
        with urllib.request.urlopen(req, timeout=1.5) as r:
            if json.loads(r.read().decode("utf-8")).get("result", {}).get("app") == app.APP_NAME:
                return port, token
    except Exception:
        pass
    return None


def write_lock(port, token):
    try:
        tmp = _lock_path() + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump({"pid": os.getpid(), "port": port, "token": token}, f)
        os.replace(tmp, _lock_path())
    except OSError:
        pass


def remove_lock(token):
    try:
        with open(_lock_path(), "r", encoding="utf-8") as f:
            if json.load(f).get("token") == token:
                os.remove(_lock_path())
    except Exception:
        pass


# --------------------------------------------------------------
# HTTP サーバー
# --------------------------------------------------------------
class AppState:
    def __init__(self):
        self.started = time.monotonic()
        self.last_alive = None
        self.bye_at = None
        self.stop = threading.Event()

    def alive(self):
        self.last_alive = time.monotonic()
        self.bye_at = None

    def bye(self):
        self.bye_at = time.monotonic()


def make_handler(api, token, state, port_ref):
    index_path = os.path.join(app.RES_DIR, "index.html")

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = "RobotMotionStudio"

        def log_message(self, fmt, *args):     # 標準エラーには出さない
            logger.debug("http: " + fmt % args)

        # ---- 確認 ----
        def _origins(self):
            p = port_ref[0]
            return {f"127.0.0.1:{p}", f"localhost:{p}"}

        def _host_ok(self):
            return self.headers.get("Host", "") in self._origins()

        def _origin_ok(self):
            o = self.headers.get("Origin")
            return o is not None and o in {f"http://{h}" for h in self._origins()}

        def _token_ok(self, query):
            t = self.headers.get("X-Token") or (query.get("t") or [""])[0]
            return hmac.compare_digest(t.encode("utf-8"), token.encode("utf-8"))

        # ---- 応答 ----
        def _send(self, code, body, ctype="application/json; charset=utf-8"):
            data = body if isinstance(body, bytes) else body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Referrer-Policy", "no-referrer")
            self.end_headers()
            self.wfile.write(data)

        def _json(self, code, obj):
            self._send(code, json.dumps(obj, ensure_ascii=False))

        def do_GET(self):
            u = urllib.parse.urlsplit(self.path)
            q = urllib.parse.parse_qs(u.query)
            if not self._host_ok():
                return self._send(403, "Forbidden", "text/plain; charset=utf-8")
            if u.path == "/favicon.ico":
                return self._send(204, b"", "image/x-icon")
            if u.path not in ("/", "/index.html"):
                return self._send(404, "Not Found", "text/plain; charset=utf-8")
            if not self._token_ok(q):
                return self._send(403, "このページは Robot Motion Studio の起動時にだけ開けます。アプリを起動し直してください。",
                                  "text/plain; charset=utf-8")
            try:
                with open(index_path, "rb") as f:
                    html = f.read()
            except OSError as e:
                return self._send(500, f"index.html を読めませんでした: {e}", "text/plain; charset=utf-8")
            self._send(200, html, "text/html; charset=utf-8")

        def do_POST(self):
            u = urllib.parse.urlsplit(self.path)
            q = urllib.parse.parse_qs(u.query)
            try:
                n = int(self.headers.get("Content-Length") or 0)
            except ValueError:
                n = -1
            if n < 0 or n > MAX_BODY:
                self.close_connection = True
                return self._json(413, {"error": "要求が大きすぎます"})
            raw = self.rfile.read(n) if n > 0 else b""   # 拒否する場合も本文は読み切る（同じ接続の次の要求が壊れないように）
            if not (self._host_ok() and self._origin_ok() and self._token_ok(q)):
                logger.warning(f"拒否した要求: {u.path} Host={self.headers.get('Host')} Origin={self.headers.get('Origin')}")
                self.close_connection = True
                return self._json(403, {"error": "Forbidden"})
            if not u.path.startswith("/api/"):
                return self._json(404, {"error": "Not Found"})
            name = u.path[len("/api/"):]
            # ---- アプリ自身の通知（ロボットの heartbeat とは別） ----
            if name == "_alive":
                state.alive()
                return self._json(200, {"result": True})
            if name == "_bye":
                state.bye()
                return self._json(200, {"result": True})
            if name == "_ping":
                return self._json(200, {"result": {"app": app.APP_NAME}})
            if name not in API_METHODS:
                return self._json(404, {"error": f"AttributeError: {name}"})
            try:
                args = json.loads(raw.decode("utf-8") or "{}").get("args") or []
                if not isinstance(args, list):
                    raise TypeError("args must be a list")
                result = getattr(api, name)(*args)
                return self._json(200, {"result": result})
            except Exception as e:
                # 旧 pywebview と同じ見え方（"ValueError: ..."）にする
                logger.exception(f"API {name} でエラー")
                return self._json(500, {"error": f"{type(e).__name__}: {e}"})

    return Handler


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    # Windows の SO_REUSEADDR は「使用中のポートにも重ねて待ち受けられる」意味になるので使わない
    allow_reuse_address = os.name != "nt"


def start_server(api, token=None):
    """ サーバーを別スレッドで起動する（テストからも使う）。戻り値: (httpd, port, token, state) """
    token = token or secrets.token_urlsafe(24)
    state = AppState()
    port_ref = [0]
    httpd = None
    # 毎回同じポートを優先する（画面側が覚える「前回の保存先」はポートごとに別扱いになるため）。使用中なら空きポート。
    for p in list(PREFERRED_PORTS) + [0]:
        try:
            httpd = _Server(("127.0.0.1", p), make_handler(api, token, state, port_ref))
            break
        except OSError:
            continue
    if httpd is None:
        raise OSError("待ち受けに使えるポートがありません")
    port_ref[0] = httpd.server_address[1]
    threading.Thread(target=httpd.serve_forever, name="http", daemon=True).start()
    return httpd, port_ref[0], token, state


def watchdog(state, proc):
    """ 画面が閉じられたら終了。生存通知が途絶え、かつ起動したブラウザも終わっていたら閉じたとみなす。
        （最小化して長く置くと通知が間引かれるので、ブラウザが動いている間は待つ） """
    while not state.stop.wait(1.0):
        now = time.monotonic()
        running = proc is not None and proc.poll() is None
        if state.bye_at is not None and now - state.bye_at > BYE_GRACE_S:
            logger.info("画面が閉じられました。")
            break
        if state.last_alive is None:
            if now - state.started > FIRST_ALIVE_S and not running:
                logger.info("画面が開かれないまま時間が過ぎたので終了します。")
                break
            continue
        silent = now - state.last_alive
        if silent > ALIVE_TIMEOUT_S and not running:
            logger.info("画面からの通知が途絶えました。終了します。")
            break
        if proc is not None and not running and silent > 12:
            logger.info("ブラウザが終了しました。")
            break
    state.stop.set()


def shutdown(api, httpd, token):
    """ BLE を安全に切断し、スレッドを止めて終了する """
    try:
        if api.client is not None:
            asyncio.run_coroutine_threadsafe(api._close_client(), api.loop).result(timeout=5)
    except Exception as e:
        logger.warning(f"切断処理でエラー: {e}")
    try:
        api.loop.call_soon_threadsafe(api.loop.stop)
    except Exception:
        pass
    try:
        httpd.shutdown()
        httpd.server_close()
    except Exception:
        pass
    remove_lock(token)
    logger.info("終了しました。")


def main(Api):
    _fix_std_streams()
    _setup_logging()
    logger.info(f"起動: {app.APP_DIR}（保存先: {app.DATA_DIR}）")
    if _running_from_zip():
        message_box("zip ファイルの中から直接起動されています。\n\n"
                    "zip を右クリック →「すべて展開」で展開してから、展開したフォルダの中の起動ファイルを実行してください。")
        return 1
    if not os.path.isfile(os.path.join(app.RES_DIR, "index.html")):
        message_box(f"画面ファイル index.html が見つかりません。\n{app.RES_DIR}\n\nフォルダの中身をすべてそろえてから起動してください。")
        return 1

    running = find_running_instance()
    if running:
        port, token = running
        logger.info("すでに起動中なので、そのウィンドウを開きます。")
        open_window(f"http://127.0.0.1:{port}/?t={token}", browser_profile_dir())
        return 0

    try:
        api = Api()
        httpd, port, token, state = start_server(api)
    except Exception as e:
        logger.exception("起動に失敗しました")
        message_box(f"起動できませんでした。\n\n{type(e).__name__}: {e}\n\nログ: {app.LOG_PATH}")
        return 1
    write_lock(port, token)
    url = f"http://127.0.0.1:{port}/?t={token}"
    api.log(f"[APP] 設定とログの保存先: {app.DATA_DIR}")
    proc, dedicated = open_window(url, browser_profile_dir())
    if not dedicated:
        api.log("[APP] 専用ウィンドウでは開けませんでした（Edge / Chrome が見つからないか、起動できませんでした）。"
                "通常のブラウザで表示しています。ブラウザのタブを閉じるとアプリも終了します。")
    try:
        watchdog(state, proc)
    except KeyboardInterrupt:
        pass
    shutdown(api, httpd, token)
    return 0
