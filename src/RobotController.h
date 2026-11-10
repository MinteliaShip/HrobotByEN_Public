#pragma once
// ==============================================================
// 通信まわり（旧 RobotController / Protocol / BLEManager をまとめたもの）
//   1. BLEプロトコル定義（proto::）
//   2. BLE送受信（setupBLE / sendPacket / queueAck ...）
//   3. 状態機械とコマンド処理（ctrl::）
// ==============================================================
// ==============================================================
// ロボット全体の状態機械 + BLEコマンドの振り分け + テレメトリ送信
//
//   AUTO ──SET_MODE(1)──▶ TEACH ──SET_MODE(0)──▶ AUTO（緊急停止ラッチ中なら ESTOP）
//    │                      │
//    │◀────UP_START 完了────┤ PLAYBACK（開始前のモードへ戻る）
//    │                      │
//   任意 ──ESTOP──▶ ESTOP ──ESTOP_CLEAR──▶ AUTO
//                   ESTOP ──SET_MODE(1)──▶ TEACH（ラッチは維持。手動操作だけ許可）
//
//  - 状態遷移はすべて loop() 側の ctrl::update() で行う（BLE callbackは要求を積むだけ）。
//  - TEACH 中に PC からのパケットが 1秒途絶えたら、その場の姿勢で保持する（脱力はしない）。
//  - BLE切断: TEACH→AUTO/ESTOP、PLAYBACK→全軸FREE+ESTOP（従来どおり）。
// ==============================================================
#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

// ---------------------------------------------------------------
// 1. BLEプロトコル定義
// ==============================================================
// PC <-> ESP32 BLEプロトコル（PCアプリ main.py と一致させること）
// すべてのパケット: [type:1][seq:1][payload...]
//  - seq はパケットごとに送信側が+1する（0..255で一周）。ACKは要求の type と seq を返すので、
//    PCは「どの要求に対する結果か」を確実に照合できる。
//  - 角度はすべて「論理ICS」（u16 LE）。7500 = 校正済み中立、0 = FREE。
//    生ICS = 論理ICS + (offset - 7500) の変換はマイコンのサーボドライバ内だけで行う。
// ==============================================================

namespace proto {

constexpr uint8_t VERSION = 7;   // v7: リアルタイム再生（TEACH_FRAME/TEACH_POSE: ACK付き・順番どおり全部実行）、書き込みはUP_DATA 48byte。v6: 校正=CalibData.h、モーション=ExportData.h

// ---- PC -> ESP32 ----
constexpr uint8_t HELLO          = 0x01; // []                      → HELLO_R + CALIB
constexpr uint8_t SET_MODE       = 0x02; // [0=AUTO(RX) 1=TEACH(TX)] → ACK
constexpr uint8_t HEARTBEAT      = 0x03; // []                      （TEACH中は300ms毎に送る）
constexpr uint8_t ESTOP          = 0x04; // [mask:3]                → ACK, EVENT
constexpr uint8_t ESTOP_CLEAR    = 0x05; // []                      → ACK
constexpr uint8_t AXIS_CMD       = 0x10; // [cmd:3][fb:3][19*u16]   （ACKなし・最新値優先）
constexpr uint8_t POSE_CMD       = 0x11; // [foot][ik][6*f32 X,Y,Z,Roll,Pitch,Yaw(rad)]（ACKなし）
constexpr uint8_t NEUTRAL_REQ    = 0x12; // [foot]                  → ACK, POSE_ECHO
constexpr uint8_t TEACH_FRAME    = 0x13; // [cmd:3][fb:3][19*u16]   → ACK（キューに積む。満杯は ST_BUSY → 送り直す）
constexpr uint8_t TEACH_POSE     = 0x14; // [foot][ik][6*f32]       → ACK（キューに積む。満杯は ST_BUSY → 送り直す）
constexpr uint8_t UP_BEGIN       = 0x20; // [frames:u16]            → ACK(token=frames)
constexpr uint8_t UP_DATA        = 0x21; // [offset:u16][data<=48]  → ACK(token=offset)
constexpr uint8_t UP_DATA_MAX    = 48;   // UP_DATA の1回あたりのデータ上限（BLE MTU 64 に収まる）
constexpr uint8_t UP_COMMIT      = 0x22; // []                      → ACK(token=内容ハッシュ16bit)（検証はloop側。RAMに保持するだけ・1件のみ）
constexpr uint8_t UP_START       = 0x23; // []                      → ACK
constexpr uint8_t CAL_ZERO_HERE  = 0x32; // [axis]                  → ACK, CALIB
constexpr uint8_t CAL_SET_FOOTDEF= 0x33; // [f32 L][f32 R]          → ACK, CALIB
constexpr uint8_t CAL_DEFAULTS   = 0x35; // []                      → ACK, CALIB（CalibData.h の値に戻す）
constexpr uint8_t CAL_SET_OFFSET = 0x34; // [axis][offset:u16]      → ACK, CALIB（1軸のオフセットを直接設定）
constexpr uint8_t CAL_SET_ALL    = 0x36; // [19*u16 offset][f32 L][f32 R] → ACK, CALIB（全校正値を一括設定。全部OKの時だけ反映）
constexpr uint8_t CAL_ZERO_ALL   = 0x37; // []                      → ACK, CALIB（全軸を今の姿勢で中立(7500)にする）

// ---- ESP32 -> PC ----
constexpr uint8_t ACK       = 0x80; // [reqType][reqSeq][status][token:u16]
constexpr uint8_t HELLO_R   = 0x81; // [version][mode][frameBytes][maxFrames:u16][fps]
constexpr uint8_t STATE     = 0x82; // [mode][flags][err:3][19*u16 指令値(論理)]   20Hz
constexpr uint8_t FB        = 0x83; // [valid:3][19*u16 実位置(論理)]             20Hz（有効値がある時）
constexpr uint8_t POSE_ECHO = 0x84; // [foot][6*f32]
constexpr uint8_t EVENT     = 0x85; // [code]
constexpr uint8_t CALIB     = 0x86; // [19*u16 offset(生)][f32 L][f32 R][dirty=CalibData.h と異なる]
constexpr uint8_t LINK      = 0x87; // [返信待ち/100us][遅延返信数][19*(応答率%,往復/20us,範囲外拒否)]  1Hz

// ACK status
constexpr uint8_t ST_OK = 0, ST_REJECTED = 1, ST_BAD_STATE = 2, ST_INVALID = 3, ST_IO_ERROR = 4, ST_BUSY = 5;

// mode
constexpr uint8_t MODE_AUTO = 0, MODE_TEACH = 1, MODE_PLAYBACK = 2, MODE_ESTOP = 3;

// STATE flags
constexpr uint8_t FL_ESTOP_LATCHED = 0x01, FL_COMM_LOST = 0x02, FL_BUS_OVERRUN = 0x04,
                  FL_CALIB_DIRTY = 0x08, FL_UPLOAD_READY = 0x10;

// EVENT code
constexpr uint8_t EV_PLAY_DONE = 0, EV_ESTOP = 1, EV_DISCONNECT_STOP = 2, EV_PLAY_START = 3,
                  EV_COMM_LOST = 4, EV_COMM_RESTORED = 5;

// アップロード形式 v2: 1フレーム61byte
//  [dur:u16][flags:u8][19*i16 関節(論理, -1=無視, -2=空白(トラックに参加して保持), 0=FREE)][左足 5*i16][右足 5*i16]
//  flags: bit0=イチギメ, bit1=左足を座標指定, bit2=右足を座標指定, bit3=左IK解, bit4=右IK解
//  座標: X,Y,Z [0.1mm], Roll,Yaw [0.01deg]。座標指定の脚の関節値は無視し、再生時にマイコンでIK。
//  v3 再生規則: 部位トラック（腰/左腕/右腕/左脚/右脚）ごとに、自分の軸に値があるフレームだけを
//  順に再生する（トラック間は独立・並列）。全軸に値のあるフレームは全トラック同時に進む。
constexpr uint16_t FRAME_BYTES = 61;
constexpr uint16_t MAX_FRAMES = 100;
constexpr uint8_t KF_SNAP = 0x01, KF_POSE_L = 0x02, KF_POSE_R = 0x04, KF_IK_L = 0x08, KF_IK_R = 0x10;

} // namespace proto

// ---------------------------------------------------------------
// 2. BLE送受信
// BLE接続状態。
extern volatile bool deviceConnected;

// ESP32_Robot のNordic UART Serviceを初期化する。
void setupBLE();

// loop()から毎回呼び出す。ACKの送信、切断後の再広告などを処理する。
void updateBLE();

// プロトコルv2のパケット送信: [type][seq][payload]。seqは自動で付与。loopタスクからのみ呼ぶこと。
void sendPacket(uint8_t type, const uint8_t* payload, size_t length);

// ACKを予約する（BLE callback / loop どちらからでも可）。送信は updateBLE() で行う。
void queueAck(uint8_t reqType, uint8_t reqSeq, uint8_t status, uint16_t token);

// ---------------------------------------------------------------
// 3. 状態機械
namespace ctrl {

// BLE callback から
void onPacket(const uint8_t* data, size_t len);
void onConnect();
void onDisconnect();

// loop から
void update();              // 要求の処理・状態遷移（loopの先頭で呼ぶ）
uint8_t mode();             // proto::MODE_*
void playbackFinished();    // 再生が最後まで終わった
void telemetry();           // STATE/FB 送信（loopの最後、flushの後で呼ぶ）

} // namespace ctrl
