#include "RobotController.h"
#include "Motion.h"
#include "servoICS.h"
#include "Config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string>

using namespace proto;

namespace ctrl {
namespace {

constexpr uint32_t ALL_MASK = (1UL << 19) - 1UL;
constexpr uint32_t COMM_TIMEOUT_MS = 1000;
constexpr uint32_t TELEMETRY_MS = 50;

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

struct Cmd { uint8_t type, seq, len; uint8_t d[12]; };
constexpr uint8_t FIFO_N = 16;
Cmd g_fifo[FIFO_N];
uint8_t g_head = 0, g_tail = 0, g_count = 0;

volatile uint8_t g_mode = MODE_AUTO;
bool g_latched = false;
uint8_t g_modeBeforePlay = MODE_AUTO;

volatile bool g_estopReq = false;
volatile uint32_t g_estopMask = ALL_MASK;
volatile uint8_t g_estopSeq = 0;
volatile bool g_disconnectReq = false;
volatile uint32_t g_lastPcMs = 0;
bool g_commLost = false;
uint32_t g_lastTelemetryMs = 0;
uint32_t g_lastLinkMs = 0;

inline uint16_t u16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
inline uint32_t u24(const uint8_t* p) { return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16); }
inline void put16(uint8_t* p, uint16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
inline void put24(uint8_t* p, uint32_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; }

void sendEvent(uint8_t code) { sendPacket(EVENT, &code, 1); }

void sendCalib() {
    uint8_t p[calib::PACKET_BYTES];
    calib::fillPacket(p);
    sendPacket(CALIB, p, sizeof(p));
}

void setMode(uint8_t m) {
    if (m == g_mode) return;
    const uint8_t old = g_mode;
    if (old == MODE_TEACH) teachExit();
    motionForceStopAll();          // どのモード遷移でも、実行中モーションは中断してstatic状態を破棄
    g_mode = m;
    servoBus::setSmoothing(m == MODE_TEACH);
    if (m == MODE_TEACH) teachEnter();
    g_commLost = false;
    Serial.printf("[CTRL] mode %u -> %u\n", old, m);
}

void doEstop(uint32_t mask) {
    if (g_mode == MODE_PLAYBACK) playbackAbort();
    setMode(MODE_ESTOP);
    for (int i = 0; i < 19; ++i) {
        if ((mask & (1UL << i)) && ServoArray[i] != nullptr) ServoArray[i]->setPosFree();
    }
    g_latched = true;
    Serial.println("[CTRL] EMERGENCY STOP");
}

void handle(const Cmd& c) {
    const uint8_t* p = c.d;
    uint8_t st = ST_OK;
    uint16_t token = 0;
    bool calibChanged = false;

    switch (c.type) {
    case HELLO: {
        // [version][mode][frameBytes][maxFrames:u16][fps]（PCのタイムラインのグリッド単位 = 1000/fps ms）
        uint8_t r[6] = {VERSION, g_mode, static_cast<uint8_t>(FRAME_BYTES), 0, 0, static_cast<uint8_t>(G_fps)};
        put16(r + 3, MAX_FRAMES);
        sendPacket(HELLO_R, r, sizeof(r));
        sendCalib();
        return;   // HELLO_R が応答なのでACKなし
    }
    case SET_MODE:
        if (c.len != 1) { st = ST_INVALID; break; }
        if (p[0] == 1) {
            if (g_mode == MODE_PLAYBACK) st = ST_BAD_STATE;
            else setMode(MODE_TEACH);
        } else {
            if (g_mode == MODE_TEACH) setMode(g_latched ? MODE_ESTOP : MODE_AUTO);
            else if (g_mode != MODE_AUTO) st = ST_BAD_STATE;   // ESTOP解除は ESTOP_CLEAR で明示的に
        }
        break;
    case ESTOP_CLEAR:
        g_latched = false;
        if (g_mode == MODE_ESTOP) setMode(MODE_AUTO);
        break;
    case NEUTRAL_REQ:
        if (c.len != 1 || p[0] > 1) st = ST_INVALID;
        else if (g_mode != MODE_TEACH) st = ST_BAD_STATE;
        else if (!teachNeutral(p[0])) st = ST_REJECTED;
        break;
    case UP_COMMIT:
        st = (g_mode == MODE_PLAYBACK) ? ST_BUSY : uploadCommit();
        if (st == ST_OK) token = uploadHash16();
        break;
    case UP_START:
        if (g_mode == MODE_PLAYBACK) { st = ST_BUSY; break; }
        st = playbackStart();
        if (st == ST_OK) {
            g_modeBeforePlay = (g_mode == MODE_ESTOP) ? MODE_AUTO : g_mode;
            g_latched = false;
            setMode(MODE_PLAYBACK);
            sendEvent(EV_PLAY_START);
        }
        break;
    case CAL_ZERO_HERE:
        if (c.len != 1 || p[0] >= 19) st = ST_INVALID;
        else st = calib::zeroHere(p[0]) ? ST_OK : ST_REJECTED;
        calibChanged = true;
        break;
    case CAL_SET_FOOTDEF: {
        if (c.len != 8) { st = ST_INVALID; break; }
        float l, r;
        memcpy(&l, p, 4);
        memcpy(&r, p + 4, 4);
        calib::setFootDef(l, r);
        calibChanged = true;
        break;
    }
    case CAL_SET_OFFSET:
        if (c.len != 3 || p[0] >= 19) { st = ST_INVALID; break; }
        st = calib::setOffset(p[0], static_cast<long>(p[1] | (p[2] << 8))) ? ST_OK : ST_REJECTED;
        calibChanged = true;
        break;
    case CAL_SET_ALL: {
        // 1件でも範囲外があれば何も変えない（半分だけ反映されて食い違うのを防ぐ）
        if (c.len != 46) { st = ST_INVALID; break; }
        long raw[19];
        bool ok = true;
        for (int i = 0; i < 19; ++i) {
            raw[i] = static_cast<long>(p[i * 2] | (p[i * 2 + 1] << 8));
            if (raw[i] < 3500 || raw[i] > 11500) ok = false;
        }
        float l, r;
        memcpy(&l, p + 38, 4);
        memcpy(&r, p + 42, 4);
        if (!isfinite(l) || !isfinite(r) || fabsf(l) > 100 || fabsf(r) > 100) ok = false;
        if (!ok) { st = ST_REJECTED; break; }
        for (int i = 0; i < 19; ++i) calib::setOffset(i, raw[i]);
        calib::setFootDef(l, r);
        calibChanged = true;
        break;
    }
    case CAL_ZERO_ALL: {
        bool allOk = true;
        for (int i = 0; i < 19; ++i) {
            if (!calib::zeroHere(i)) allOk = false;
        }
        st = allOk ? ST_OK : ST_REJECTED;   // 応答のないサーボがあっても、取れた軸は反映済み
        calibChanged = true;
        break;
    }
    case CAL_DEFAULTS:
        calib::loadDefaults();
        calibChanged = true;
        break;
    default:
        st = ST_INVALID;
        break;
    }
    queueAck(c.type, c.seq, st, token);
    if (calibChanged) sendCalib();
}

void handleDisconnect() {
    if (g_mode == MODE_PLAYBACK) {
        playbackAbort();
        setMode(MODE_ESTOP);
        for (int i = 0; i < 19; ++i) if (ServoArray[i] != nullptr) ServoArray[i]->setPosFree();
        g_latched = true;
        Serial.println("[CTRL] BLE disconnected during playback: all servos freed.");
    } else if (g_mode == MODE_TEACH) {
        setMode(g_latched ? MODE_ESTOP : MODE_AUTO);
    }
}

} // namespace

// ------------------------------------------------------------ BLE callback 側
void onPacket(const uint8_t* d, size_t n) {
    if (d == nullptr || n < 2) return;
    const uint8_t type = d[0];
    const uint8_t seq = d[1];
    const uint8_t* p = d + 2;
    const size_t len = n - 2;
    g_lastPcMs = millis();

    switch (type) {
    case HEARTBEAT:
        return;
    case AXIS_CMD:
        if (g_mode == MODE_TEACH) teachPostAxis(p, len);
        return;
    case POSE_CMD:
        if (g_mode == MODE_TEACH) teachPostPose(p, len);
        return;
    case UP_BEGIN: {
        const uint16_t frames = len == 2 ? u16(p) : 0;
        queueAck(type, seq, len == 2 ? uploadBegin(frames) : ST_INVALID, frames);
        return;
    }
    case TEACH_FRAME: {
        // 軸1コマ。受け取った順に全部実行する（最新値で上書きしない）。ACKは取り込んだ時点で返す
        const uint8_t st = (g_mode != MODE_TEACH) ? ST_BAD_STATE : teachQueueAxis(p, len);
        queueAck(type, seq, st, 0);
        return;
    }
    case TEACH_POSE: {
        const uint8_t st = (g_mode != MODE_TEACH) ? ST_BAD_STATE : teachQueuePose(p, len);
        queueAck(type, seq, st, 0);
        return;
    }
    case UP_DATA: {
        if (len < 3 || len > 2 + UP_DATA_MAX) { queueAck(type, seq, ST_INVALID, 0); return; }
        const uint16_t off = u16(p);
        queueAck(type, seq, uploadData(off, p + 2, len - 2), off);
        return;
    }
    case ESTOP: {
        // 緊急停止はFIFOに積まない（満杯でも取りこぼさない）
        const uint32_t mask = (len >= 3 ? u24(p) : ALL_MASK) & ALL_MASK;
        portENTER_CRITICAL(&g_mux);
        g_estopMask = mask ? mask : ALL_MASK;
        g_estopSeq = seq;
        g_estopReq = true;
        portEXIT_CRITICAL(&g_mux);
        return;
    }
    default:
        break;
    }

    if (len > sizeof(Cmd::d)) { queueAck(type, seq, ST_INVALID, 0); return; }
    bool ok = false;
    portENTER_CRITICAL(&g_mux);
    if (g_count < FIFO_N) {
        Cmd& c = g_fifo[g_tail];
        c.type = type; c.seq = seq; c.len = static_cast<uint8_t>(len);
        memcpy(c.d, p, len);
        g_tail = (g_tail + 1) % FIFO_N;
        ++g_count;
        ok = true;
    }
    portEXIT_CRITICAL(&g_mux);
    if (!ok) queueAck(type, seq, ST_BUSY, 0);
}

void onConnect() { g_lastPcMs = millis(); }

void onDisconnect() { g_disconnectReq = true; }

// ------------------------------------------------------------ loop 側
uint8_t mode() { return g_mode; }

void playbackFinished() {
    setMode(g_modeBeforePlay);
    sendEvent(EV_PLAY_DONE);
}

void update() {
    if (g_disconnectReq) {
        g_disconnectReq = false;
        handleDisconnect();
    }

    bool estop = false;
    uint32_t mask = ALL_MASK;
    uint8_t eseq = 0;
    portENTER_CRITICAL(&g_mux);
    if (g_estopReq) {
        estop = true;
        mask = g_estopMask;
        eseq = g_estopSeq;
        g_estopReq = false;
    }
    portEXIT_CRITICAL(&g_mux);
    if (estop) {
        doEstop(mask);
        queueAck(ESTOP, eseq, ST_OK, 0);
        sendEvent(EV_ESTOP);
    }

    for (;;) {
        Cmd c;
        bool has = false;
        portENTER_CRITICAL(&g_mux);
        if (g_count > 0) {
            c = g_fifo[g_head];
            g_head = (g_head + 1) % FIFO_N;
            --g_count;
            has = true;
        }
        portEXIT_CRITICAL(&g_mux);
        if (!has) break;
        handle(c);
    }

    // 通信途絶の検出（TEACH中のみ）。途絶したら現在の出力で保持する。
    if (g_mode == MODE_TEACH && deviceConnected) {
        const bool lost = (millis() - g_lastPcMs) > COMM_TIMEOUT_MS;
        if (lost && !g_commLost) {
            g_commLost = true;
            servoBus::holdCurrent();
            Serial.println("[CTRL] PC heartbeat lost: holding current pose.");
            sendEvent(EV_COMM_LOST);   // リンク自体は生きている（PCアプリ停止など）場合に通知が届く
        } else if (!lost && g_commLost) {
            g_commLost = false;
            sendEvent(EV_COMM_RESTORED);
        }
    } else {
        g_commLost = false;
    }
}

void telemetry() {
    if (!deviceConnected) return;
    const uint32_t now = millis();
    if ((now - g_lastTelemetryMs) < TELEMETRY_MS) return;
    g_lastTelemetryMs = now;

    uint8_t s[43];
    s[0] = g_mode;
    s[1] = (g_latched ? FL_ESTOP_LATCHED : 0) | (g_commLost ? FL_COMM_LOST : 0) |
           (servoBus::overrun() ? FL_BUS_OVERRUN : 0) | (calib::dirty() ? FL_CALIB_DIRTY : 0) |
           (uploadReady() ? FL_UPLOAD_READY : 0);
    put24(s + 2, (teachErrorMask() | servoBus::noReplyMask()) & ALL_MASK);
    for (int i = 0; i < 19; ++i) {
        uint16_t v = 0;
        servoICS::Servo* sv = ServoArray[i];
        if (sv != nullptr && sv->hasTarget()) v = static_cast<uint16_t>(sv->toLogicalIcs(sv->targetRaw()));
        put16(s + 5 + i * 2, v);
    }
    sendPacket(STATE, s, sizeof(s));

    uint8_t f[41] = {0};
    uint32_t valid = 0;
    for (int i = 0; i < 19; ++i) {
        long raw;
        if (ServoArray[i] != nullptr && servoBus::getFb(i, raw)) {
            valid |= (1UL << i);
            put16(f + 3 + i * 2, static_cast<uint16_t>(ServoArray[i]->toLogicalIcs(raw)));
        }
    }
    if (valid) {
        put24(f, valid);
        sendPacket(FB, f, sizeof(f));
    }

    // 通信診断（1Hz）: 軸ごとの応答率・往復時間・範囲外拒否数
    if ((now - g_lastLinkMs) >= 1000) {
        g_lastLinkMs = now;
        uint8_t l[servoBus::LINK_PACKET_BYTES];
        servoBus::fillLinkPacket(l);
        sendPacket(LINK, l, sizeof(l));
    }
}

} // namespace ctrl

// ##############################################################
// BLE送受信（旧 BLEManager.cpp）
// ##############################################################
// Nordic UART Service UUID
namespace {
constexpr char SERVICE_UUID[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char CHARACTERISTIC_UUID_RX[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; // PC -> ESP32
constexpr char CHARACTERISTIC_UUID_TX[] = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"; // ESP32 -> PC

BLEServer* pServer = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
bool oldDeviceConnected = false;
uint8_t g_txSeq = 0;

// GATT callbackからnotify()を直接呼ばない。ACKをFIFOへ積み、loop()側のupdateBLE()から送る。
portMUX_TYPE ackMux = portMUX_INITIALIZER_UNLOCKED;
constexpr uint8_t ACK_QUEUE_CAPACITY = 24;
struct AckRecord {
    uint8_t type;
    uint8_t seq;
    uint8_t status;
    uint16_t token;
};
AckRecord ackQueue[ACK_QUEUE_CAPACITY] = {};
uint8_t ackHead = 0;
uint8_t ackTail = 0;
uint8_t ackCount = 0;

void clearAcks() {
    portENTER_CRITICAL(&ackMux);
    ackHead = ackTail = ackCount = 0;
    portEXIT_CRITICAL(&ackMux);
}

class ServerCallbacks final : public BLEServerCallbacks {
public:
    void onConnect(BLEServer* server) override {
        (void)server;
        deviceConnected = true;
        ctrl::onConnect();
    }

    void onDisconnect(BLEServer* server) override {
        (void)server;
        deviceConnected = false;
        ctrl::onDisconnect();   // 状態遷移はloop側で行う
        clearAcks();
    }
};

// BLE callback内ではサーボUARTを操作しない。受信パケットをコントローラへ渡すだけ。
class RxCallbacks final : public BLECharacteristicCallbacks {
public:
    void onWrite(BLECharacteristic* characteristic) override {
        const std::string rxValue = characteristic->getValue();
        if (rxValue.size() < 2) return;
        ctrl::onPacket(reinterpret_cast<const uint8_t*>(rxValue.data()), rxValue.size());
    }
};
} // namespace

volatile bool deviceConnected = false;

void queueAck(uint8_t reqType, uint8_t reqSeq, uint8_t status, uint16_t token) {
    portENTER_CRITICAL(&ackMux);
    if (ackCount < ACK_QUEUE_CAPACITY) {
        ackQueue[ackTail] = {reqType, reqSeq, status, token};
        ackTail = static_cast<uint8_t>((ackTail + 1) % ACK_QUEUE_CAPACITY);
        ++ackCount;
    } else {
        // 満杯時は末尾を最新結果で上書き（PCはACKごとに次を送るため通常は到達しない）
        const uint8_t last = static_cast<uint8_t>((ackTail + ACK_QUEUE_CAPACITY - 1) % ACK_QUEUE_CAPACITY);
        ackQueue[last] = {reqType, reqSeq, status, token};
    }
    portEXIT_CRITICAL(&ackMux);
}

void setupBLE() {
    BLEDevice::init("ESP32_Robot");
    BLEDevice::setMTU(64);

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService* service = pServer->createService(SERVICE_UUID);

    pTxCharacteristic = service->createCharacteristic(
        CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic* rxCharacteristic = service->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    rxCharacteristic->setCallbacks(new RxCallbacks());

    service->start();
    pServer->getAdvertising()->start();
    oldDeviceConnected = false;
    Serial.println("BLE UART Ready.");
}

// ESP32 -> PCへのNotification送信（生データ）。loopタスクからのみ。
static void sendBLE(uint8_t* data, size_t length) {
    if (!deviceConnected || pTxCharacteristic == nullptr || data == nullptr) return;
    // ATT MTU 64の場合のNotificationペイロード上限(64 - 3)。
    if (length == 0 || length > 61) return;

    pTxCharacteristic->setValue(data, length);
    pTxCharacteristic->notify();
}

void sendPacket(uint8_t type, const uint8_t* payload, size_t length) {
    if (length > 59) return;
    uint8_t buf[61];
    buf[0] = type;
    buf[1] = g_txSeq++;
    if (length) memcpy(buf + 2, payload, length);
    sendBLE(buf, length + 2);
}

void updateBLE() {
    // ACKはまとめて送る（転送中はACK待ちでPCが止まっているので、溜めない）
    for (int n = 0; n < 6; ++n) {
        AckRecord r;
        bool has = false;
        portENTER_CRITICAL(&ackMux);
        if (ackCount > 0) {
            r = ackQueue[ackHead];
            ackHead = static_cast<uint8_t>((ackHead + 1) % ACK_QUEUE_CAPACITY);
            --ackCount;
            has = true;
        }
        portEXIT_CRITICAL(&ackMux);
        if (!has) break;
        const uint8_t p[5] = {r.type, r.seq, r.status,
                              static_cast<uint8_t>(r.token & 0xFF), static_cast<uint8_t>((r.token >> 8) & 0xFF)};
        sendPacket(proto::ACK, p, sizeof(p));
    }

    // BLE切断後に再広告し、再接続できるようにする。
    if (!deviceConnected && oldDeviceConnected) {
        if (pServer != nullptr) pServer->startAdvertising();
        Serial.println("[BLE] Advertising restarted.");
        oldDeviceConnected = false;
    } else if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = true;
        Serial.println("[BLE] Client connected.");
    }
}
