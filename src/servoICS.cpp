// SPDX-License-Identifier: LGPL-2.1-or-later
// Copyright (C) 2025 MinteIiaShip

#include "servoICS.h"
#include "Config.h"
#include <math.h>

namespace servoICS {

    // 静的メンバ変数の定義
    bool Servo::deferred_ = false;
    uint32_t Servo::lateBytes_ = 0;

    //返信を待つ共通処理。エコー(自分の送信3byte)が見える配線/見えない配線の両方に対応する。
    //返信の先頭byteは「送信先頭byteの最上位bitを落とした値」。
    static bool readIcsReply(Stream* s, const unsigned char* tx, unsigned long timeoutUs, unsigned char out[3]) {
        unsigned char buf[6];
        size_t n = 0;
        const unsigned long t0 = micros();
        const unsigned char head = tx[0] & 0x7F;
        while ((micros() - t0) < timeoutUs) {
            const int ch = s->read();
            if (ch < 0) continue;
            if (n < sizeof(buf)) buf[n++] = (unsigned char)ch;
            // 返信が先頭から（エコー無し）
            if (n >= 3 && buf[0] == head) { memcpy(out, buf, 3); return true; }
            // エコー3byteの後に返信
            if (n >= 6 && buf[0] == tx[0] && buf[1] == tx[1] && buf[2] == tx[2] && buf[3] == head) {
                memcpy(out, buf + 3, 3); return true;
            }
            if (n >= sizeof(buf)) {   // 想定外のbyte列。1byteずらして探し直す
                memmove(buf, buf + 1, sizeof(buf) - 1);
                n = sizeof(buf) - 1;
            }
        }
        return false;
    }

    // 送信のみ（返信は読まない）。遅延送信モードに入る前（setup中）の setPos / setStretch で使う。
    Result<void> Servo::send_(const unsigned char* txBuf, unsigned long txLen){
        Result<void> result;
        result.success = 1;
        if(portStream_ == nullptr){
            result.success = 0;
            result.error_msg = "[E]send_:portStream_に実体がない。";
            return result;
        }
        ::delayMicroseconds(waitTimeUs_);
        if(enPin_ >= 0)digitalWrite(enPin_, HIGH);
        portStream_->write(txBuf, txLen);
        portStream_->flush();
        if(enPin_ >= 0)digitalWrite(enPin_, LOW);
        return result;
    }

    Result<void> Servo::attach(Stream* port, char enPin, uint8_t id) {
        Result<void> result;
        result.success = 1;
        portStream_ = port;
        servoId_ = id;
        enPin_ = enPin;
        if(enPin_ >= 0)pinMode(enPin_, OUTPUT);
        if(enPin_ >= 0)digitalWrite(enPin_, LOW);

        return result;
    }

    Result<void> Servo::setPos(long ics, bool offsetEn){
        static char error_str[160] = "";

        status.success = 1;
        status.error_msg = "";

        if(ics != 0){
            //オフセット有効時は論理値→生値へ変換
            if(offsetEn) ics = ics + (offSet_ - 7500);

            //異常指令値をはじく（送信しない＝直前の姿勢を保持）
            if(ics > ICS_HIGH){
                status.success = 0;
                if (rejectCount_ < 0xFFFF) ++rejectCount_;
                snprintf(error_str, sizeof(error_str), "[E]setPos:上限ICS_HIGHより指令値icsの方が大きいです。ICS=%ld Deg=%d", ics, (int)fromIcs_toDeg(ics));
                status.error_msg = error_str;
                return status;
            }else if(ics < ICS_LOW){
                status.success = 0;
                if (rejectCount_ < 0xFFFF) ++rejectCount_;
                snprintf(error_str, sizeof(error_str), "[E]setPos:下限ICS_LOWより指令値icsの方が小さいです。ICS=%ld Deg=%d", ics, (int)fromIcs_toDeg(ics));
                status.error_msg = error_str;
                return status;
            }
        }

        //遅延送信モード: 目標を登録するだけ。送信は ServoBus::flush()。
        if(deferred_){
            targetRaw_ = ics;
            return status;
        }

        //スキップ判定は「オフセット適用後の実際に送る値」で行う。
        //旧実装はオフセット適用前の値で比較し、さらに送信失敗/リミット拒否でもprePos_を更新していたため、
        //同じ値を再送しても送られないことがあった。
        if(isSkip_ && (ics == prePos_)){
            status.success = 1;
            status.error_msg = "[I]スキップを実行しました。";
            return status;
        }

        unsigned char txByte[3];
        txByte[0] = 0b10000000 | (0b00011111 & servoId_);
        txByte[1] = (unsigned char)((ics >> 7) & 0x7F);
        txByte[2] = (unsigned char)(ics & 0x7F);

        status = send_(txByte, 3);

        // 送信処理に成功した場合、保持値を更新
        if (status.success) {
            lastSetPos_ = ics;
            prePos_ = ics;
            if (ics != 0) hasSetPos_ = true;
        }

        if (postWaitUs_ > 0) delayMicroseconds(postWaitUs_);

        return status;
    }

    Result<void> Servo::setStretch(unsigned char stretch){
        Result<void> result;
        result.success = 1;
        unsigned char txByte[3];

        if(stretch < 1 || stretch > 127){
            result.success = 0;
            result.error_msg = "[E]setStretch:stretch指令値が範囲外(1~127)";
            return result;
        }

        if(deferred_){ pendStretch_ = stretch; return result; }

        txByte[0] = 0b11000000 | (0b00011111 & servoId_);
        txByte[1] = 0x01;
        txByte[2] = stretch;

        result = send_(txByte, 3);
        return result;
    }

    Result<void> Servo::setOffset(long offsetIcs){
        Result<void> result;
        result.success = 1;

        if((offsetIcs<ICS_LOW)||(ICS_HIGH<offsetIcs)){
            result.error_msg = "[E] setOffset:オフセット設定値が設置した角度内にありません";
            result.success = 0;
            return result;
        }

        offSet_ = offsetIcs;
        return result;
    }

    void Servo::setSkip(bool isSkip){
        isSkip_ = isSkip;
    }


    bool Servo::takePendingParam(uint8_t& sc, uint8_t& value){
        if(pendStretch_){ sc = 0x01; value = pendStretch_; pendStretch_ = 0; return true; }
        return false;
    }

    Result<long> Servo::transmitPosRaw(long raw, unsigned long replyTimeoutUs){
        Result<long> r;
        r.success = 0;
        r.value = 0;
        if(portStream_ == nullptr){ r.error_msg = "[E]transmitPosRaw:portStream_に実体がない。"; return r; }

        unsigned char tx[3];
        tx[0] = 0b10000000 | (0b00011111 & servoId_);
        tx[1] = (unsigned char)((raw >> 7) & 0x7F);
        tx[2] = (unsigned char)(raw & 0x7F);

        if(waitTimeUs_ > 0) ::delayMicroseconds(waitTimeUs_);
        while(portStream_->available() > 0){ portStream_->read(); ++lateBytes_; }   //前回の残り/遅れた返信を捨てる
        if(enPin_ >= 0) digitalWrite(enPin_, HIGH);
        portStream_->write(tx, 3);
        portStream_->flush();
        if(enPin_ >= 0) digitalWrite(enPin_, LOW);

        lastSetPos_ = raw;
        prePos_ = raw;
        if(raw != 0) hasSetPos_ = true;

        unsigned char rx[3];
        if(readIcsReply(portStream_, tx, replyTimeoutUs, rx)){
            r.success = 1;
            r.value = (long)(((rx[1] & 0x7F) << 7) | (rx[2] & 0x7F));
        }else{
            r.error_msg = "[W]transmitPosRaw:返信なし";
        }
        return r;
    }

    Result<void> Servo::transmitParam(uint8_t sc, uint8_t value, unsigned long replyTimeoutUs){
        Result<void> r;
        r.success = 0;
        if(portStream_ == nullptr){ r.error_msg = "[E]transmitParam:portStream_に実体がない。"; return r; }
        unsigned char tx[3];
        tx[0] = 0b11000000 | (0b00011111 & servoId_);
        tx[1] = sc;
        tx[2] = value;
        if(waitTimeUs_ > 0) ::delayMicroseconds(waitTimeUs_);
        while(portStream_->available() > 0) portStream_->read();
        if(enPin_ >= 0) digitalWrite(enPin_, HIGH);
        portStream_->write(tx, 3);
        portStream_->flush();
        if(enPin_ >= 0) digitalWrite(enPin_, LOW);
        unsigned char rx[3];
        r.success = readIcsReply(portStream_, tx, replyTimeoutUs, rx) ? 1 : 0;
        if(!r.success) r.error_msg = "[W]transmitParam:返信なし";
        return r;
    }

}

// ##############################################################
// ServoBus（旧 ServoBus.cpp）: 全モーション共通のサーボ出力層
// ##############################################################
// ==============================================================
// 【v3 修正】返信待ちの安全化
//  旧v2: 返信が来ない軸を「不調」とみなし、待ち時間を0.3msに縮めていた。
//        実機ではサーボの応答遅れ + ESP32 UARTの受信通知の遅れ(RXタイムアウト≒1ms)で
//        返信が1.5ms以内に「見えない」ことがあり、正常な軸まで不調扱いになる
//        → 待ちが0.3msに縮む → まだ線上に返信が流れている最中に次の軸へ送信して衝突
//        → 次の軸の指令が壊れる（模擬で脚の軸の指令が最大99%失われることを確認）。
//  v3: (1) 待ち時間は全軸同じ。縮めない（返信が物理的に終わるまで次を送らない）。
//      (2) 遅れて届いた返信を検出したら、待ち時間を自動で延ばす（最大3ms）。
//      (3) UARTの受信通知の遅れ自体を main.cpp で小さくする（uart_set_rx_timeout）。
//      (4) 軸ごとの通信統計をPCへ送る（LINKパケット）。
// ==============================================================

namespace servoBus {
namespace {

constexpr int N = 19;
constexpr unsigned long PRE_WAIT_US = 200;          // 送信前の空き時間（返信の受信完了を確認してから送るので短くてよい）
constexpr unsigned long REPLY_TIMEOUT_MIN_US = 1500;
constexpr unsigned long REPLY_TIMEOUT_MAX_US = 3000;
constexpr unsigned long REPLY_TIMEOUT_STEP_US = 250;
constexpr uint8_t  FAILS_BAD = 3;                   // この回数続けて無応答なら「不調」（FB読み取りを間引くだけ）
constexpr uint32_t BAD_POLL_INTERVAL_MS = 500;
constexpr uint32_t POLL_MIN_INTERVAL_MS = 40;
constexpr uint32_t SEND_COST_US = 1100;             // 正常な1回の送信の見積り（予算判定用）
constexpr uint8_t  MAX_RESEND = 3;

struct Axis {
    float out = 0;
    bool outValid = false;
    long desired = -1;
    bool confirmed = true;
    uint8_t resend = 0;
    uint8_t fails = 0;
    long fb = 0;
    uint32_t fbMs = 0;
    bool fbValid = false;
    uint32_t lastPollMs = 0;
    // 統計（1秒ごとにリセット）
    uint16_t statSent = 0;
    uint16_t statOk = 0;
    uint32_t statRttSum = 0;
};

Axis g_ax[N];
bool g_smooth = false;
float g_alpha = 0.35f;
float g_maxStep = 600.0f;
uint32_t g_fbReq = 0;
uint8_t g_rrSend = 0;
uint8_t g_rrPoll = 0;
bool g_overrun = false;
uint32_t g_lastFlushUs = 0;
unsigned long g_replyTimeoutUs = REPLY_TIMEOUT_MIN_US;
uint16_t g_statLate = 0;
uint16_t g_statOverrun = 0;

inline uint32_t sendCost(const Axis& a) {
    return a.fails >= FAILS_BAD ? (PRE_WAIT_US + 300 + g_replyTimeoutUs) : SEND_COST_US;
}

void onReply(Axis& a, const servoICS::Result<long>& r, uint32_t rttUs) {
    if (a.statSent < 0xFFFF) ++a.statSent;
    if (r.success) {
        if (a.statOk < 0xFFFF) ++a.statOk;
        a.statRttSum += rttUs;
        a.fb = r.value;
        a.fbMs = millis();
        a.fbValid = true;
        a.fails = 0;
        a.confirmed = true;
        a.resend = 0;
    } else {
        if (a.fails < 255) ++a.fails;
        if (a.fails >= FAILS_BAD || a.resend >= MAX_RESEND) {
            a.confirmed = true;
            a.resend = 0;
        } else {
            a.confirmed = false;
            ++a.resend;
        }
    }
}

servoICS::Result<long> transmit(int i, long v, uint32_t& rtt) {
    const uint32_t t = micros();
    servoICS::Result<long> r = ServoArray[i]->transmitPosRaw(v, g_replyTimeoutUs);
    rtt = micros() - t;
    // 前回の返信が「待ち時間を過ぎてから」届いていた → 待ち時間を延ばす
    if (servoICS::Servo::takeLateBytes() > 0) {
        if (g_statLate < 0xFFFF) ++g_statLate;
        if (g_replyTimeoutUs < REPLY_TIMEOUT_MAX_US) g_replyTimeoutUs += REPLY_TIMEOUT_STEP_US;
    }
    return r;
}

} // namespace

void begin() {
    for (int i = 0; i < N; ++i) {
        if (ServoArray[i] != nullptr) ServoArray[i]->setWaitTimeUs(PRE_WAIT_US, 0);
        g_ax[i] = Axis();
    }
    servoICS::Servo::takeLateBytes();
    servoICS::Servo::setDeferredMode(true);
}

void setSmoothing(bool on, float alpha, float maxStepIcs) {
    g_smooth = on;
    g_alpha = alpha;
    g_maxStep = maxStepIcs;
    for (int i = 0; i < N; ++i) g_ax[i].outValid = false;
}

void requestFb(uint32_t mask) { g_fbReq |= mask; }

void forceSend(uint32_t mask) {
    for (int i = 0; i < N; ++i) {
        if (mask & (1UL << i)) g_ax[i].confirmed = false;   // flush の送信条件 (!a.confirmed) に当たる
    }
}

void holdCurrent() {
    for (int i = 0; i < N; ++i) {
        servoICS::Servo* s = ServoArray[i];
        if (s == nullptr || !s->hasTarget()) continue;
        const long last = s->getLastSetPosIcs();
        if (last > 0) s->setPos(last, false);
    }
}

bool getFb(int i, long& raw, uint32_t maxAgeMs) {
    if (i < 0 || i >= N || !g_ax[i].fbValid) return false;
    if (ABSENT_AXIS_MASK & (1UL << i)) return false;
    if ((millis() - g_ax[i].fbMs) > maxAgeMs) return false;
    raw = g_ax[i].fb;
    return true;
}

bool overrun() { return g_overrun; }
uint32_t lastFlushUs() { return g_lastFlushUs; }
uint32_t replyTimeoutUs() { return g_replyTimeoutUs; }
uint32_t noReplyMask() {
    uint32_t m = 0;
    for (int i = 0; i < N; ++i) if (g_ax[i].fails >= FAILS_BAD) m |= (1UL << i);
    return m;
}

void fillLinkPacket(uint8_t* out) {
    out[0] = static_cast<uint8_t>(g_replyTimeoutUs / 100);
    out[1] = static_cast<uint8_t>(g_statLate > 255 ? 255 : g_statLate);
    for (int i = 0; i < N; ++i) {
        Axis& a = g_ax[i];
        uint8_t rate = 255;                       // 255 = この1秒は送信なし
        uint8_t rtt = 0;
        if (a.statSent > 0) {
            rate = static_cast<uint8_t>((a.statOk * 100UL) / a.statSent);
            if (a.statOk > 0) {
                const uint32_t avg = a.statRttSum / a.statOk;
                rtt = static_cast<uint8_t>(avg / 20 > 255 ? 255 : avg / 20);
            }
        }
        uint16_t rej = ServoArray[i] != nullptr ? ServoArray[i]->takeRejectCount() : 0;
        out[2 + i * 3] = rate;
        out[3 + i * 3] = rtt;
        out[4 + i * 3] = static_cast<uint8_t>(rej > 255 ? 255 : rej);
        a.statSent = a.statOk = 0;
        a.statRttSum = 0;
    }
    g_statLate = 0;
    g_statOverrun = 0;
}

void flush(uint32_t budgetUs) {
    const uint32_t t0 = micros();
    const uint32_t now = millis();
    auto elapsed = [&]() { return static_cast<uint32_t>(micros() - t0); };

    // --- 1. 各軸の今周期の出力値を決める ---
    uint32_t needSend = 0;
    for (int i = 0; i < N; ++i) {
        servoICS::Servo* s = ServoArray[i];
        Axis& a = g_ax[i];
        a.desired = -1;
        if (ABSENT_AXIS_MASK & (1UL << i)) continue;   // 無い軸は送らない
        if (s == nullptr || !s->hasTarget()) continue;
        const long tgt = s->targetRaw();
        long d;
        if (tgt == 0) {
            d = 0;
            a.outValid = false;
        } else if (!g_smooth) {
            d = tgt;
            a.out = static_cast<float>(tgt);
            a.outValid = true;
        } else {
            if (!a.outValid) {
                // 開始点: 最後に送った値 → 新しいFB の順。どちらも無い（脱力中）軸は、
                // まずFREE指令を1回送って返信から現在位置を読み、次の周期から滑らかに動かす。
                long start = s->getLastSetPosIcs();
                long fb;
                if (!(s->hasSetPos() && start > 0)) {
                    if (getFb(i, fb, 500)) {
                        start = fb;
                    } else if (a.fails < FAILS_BAD) {
                        a.desired = 0;
                        needSend |= (1UL << i);
                        continue;
                    } else {
                        start = tgt;
                    }
                }
                a.out = static_cast<float>(start);
                a.outValid = true;
            }
            const float diff = static_cast<float>(tgt) - a.out;
            if (fabsf(diff) <= 1.0f) {
                a.out = static_cast<float>(tgt);
            } else {
                float step = diff * g_alpha;
                if (fabsf(step) < 1.0f) step = diff > 0 ? 1.0f : -1.0f;
                if (step > g_maxStep) step = g_maxStep;
                if (step < -g_maxStep) step = -g_maxStep;
                a.out += step;
            }
            d = lroundf(a.out);
        }
        a.desired = d;
        if (d != s->getLastSetPosIcs() || !a.confirmed || (!s->hasSetPos() && d != 0)) {
            needSend |= (1UL << i);
        }
    }

    g_overrun = false;
    uint32_t sentMask = 0;

    // --- 2. パラメータ書き込み（ストレッチ等） ---
    for (int i = 0; i < N; ++i) {
        servoICS::Servo* s = ServoArray[i];
        if (s == nullptr || (ABSENT_AXIS_MASK & (1UL << i))) continue;
        uint8_t sc, val;
        if (elapsed() + sendCost(g_ax[i]) > budgetUs) { g_overrun = true; break; }
        if (s->takePendingParam(sc, val)) {
            s->transmitParam(sc, val, g_replyTimeoutUs);
            servoICS::Servo::takeLateBytes();
        }
    }

    // --- 3. 位置指令（変化した軸だけ、ラウンドロビン） ---
    for (int n = 0; n < N && needSend; ++n) {
        const int i = (g_rrSend + n) % N;
        const uint32_t bit = (1UL << i);
        if (!(needSend & bit)) continue;
        Axis& a = g_ax[i];
        if (elapsed() + sendCost(a) > budgetUs) {
            g_overrun = true;
            g_rrSend = static_cast<uint8_t>(i);
            break;
        }
        uint32_t rtt;
        const servoICS::Result<long> r = transmit(i, a.desired, rtt);
        onReply(a, r, rtt);
        a.lastPollMs = now;
        needSend &= ~bit;
        sentMask |= bit;
        if (needSend == 0) g_rrSend = static_cast<uint8_t>((i + 1) % N);
    }
    if (g_overrun && g_statOverrun < 0xFFFF) ++g_statOverrun;

    // --- 4. FBポーリング（同じ値の再送で返信の現在位置を得る。予算が残っている時だけ） ---
    uint32_t pollMask = g_fbReq;
    for (int i = 0; i < N; ++i) {
        if (ServoArray[i] != nullptr && ServoArray[i]->hasTarget() && ServoArray[i]->targetRaw() == 0) {
            pollMask |= (1UL << i);
        }
    }
    g_fbReq = 0;
    pollMask &= ~sentMask;
    pollMask &= ~ABSENT_AXIS_MASK;
    for (int n = 0; n < N && pollMask; ++n) {
        const int i = (g_rrPoll + n) % N;
        const uint32_t bit = (1UL << i);
        if (!(pollMask & bit)) continue;
        servoICS::Servo* s = ServoArray[i];
        Axis& a = g_ax[i];
        if (s == nullptr) continue;
        const long last = s->getLastSetPosIcs();
        if (!s->hasTarget() && !s->hasSetPos()) continue;
        const uint32_t interval = a.fails >= FAILS_BAD ? BAD_POLL_INTERVAL_MS : POLL_MIN_INTERVAL_MS;
        if ((now - a.lastPollMs) < interval) continue;
        if (elapsed() + sendCost(a) > budgetUs) {
            g_rrPoll = static_cast<uint8_t>(i);
            break;
        }
        const long v = (s->hasTarget() && s->targetRaw() == 0) ? 0 : last;
        uint32_t rtt;
        const servoICS::Result<long> r = transmit(i, v, rtt);
        onReply(a, r, rtt);
        a.lastPollMs = now;
        g_rrPoll = static_cast<uint8_t>((i + 1) % N);
    }

    g_lastFlushUs = elapsed();
}

} // namespace servoBus
