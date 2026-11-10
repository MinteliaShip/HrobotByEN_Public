#ifndef SERVOICS_H
#define SERVOICS_H
// SPDX-License-Identifier: LGPL-2.1-or-later
// Copyright (C) 2025 MinteIiaShip

#include <Arduino.h>
#include <type_traits>

// （旧: 教示モード用グローバル変数のexternがここにあったが、ライブラリとは無関係なので削除。
//   教示状態は Motion.cpp / RobotController が持つ）

namespace servoICS {

    // KRSモータ用定数
    const long ICS_LOW  = 3500;   // ICS最小値
    const long ICS_HIGH = 11500;  // ICS最大値
    const float DEG_LOW  = -135.0;   // ICS対応角度最小値
    const float DEG_HIGH = 135.0;    // ICS対応角度最大値

    const int ICS_MAX_SPEED = 127;
    const int ICS_MIN_SPEED = 1;

    const int ICS_MAX_STRETCH = 127;
    const int ICS_MIN_STRETCH = 1;

    const float myPI = 3.141592653589;

    template<typename servoICS_T>
    servoICS_T servoICS_map(servoICS_T value, servoICS_T fromLow, servoICS_T fromHigh, servoICS_T toLow, servoICS_T toHigh) {
        servoICS_T inRange = fromHigh - fromLow;
        servoICS_T outRange = toHigh - toLow;
        return ((value - fromLow) * outRange) / inRange  + toLow;
    }

    template<typename servoICS_U>
    struct Result {
        bool success;
        const char* error_msg;
        servoICS_U value;

        inline bool s() const { return success; }
        inline const char* m() const { return error_msg; }
        inline servoICS_U v() const { return value; }

        Result() : success(false), error_msg(""), value(servoICS_U{}) {}

        template<typename servoICS_U_2>
        void assign_value(const Result<servoICS_U_2>& other, std::false_type) {
            value = static_cast<servoICS_U>(other.value);
        }

        template<typename servoICS_U_2>
        void assign_value(const Result<servoICS_U_2>& other, std::true_type) {
            value = static_cast<servoICS_U>(0);
        }

        template<typename servoICS_U_2>
        Result(const Result<servoICS_U_2>& other) : success(other.success), error_msg(other.error_msg)
        {
            assign_value(other, std::is_void<servoICS_U_2>());
        }

        operator servoICS_U() const {
            return value;
        }
    };

    template<>
    struct Result<void> {
        bool success;
        const char* error_msg;

        inline bool s() const { return success; }
        inline const char* m() const { return error_msg; }

        Result() : success(false), error_msg("") {}

        template<typename servoICS_Z>
        Result(const Result<servoICS_Z>& other) 
            : success(other.success), 
            error_msg(other.error_msg) {}
    };

    inline float fromIcs_toDeg(long icsPos){
        return servoICS_map((float)icsPos, (float)ICS_LOW, (float)ICS_HIGH, DEG_LOW, DEG_HIGH);
    }

    inline long fromDeg_toIcs(float degPos){
        // 旧実装は(long)キャストで0方向へ切り捨てていたため、正負で最大1カウントの偏りがあった。四捨五入にする。
        return lroundf(servoICS_map((float)degPos, (float)DEG_LOW, (float)DEG_HIGH, (float)ICS_LOW, (float)ICS_HIGH));
    }

    inline float fromRad_toDeg(float radPos){
        return radPos * (180.0f / myPI);
    }

    inline long fromRad_toIcs(float radPos){   // 旧: 戻り値がfloatだった
        return fromDeg_toIcs(fromRad_toDeg(radPos));
    }

    class Servo{
        private:
            //通信設定
            Stream* portStream_ = nullptr;   // 使用するシリアルインスタンス
            char enPin_ = -1;        // 送受切替ピン（不要な場合は-1）
            uint8_t servoId_ = 0;    // サーボID

            //チェーンメソッド用変数
            Result<void> status;

            //スキップ機能有効化フラグ(0:off 1:on)
            bool isSkip_ = 0;
            //前回「実際に送信した」生の値。スキップ判定に使用（-1=未送信）。
            long prePos_ = -1;
            bool hasSetPos_ = false;

            //送信角度の保持用変数
            long lastSetPos_ = 7500; // 最後に送信されたICS値（初期値: 7500）

            //追加機能パラメータ
            long offSet_ = 7500;    //ソフトウェアオフセット値(ICS値)
            unsigned long waitTimeUs_ = 1000; //送信前に通信を空ける時間(us)
            unsigned long postWaitUs_ = 500;  //setPos送信後の待ち時間(us)（サーボの返信と衝突しないため）

            //遅延送信モード（ServoBus用）。trueの間 setPos/setStretch は送信せず「目標の登録」だけ行い、
            //ServoBus::flush() が周期の最後に時間予算内でまとめて送る。
            static bool deferred_;
            long targetRaw_ = -1;           //登録された目標（生ICS。0=FREE、-1=なし）
            uint8_t pendStretch_ = 0;       //送信待ちストレッチ（0=なし）
            uint16_t rejectCount_ = 0;      //範囲外で拒否した指令の数（診断用）
            static uint32_t lateBytes_;     //送信前に捨てた「遅れて届いた返信」のbyte数（診断・自動調整用）

            //データの送信
            Result<void> send_(const unsigned char* txBuf, unsigned long txLen);

        public:

            Servo(Stream* port, char enPin, uint8_t id){
                status = attach(port, enPin, id);
            }
            Servo() {}

            //成功の場合true、失敗の場合falseを返す
            Result<void> attach(Stream* port, char enPin, uint8_t id);

            //setPos（offsetEn=true: 論理値として受け取り、オフセットを足して送る）。戻り値は returnStatus() と同じ。
            Result<void> setPos(long ics, bool offsetEn = true);
            Result<void> setPosIcs(long ics, bool offsetEn = true){ return setPos(ics, offsetEn); }
            Result<void> setPosDeg(float deg, bool offsetEn = true){ return setPos(fromDeg_toIcs(deg), offsetEn); }
            Result<void> setPosFree(){ return setPos(0); }

            // 保持した送信角度の取得機能
            // ※「生のICS値」（オフセット加算後、サーボへ実際に送った値）。FREE送信後は0。
            long getLastSetPosIcs() const { return lastSetPos_; }
            // 一度でも位置指令(FREE以外)を送ったか
            bool hasSetPos() const { return hasSetPos_; }

            // ---- 論理値 ⇔ 生値 の変換（論理値 = setPos(x, true)に渡す値。7500 = 校正済みの中立）----
            long toRawIcs(long logicalIcs) const { return logicalIcs == 0 ? 0 : logicalIcs + (offSet_ - 7500); }
            long toLogicalIcs(long rawIcs) const { return rawIcs == 0 ? 0 : rawIcs - (offSet_ - 7500); }
            long getLastSetPosLogicalIcs() const { return toLogicalIcs(lastSetPos_); }

            //ストレッチ（保持力）のset
            Result<void> setStretch(unsigned char stretch);

            //追加機能の設定
            //オフセット
            Result<void> setOffset(long offsetIcs);
            Result<void> setOffsetIcs(long offsetIcs){return setOffset(offsetIcs);}

            //通信間隔の設定（既定: 送信前1000us / setPos後500us）。短くするほど19軸の更新が速くなるが、
            //サーボの返信(3byte)とバス上で衝突しない値にすること。
            void setWaitTimeUs(unsigned long preUs, unsigned long postUs){ waitTimeUs_ = preUs; postWaitUs_ = postUs; }

            //スキップ機能（遅延送信モードではServoBusが変化時のみ送るため無関係）
            void setSkip(bool isSkip);

            // ---- 遅延送信モード（ServoBus）----
            static void setDeferredMode(bool on){ deferred_ = on; }
            static bool deferredMode(){ return deferred_; }
            bool hasTarget() const { return targetRaw_ >= 0; }
            uint16_t takeRejectCount(){ uint16_t n = rejectCount_; rejectCount_ = 0; return n; }
            static uint32_t takeLateBytes(){ uint32_t n = lateBytes_; lateBytes_ = 0; return n; }
            long targetRaw() const { return targetRaw_; }
            //ServoBus専用: 送信待ちパラメータを取り出す（sc: 0x01=stretch）
            bool takePendingParam(uint8_t& sc, uint8_t& value);
            //ServoBus専用: 位置指令を即送信し、サーボの返信(現在位置)を最大replyTimeoutUsだけ待つ。
            //返信が来なくても待ち時間は replyTimeoutUs で打ち切る（周期を延ばさない）。
            Result<long> transmitPosRaw(long raw, unsigned long replyTimeoutUs);
            //ServoBus専用: パラメータ書き込み（stretch/speed）を即送信
            Result<void> transmitParam(uint8_t sc, uint8_t value, unsigned long replyTimeoutUs);

            //サーボIDの取得
            uint8_t getId() const { return servoId_; }

            Result<void> returnStatus()const{return status;}
    };
}

// ==============================================================
// ServoBus: 全モーション共通のサーボ出力層
//  - servoBus::begin() 以降、ServoArray[i]->setPos*() / setStretch() は「目標の登録」だけになる。
//  - loop() の最後に servoBus::flush(予算us) を呼ぶと、変化した軸だけを時間予算内で送信する。
//    予算内に送り切れない軸は次の周期へ持ち越す（周期を延ばさない）。
//  - 位置指令に対するサーボの返信（現在位置）をFBとして保持する。読み取り専用の通信は不要。
//  - 返信の無い軸は失敗回数に応じて待ち時間を短縮し、FB読み取り(ポーリング)を一時停止する。
//  - スムージング(一次遅れ+速度制限)は全モーション共通の機能。既定はOFF、教示モードでON。
// ==============================================================
namespace servoBus {

void begin();

// スムージング: 出力 += clamp((目標-出力)*alpha, ±maxStepIcs)
void setSmoothing(bool on, float alpha = 0.35f, float maxStepIcs = 600.0f);

// この周期にFBが欲しい軸（ビットマスク）。毎周期呼ぶ（呼ばなければ解除）。
// FREE中の軸は指定しなくても自動的にポーリングする（手で動かした角度を読むため）。
void requestFb(uint32_t mask);

// 次の flush で、指定軸（ビットマスク）を「目標値が同じでも」必ず送る（一回限り）。
// 返信が得られると通常どおり確定する。pose など、その瞬間だけ全軸を確実に出したい場合に使う。
void forceSend(uint32_t mask);

// 全軸の目標を「今出力している値」に置き換える（通信途絶時の保持など）
void holdCurrent();

// 1周期分の送信。budgetUs を超えそうになったら残りは次周期へ。
void flush(uint32_t budgetUs);

// FB（生ICS）。maxAgeMs より古い値は無効。
bool getFb(int i, long& raw, uint32_t maxAgeMs = 300);

// 直近のflushで予算不足により送れなかった軸があったか
bool overrun();
// 返信が連続して無い軸
uint32_t noReplyMask();
// 直近のflushの所要時間[us]
uint32_t lastFlushUs();
// 現在の返信待ち時間[us]（遅れて届く返信を検出すると自動で延びる）
uint32_t replyTimeoutUs();

// 診断パケット用: [返信待ち/100us][遅延返信回数][19*(応答率% 255=送信なし, 往復時間/20us, 範囲外拒否数)] = 59 byte（BLE上限61-ヘッダ2）
// 呼ぶたびに統計をリセットする（1秒ごとに呼ぶ）
void fillLinkPacket(uint8_t* out);
constexpr size_t LINK_PACKET_BYTES = 59;

} // namespace servoBus

#endif // SERVOICS_H
