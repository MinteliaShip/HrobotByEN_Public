#ifndef MOTION_H_
#define MOTION_H_

#include <Arduino.h>
#include "Config.h"
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include "FootController.h"
#include "ExportData.h"             // モーション本体（ExportFrame・論理ICS）。ExportData.h と両方 include すると再定義エラーになる

// ==============================================================
// Motion.cpp が提供する、教示・事前転送再生・モーション管理の関数（RobotController / main から使う）
// ==============================================================
// ---- モーション管理 ----
// 実行中の全モーションを止め、関数内static状態を次回起動時に初期化させる
void motionForceStopAll();

// ---- 教示(TEACH) ----
// BLE callbackから呼ぶ（最新値優先のメールボックスへ置くだけ）。payloadは [type][seq] の後ろ。
bool teachPostAxis(const uint8_t* payload, size_t len);
bool teachPostPose(const uint8_t* payload, size_t len);
// TEACH_FRAME / TEACH_POSE: キューへ積む（順番どおりに全部実行。戻り値は ACK の status、満杯は ST_BUSY）
uint8_t teachQueueAxis(const uint8_t* payload, size_t len);
uint8_t teachQueuePose(const uint8_t* payload, size_t len);
// loopから呼ぶ
void teachEnter();
void teachExit();
void teachUpdate();
bool teachNeutral(uint8_t foot);   // ニュートラル座標を適用し POSE_ECHO を送る
uint32_t teachErrorMask();         // IK到達不可/範囲外などのエラー軸

// ---- 事前転送モーション ----
uint8_t uploadBegin(uint16_t frames);                              // callback（status）
uint8_t uploadData(uint16_t offset, const uint8_t* d, size_t n);   // callback（status）
uint8_t uploadCommit();     // loop: 全フレーム検証(IK含む)。RAMに保持するだけでフラッシュには書かない（1件のみ）
uint16_t uploadHash16();    // 直近のCOMMIT内容の16bitハッシュ（ACKのtokenに入れる）
uint8_t playbackStart();    // loop: RAMの内容を検査して再生準備
bool playbackUpdate();      // loop: 再生中はtrue
void playbackAbort();
bool uploadReady();

class motionBase;

namespace motionList {
    inline motionBase* motionArr[128];
    inline int motionArrNum = 0;
}

class motionBase {
public:
    explicit motionBase(char motionArrUsingBit_ = 0) {
        motionArrUsingBit = motionArrUsingBit_;
        if (motionList::motionArrNum < 128) {
            motionList::motionArr[motionList::motionArrNum] = this;
            motionList::motionArrNum++;
        }
    }

    bool isActive = false;

    virtual bool canStart() = 0;
    virtual bool motionLoop() = 0;
    // 状態遷移などで強制中断されたときに呼ばれる（メンバで状態を持つモーション用）
    virtual void onForceStop() {}
    char motionArrUsingBit = 0;
};

// モーションデータ逐次再生関数の前方宣言
//  logical=false: MotionData.h（旧来の生ICS。旧オフセットで論理値へ変換して出す）
//  logical=true : ExportData.h（PCアプリが書き出した論理ICS。そのまま出す）
bool playCodeMotion(const int16_t motionData[][19], uint16_t totalFrames, bool logical, char motionArrUsingBit, int &taskPhase, uint16_t &currentFrame);

// [PERF] 計測用: データモーションの処理時間の合計（us）と呼び出し回数。main.cpp が1秒ごとに表示する
inline uint32_t g_perfDataUs = 0;
inline uint32_t g_perfDataCalls = 0;

// MotionData系モーションの共通基底クラス
//  DECLARE_DATA_MOTION の第2引数（モーション名の namespace）が MotionData.h のものでも ExportData.h のものでも使える。
//  （配列の型で見分ける: int16_t[][19] = 旧来の生ICS / ExportFrame[] = 論理ICS）
class motionDataBase : public motionBase {
protected:
    const int16_t (*motionData)[19];
    uint16_t totalFrames;
    bool logical;
    int taskPhase = 0;
    uint16_t currentFrame = 0;

public:
    motionDataBase(const int16_t motionData[][19], uint16_t totalFrames, char usingBit = ALL_BIT)
        : motionBase(usingBit), motionData(motionData), totalFrames(totalFrames), logical(false) {}
    motionDataBase(const ExportFrame* frames, uint16_t totalFrames, char usingBit = ALL_BIT)
        : motionBase(usingBit), motionData(reinterpret_cast<const int16_t (*)[19]>(frames)), totalFrames(totalFrames), logical(true) {}

    bool motionLoop() override {
        const uint32_t t0 = micros();                 // [PERF] データモーション1回の処理時間
        const bool running = playCodeMotion(
            motionData,
            totalFrames,
            logical,
            this->motionArrUsingBit,
            taskPhase,
            currentFrame
        );
        g_perfDataUs += micros() - t0;
        ++g_perfDataCalls;
        return running;
    }

    void resetState() {
        taskPhase = 0;
        currentFrame = 0;
    }
    void onForceStop() override { resetState(); }
};

#define DECLARE_MOTION(ClassName, UsingBit) \
    class ClassName : public motionBase { \
    public: \
        ClassName(char bit = UsingBit) : motionBase(bit) {} \
        bool canStart() override; \
        bool motionLoop() override; \
    }; \
    inline ClassName ClassName##_inst;

#define DECLARE_DATA_MOTION(ClassName, TargetNamespace, UsingBit) \
    class ClassName : public motionDataBase { \
    public: \
        ClassName() : motionDataBase(TargetNamespace::flameData, TargetNamespace::flameNum, UsingBit) {} \
        bool canStart() override; \
    }; \
    inline ClassName ClassName##_inst;

namespace motion {
    DECLARE_MOTION(walk1, FOOT_BIT)
    DECLARE_MOTION(walkBack, FOOT_BIT)
    DECLARE_MOTION(walkY, FOOT_BIT)
    DECLARE_MOTION(turn, FOOT_BIT | HIP_BIT)

    //PCアプリケーションで作成したモーションを実行。
    DECLARE_DATA_MOTION(attack_Light_1, syo_kougeki_1, ALL_BIT)
    DECLARE_DATA_MOTION(attack_Light_2, syo_kougeki_2, ALL_BIT)

    DECLARE_DATA_MOTION(attack_Heavy_1, dai_kougeki_1, ALL_BIT)
    DECLARE_DATA_MOTION(attack_Heavy_2, dai_kougeki_2, ALL_BIT)

    //つかんで背負投
    DECLARE_MOTION(attack_TN,ALL_BIT)
    DECLARE_DATA_MOTION(attack_big_T, Owaza_tukami, ALL_BIT)
    DECLARE_DATA_MOTION(attack_big_N, Owaza_nage, ALL_BIT)

    //しゃがみ姿勢に入る、起きるの実装
    DECLARE_DATA_MOTION(stand_to_sit_, stand_to_sit, ALL_BIT)
    DECLARE_DATA_MOTION(sit_to_stand_, sit_to_stand, ALL_BIT)

    //腕用の初期姿勢
    DECLARE_DATA_MOTION(syoki_arm_, syoki_arm, ALL_BIT)

    DECLARE_MOTION(zero, ALL_BIT)//零点調整用
    DECLARE_MOTION(set_free, ALL_BIT)
    DECLARE_MOTION(nop, ALL_BIT)
    DECLARE_MOTION(LOCK_DebugMode, ALL_BIT)
    DECLARE_MOTION(pose, ALL_BIT)//攻撃準備位置

    DECLARE_MOTION(getUp, ALL_BIT)
    DECLARE_DATA_MOTION(getUp_supine,okiagari_2, ALL_BIT)//仰向け
    DECLARE_DATA_MOTION(getUp_prone,okiagari_1, ALL_BIT)//うつぶせ

    DECLARE_MOTION(sit_stand, ALL_BIT)
    DECLARE_MOTION(lowerPos, ALL_BIT)
    DECLARE_MOTION(hip, HIP_BIT)

} // namespace motion

#endif // MOTION_H_