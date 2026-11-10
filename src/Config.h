#ifndef CONFIG_H
#define CONFIG_H
// ==============================================================
// 機体の設定と、全体で使う宣言
//  （旧 Config / Declaration / AssistiveProgram / Calibration をまとめたもの）
//   1. 基本設定・歩容パラメータ・サーボID
//   2. 補助関数（Vector2, map_controller, FrameLimiter など）
//   3. サーボ・脚・コントローラの実体（ServoArray など）
//   4. 校正（calib）
// ==============================================================
#include <Arduino.h>
#include <PS4Controller.h>
#include <esp_gap_bt_api.h>
#include "servoICS.h"

#define DEBUG
//#define SIMULATION
//#define SKIP_IMU      // IMU(MPU6886)の初期化失敗で止まらない・読まない（無印ESP32など、IMUが無い基板でPCアプリを試すとき）
#define SKIP_PS4     // PS4コントローラの接続待ちをしない（PCアプリだけで使うとき。後から接続すれば操作できる）

/**********************************************/
// 基本設定
extern const int serialPC_bps;
extern const int serialServo_bps;

// 構造変更で無くなった軸（左脚1 = サーボ9、右脚1 = サーボ14）。送信も FB 読み取りもしない。
// 論理値（モーション・IK）の計算では従来どおり値を持つが、バスには出ない。
constexpr uint32_t ABSENT_AXIS_MASK = (1UL << 9) | (1UL << 14);
extern const int txPin;
extern const int rxPin;
extern const int enPin;
/**********************************************/
extern const char stretch;
extern int G_fps;

extern float NeutralPoseY;
extern float NeutralPoseX;
extern float NeutralSpac;

extern float footPoseDefLeft;
extern float footPoseDefRight;

struct GaitParameters {
  float T;
  float h;
  float Wd;
  float DutyX;
  float DutyY;
  int Fps;
  float Spac;
  float offsetZ_left;
  float offsetX_left;
  float offsetZ_right;
  float offsetX_right;
  float kickX_left;
  float kickY_left;
  float kickX_right;
  float kickY_right;
  float kickTime;
};

namespace Config {
    // コントローラのMACアドレス
    extern const char ControllerMac[18];

    // サーボとの通信設定
    extern HardwareSerial* ServoSerial;

    extern const char hipServoID;

    // 足寸法
    extern float lengs8[8];

    // 歩行軌道のパラメータ
    extern GaitParameters MV_X_PARAM_3;
    extern GaitParameters MV_X_PARAM_BACK;

    extern GaitParameters MV_Y_SINGLE;

    // サーボID定義
    extern const char leftFootJ1ID;
    extern const char leftFootJ2ID;
    extern const char leftFootJ3ID;
    extern const char leftFootJ4ID;
    extern const char leftFootJ5ID;

    extern const char rightFootJ1ID;
    extern const char rightFootJ2ID;
    extern const char rightFootJ3ID;
    extern const char rightFootJ4ID;
    extern const char rightFootJ5ID;

    extern const char leftArmJ1ID;
    extern const char leftArmJ2ID;
    extern const char leftArmJ3ID;
    extern const char leftArmJ4ID;

    extern const char rightArmJ1ID;
    extern const char rightArmJ2ID;
    extern const char rightArmJ3ID;
    extern const char rightArmJ4ID;

}

/**********************************************/
// 2. 補助関数
// 2次元ベクトル構造体
struct Vector2 {
    float x;
    float y;
    Vector2(float _x = 0.0f, float _y = 0.0f) : x(_x), y(_y) {}
};

/**********************************************/
// 補助関数群
int map_controller(int in, int ineRange, int inMin, int inMax, int outMin, int outMax);

void bondReset();

class FrameLimiter {
public:
    unsigned long interval_ms;
    unsigned long next_target_ms;

    // 指定時間設定（間隔変更時にカウンタをリセット）
    void setInterval(unsigned long interval);

private:
    // 指定時間待ち
    void wait();

    // 時間更新
    void update();

public:
    // 同期を行い、目標時刻からの超過時間(ms)を返す（超過なしは 0）
    long sync();
};

/**********************************************/
// 3. サーボ・脚・コントローラの実体
extern bool g_isFaceUp;
extern float g_lpfAcc;

extern PS4Controller Dualshock4;

/* 宣言・初期化・定数 */
extern servoICS::Servo hipServo;
extern servoICS::Servo leftFoot1;

extern servoICS::Servo leftArmJ1;
extern servoICS::Servo leftArmJ2;
extern servoICS::Servo leftArmJ3;
extern servoICS::Servo leftArmJ4;

extern servoICS::Servo rightArmJ1;
extern servoICS::Servo rightArmJ2;
extern servoICS::Servo rightArmJ3;
extern servoICS::Servo rightArmJ4;

extern servoICS::Servo* ServoArray[19];

extern const int leftArm_index_num;
extern const int rightArm_index_num;
extern const int foot_index_num;

class FootController;   // 本体は FootController.h（ここでは宣言だけ）
extern FootController leftFoot;
extern FootController rightFoot;

void resetRumble(long time);

enum BusyPartsBit : char {
    NON_BIT        = 0b00000000,
    HIP_BIT        = 0b00000001,
    LEFT_ARM_BIT   = 0b00000010,
    RIGHT_ARM_BIT  = 0b00000100,
    LEFT_FOOT_BIT  = 0b00001000,
    RIGHT_FOOT_BIT = 0b00010000,
    FOOT_BIT       = 0b00011000,
    ARM_BIT        = 0b00000110,
    ALL_BIT        = HIP_BIT | FOOT_BIT | ARM_BIT
};

enum ButtonBit : uint32_t {
    RIGHT_BIT    = (1UL << 0),
    DOWN_BIT     = (1UL << 1),
    UP_BIT       = (1UL << 2),
    LEFT_BIT     = (1UL << 3),

    SQUARE_BIT   = (1UL << 4),
    CROSS_BIT    = (1UL << 5),
    CIRCLE_BIT   = (1UL << 6),
    TRIANGLE_BIT = (1UL << 7),

    UPRIGHT_BIT   = (1UL << 8),
    DOWNRIGHT_BIT = (1UL << 9),
    UPLEFT_BIT    = (1UL << 10),
    DOWNLEFT_BIT  = (1UL << 11),

    L1_BIT       = (1UL << 12),
    R1_BIT       = (1UL << 13),
    L2_BIT       = (1UL << 14),
    R2_BIT       = (1UL << 15),

    SHARE_BIT    = (1UL << 16),
    OPTIONS_BIT  = (1UL << 17),
    L3_BIT       = (1UL << 18),
    R3_BIT       = (1UL << 19),

    PS_BIT       = (1UL << 20),
    TOUCHPAD_BIT = (1UL << 21)
};

/**********************************************/
// 4. 校正
// ==============================================================
// 校正（キャリブレーション）
//  - 各サーボのオフセット（生ICSで「論理7500=中立」になる値）と、左右の脚の高さ補正 footPoseDefLeft/Right [mm]
//  - 値の正本は CalibData.h の calib_data（PCアプリの「校正を書き出し」が書き換える。フラッシュには保存しない）。
//    起動時は常にそこから読み込む。PCアプリで調整した値はRAM上だけ（再起動で CalibData.h の値に戻る）。
//  - 「論理値」= setPos(x, true) に渡す値。プロトコル・モーションデータ・UIはすべて論理値で扱い、
//    オフセットを足すのはサーボドライバ(servoICS)の中だけ。
// ==============================================================
namespace calib {

void begin();                       // CalibData.h の値を読み込み → サーボと歩容へ反映

long defaultOffset(int axis);       // 旧データ変換用の固定値（旧main.cppの値。ExportData.h とは無関係）
long offset(int axis);

// 変更はRAM上で即反映（フラッシュには残らない）。
// オフセット変更時もサーボの生の目標は変えない＝ロボットは動かない（論理値の表示が変わる）。
bool setOffset(int axis, long raw);
bool zeroHere(int axis);            // 今の実位置（FB、無ければ最後の指令）を中立(論理7500)にする
void setFootDef(float left, float right);
void loadDefaults();                // CalibData.h の値に戻す
bool dirty();                       // 今の値が CalibData.h と異なる（=ビルドに反映されていない変更がある）

// 旧データ（オフセット無しの生値で記録されたモーション）を論理値へ変換するための既定オフセット差
inline long legacyRawToLogicalIcs(int axis, long raw) { return raw == 0 ? 0 : raw - (defaultOffset(axis) - 7500); }

// 歩容パラメータの offsetZ を NeutralPoseY + footPoseDef + 歩容ごとの補正 で再計算する
void applyGaitOffsets();

// 送信用: [19*u16 offset][f32 footDefLeft][f32 footDefRight][u8 dirty] = 47 byte
void fillPacket(uint8_t* out);
constexpr size_t PACKET_BYTES = 47;

} // namespace calib

#endif // CONFIG_H
