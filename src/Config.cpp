#include "Config.h"
#include "FootController.h"
#include "CalibData.h"
#include "ExportData.h"

// ==============================================================
// 1. 基本設定・歩容パラメータ・サーボID
// ==============================================================
/**********************************************/
// 基本設定の実体定義
const int serialPC_bps = 115200;
const int serialServo_bps = 115200;
const int txPin = 19;
const int rxPin = 22;
const int enPin = 23;
/**********************************************/

const char stretch = 120;
int G_fps = 40;

float NeutralPoseY = 250;
float NeutralPoseX = -20;
float NeutralSpac = 120;
//左右の足の差をここで吸収させる。Def = 差
float footPoseDefLeft=calib_data::FOOTDEF_L;
float footPoseDefRight=calib_data::FOOTDEF_R;

// グローバル変数の実体定義
bool g_isFaceUp = true;
float g_lpfAcc = 1.0f;

// コントローラのMACアドレス
const char Config::ControllerMac[18] = "06:02:01:02:05:10";

#ifdef SIMULATION
HardwareSerial* Config::ServoSerial = &Serial;
#else
HardwareSerial* Config::ServoSerial = &Serial1;
#endif



// 足寸法
float Config::lengs8[8] = {9.4, 29.219, 25.4, 120, 120.0, 25.4, 25, 11.5};

GaitParameters Config::MV_Y_SINGLE{//そこまで早くない
    0.3,         // T
    80,          // h
    -400,         // Wd
    0.90,        // DutyX
    0.85,        // DutyY
    G_fps,       // Fps
    NeutralSpac-40, // Spac
    NeutralPoseY + footPoseDefLeft, // offsetZ_left
    NeutralPoseX,        // offsetX_left
    NeutralPoseY + footPoseDefRight, // offsetZ_right
    NeutralPoseX,        // offsetX_right
    0.0f,      // kickX_left
    -10.0f,      // kickY_left
    0.0f,      // kickX_right
    -10.0f,      // kickY_right
    0.1f         // kickTime
};

GaitParameters Config::MV_X_PARAM_3{
    0.7,         // T
    90,          // h
    150,          // Wd
    0.90,        // DutyX
    0.88,        // DutyY
    G_fps,       // Fps
    NeutralSpac*0.6, // Spac
    NeutralPoseY + footPoseDefLeft, // offsetZ_left
    NeutralPoseX,        // offsetX_left
    NeutralPoseY + footPoseDefRight, // offsetZ_right
    NeutralPoseX,        // offsetX_right
    -0.0f,       // kickX_left
    -25.0f,      // kickY_left
    -0.0f,       // kickX_right
    -30.0f,      // kickY_right
    0.15f        // kickTime
};

GaitParameters Config::MV_X_PARAM_BACK{
    0.7,         // T
    70,          // h
    -150,          // Wd
    0.90,        // DutyX
    0.88,        // DutyY
    G_fps,       // Fps
    NeutralSpac*0.6, // Spac
    NeutralPoseY + footPoseDefLeft, // offsetZ_left
    NeutralPoseX,        // offsetX_left
    NeutralPoseY + footPoseDefRight, // offsetZ_right
    NeutralPoseX,        // offsetX_right
    -0.0f,       // kickX_left
    -15.0f,      // kickY_left
    -0.0f,       // kickX_right
    -20.0f,      // kickY_right
    0.15f        // kickTime
};

const char Config::hipServoID = 11;

const char Config::leftFootJ1ID = 10;
const char Config::leftFootJ2ID = 16;
const char Config::leftFootJ3ID = 17;
const char Config::leftFootJ4ID = 18;
const char Config::leftFootJ5ID = 19;

const char Config::rightFootJ1ID = 9;
const char Config::rightFootJ2ID = 12;
const char Config::rightFootJ3ID = 14;
const char Config::rightFootJ4ID = 13;
const char Config::rightFootJ5ID = 15;

const char Config::leftArmJ1ID = 1;
const char Config::leftArmJ2ID = 3;
const char Config::leftArmJ3ID = 4;
const char Config::leftArmJ4ID = 5;

const char Config::rightArmJ1ID = 2;
const char Config::rightArmJ2ID = 6;
const char Config::rightArmJ3ID = 7;
const char Config::rightArmJ4ID = 8;

// ==============================================================
// 2. 補助関数
// ==============================================================
int map_controller(int in, int ineRange, int inMin, int inMax, int outMin, int outMax) {
    int in_abs = abs(in);
    int in_pn = (in_abs != 0) ? (in / in_abs) : 0;

    int in_cut;
    if (in_abs < ineRange) {
        in_cut = 0;
    } else {
        in_cut = in - ineRange * in_pn;
    }

    return map(in_cut, inMin + ineRange, inMax - ineRange, outMin, outMax);
}

void bondReset() {
    int dev_num = esp_bt_gap_get_bond_device_num();

    if (dev_num > 0) {
        esp_bd_addr_t dev_list[dev_num];
        esp_bt_gap_get_bond_device_list(&dev_num, dev_list);
        
        for (int i = 0; i < dev_num; i++) {
            esp_bt_gap_remove_bond_device(dev_list[i]);
        }
    }
}

void FrameLimiter::setInterval(unsigned long interval) {
    interval_ms = interval;
    next_target_ms = millis() + interval_ms;
}

void FrameLimiter::wait() {
    unsigned long now = millis();
    if ((long)(next_target_ms - now) > 0) {
        delay(next_target_ms - now);
    }
}

void FrameLimiter::update() {
    next_target_ms += interval_ms;
    if ((long)(millis() - next_target_ms) > 0) {
        next_target_ms = millis() + interval_ms;
    }
}

long FrameLimiter::sync() {
    unsigned long now = millis();
    long overrun = (long)(now - next_target_ms);

    wait();
    update();

    return (overrun > 0) ? overrun : 0;
}

// ==============================================================
// 3. サーボ・脚・コントローラの実体
// ==============================================================
PS4Controller Dualshock4; // 実体の定義

servoICS::Servo hipServo(Config::ServoSerial, enPin, Config::hipServoID);
servoICS::Servo leftFoot1(Config::ServoSerial, enPin, Config::leftFootJ1ID);
servoICS::Servo leftFoot2(Config::ServoSerial, enPin, Config::leftFootJ2ID);
servoICS::Servo leftFoot3(Config::ServoSerial, enPin, Config::leftFootJ3ID);
servoICS::Servo leftFoot4(Config::ServoSerial, enPin, Config::leftFootJ4ID);
servoICS::Servo leftFoot5(Config::ServoSerial, enPin, Config::leftFootJ5ID);

servoICS::Servo rightFoot1(Config::ServoSerial, enPin, Config::rightFootJ1ID);
servoICS::Servo rightFoot2(Config::ServoSerial, enPin, Config::rightFootJ2ID);
servoICS::Servo rightFoot3(Config::ServoSerial, enPin, Config::rightFootJ3ID);
servoICS::Servo rightFoot4(Config::ServoSerial, enPin, Config::rightFootJ4ID);
servoICS::Servo rightFoot5(Config::ServoSerial, enPin, Config::rightFootJ5ID);

servoICS::Servo leftArmJ1(Config::ServoSerial, enPin, Config::leftArmJ1ID);
servoICS::Servo leftArmJ2(Config::ServoSerial, enPin, Config::leftArmJ2ID);
servoICS::Servo leftArmJ3(Config::ServoSerial, enPin, Config::leftArmJ3ID);
servoICS::Servo leftArmJ4(Config::ServoSerial, enPin, Config::leftArmJ4ID);

servoICS::Servo rightArmJ1(Config::ServoSerial, enPin, Config::rightArmJ1ID);
servoICS::Servo rightArmJ2(Config::ServoSerial, enPin, Config::rightArmJ2ID);
servoICS::Servo rightArmJ3(Config::ServoSerial, enPin, Config::rightArmJ3ID);
servoICS::Servo rightArmJ4(Config::ServoSerial, enPin, Config::rightArmJ4ID);

servoICS::Servo* ServoArray[19]{
    &hipServo,          // 0
    &leftArmJ1,         // 1
    &leftArmJ2,         // 2
    &leftArmJ3,         // 3
    &leftArmJ4,         // 4
    &rightArmJ1,        // 5
    &rightArmJ2,        // 6
    &rightArmJ3,        // 7
    &rightArmJ4,        // 8
    &leftFoot1,         // 9
    &leftFoot2,         // 10
    &leftFoot3,         // 11
    &leftFoot4,         // 12
    &leftFoot5,         // 13
    &rightFoot1,        // 14
    &rightFoot2,        // 15
    &rightFoot3,        // 16
    &rightFoot4,        // 17
    &rightFoot5         // 18
};

const int leftArm_index_num = 1;
const int rightArm_index_num = 5;
const int foot_index_num = 9;

FootController leftFoot(&ServoArray[9], Config::lengs8);
FootController rightFoot(&ServoArray[14], Config::lengs8);

// ==============================================================
// 4. 校正（値の正本は CalibData.h の calib_data）
// ==============================================================
namespace calib {
namespace {

// 旧 main.cpp の ServoArray_pos と同じ値。旧モーション(MotionData.h)の生ICSを論理値へ変換するためだけに使う。
// ※機体ごとの校正値は ExportData.h。ここは変えないこと（変えると旧モーションの角度がずれる）。
const int16_t LEGACY_OFFSET[19] = {
    7500,
    7500, 7500, 7500, 7500,
    7500, 7500, 7500, 7500,
    7500, 7676, 7710, 7777, 7600,
    7500, 7575, 7299, 7299, 7720
};

int16_t g_offset[19];

bool headerValid() {
    for (int i = 0; i < 19; ++i) {
        if (calib_data::OFFSET[i] < 3500 || calib_data::OFFSET[i] > 11500) return false;
    }
    const float l = calib_data::FOOTDEF_L, r = calib_data::FOOTDEF_R;
    return isfinite(l) && isfinite(r) && fabsf(l) <= 100 && fabsf(r) <= 100;
}

void applyOffsets() {
    for (int i = 0; i < 19; ++i) {
        if (ServoArray[i] != nullptr) ServoArray[i]->setOffsetIcs(g_offset[i]);
    }
}

// 歩容ごとの高さ補正。値は「旧Config.cppの値と完全に同じになる」ように決めてある。
//  旧: offsetZ_left = NeutralPoseY + 3, offsetZ_right = NeutralPoseY + 0 （footPoseDefを使っていない歩容）
//  新: offsetZ = NeutralPoseY + footPoseDef(側) + trim  → trimL = 3-0, trimR = 0-20
//  ※つまり既存の前後歩行・旋回は「右+20mm補正」を打ち消している。意図的でなければ trimR を0にすること。
struct GaitTrim { GaitParameters* p; float trimL; float trimR; };
GaitTrim GAITS[] = {
    {&Config::MV_X_PARAM_3,     0.0f,   0.0f},
    {&Config::MV_X_PARAM_BACK,  0.0f,   0.0f},
    {&Config::MV_Y_SINGLE,      0.0f,   0.0f},
};

} // namespace

long defaultOffset(int axis) { return (axis >= 0 && axis < 19) ? LEGACY_OFFSET[axis] : 7500; }
long offset(int axis) { return (axis >= 0 && axis < 19) ? g_offset[axis] : 7500; }

bool dirty() {
    for (int i = 0; i < 19; ++i) if (g_offset[i] != calib_data::OFFSET[i]) return true;
    return footPoseDefLeft != calib_data::FOOTDEF_L || footPoseDefRight != calib_data::FOOTDEF_R;
}

void applyGaitOffsets() {
    for (auto& g : GAITS) {
        g.p->offsetZ_left = NeutralPoseY + footPoseDefLeft + g.trimL;
        g.p->offsetZ_right = NeutralPoseY + footPoseDefRight + g.trimR;
    }
}

void loadDefaults() {
    for (int i = 0; i < 19; ++i) g_offset[i] = calib_data::OFFSET[i];
    footPoseDefLeft = calib_data::FOOTDEF_L;
    footPoseDefRight = calib_data::FOOTDEF_R;
    applyOffsets();
    applyGaitOffsets();
}

void begin() {
    const bool ok = headerValid();
    if (ok) {
        loadDefaults();
    } else {
        // CalibData.h の値が範囲外。黙って別の値で動かさず、中立(7500)・補正0で起動して知らせる。
        for (int i = 0; i < 19; ++i) g_offset[i] = 7500;
        footPoseDefLeft = 0.0f;
        footPoseDefRight = 0.0f;
        applyOffsets();
        applyGaitOffsets();
    }
    Serial.printf("[CALIB] %s\n", ok ? "Loaded CalibData.h" : "CalibData.h is out of range: using neutral 7500 (fix the header!)");
    if (export_data::FPS != G_fps)
        Serial.printf("[WARN] ExportData.h was exported at %u fps, but G_fps is %d. Exported motions will play at a different speed.\n", export_data::FPS, G_fps);
}

bool setOffset(int axis, long raw) {
    if (axis < 0 || axis >= 19 || raw < 3500 || raw > 11500) return false;
    g_offset[axis] = static_cast<int16_t>(raw);
    if (ServoArray[axis] != nullptr) ServoArray[axis]->setOffsetIcs(raw);
    // サーボの「生の目標」は変えていないので、この時点でロボットは動かない。
    return true;
}

bool zeroHere(int axis) {
    if (axis < 0 || axis >= 19 || ServoArray[axis] == nullptr) return false;
    if (ABSENT_AXIS_MASK & (1UL << axis)) return true;   // 無い軸は何もしない（全軸ゼロの途中で失敗扱いにしない）
    long raw = 0;
    if (!servoBus::getFb(axis, raw, 500)) raw = ServoArray[axis]->getLastSetPosIcs();
    if (raw <= 0) return false;
    if (!setOffset(axis, raw)) return false;
    // 保持中の軸は、新しい中立(=今の位置)をそのまま目標にする
    if (ServoArray[axis]->hasTarget() && ServoArray[axis]->targetRaw() != 0) ServoArray[axis]->setPos(raw, false);
    return true;
}

void setFootDef(float left, float right) {
    if (!isfinite(left) || !isfinite(right) || fabsf(left) > 100 || fabsf(right) > 100) return;
    footPoseDefLeft = left;
    footPoseDefRight = right;
    applyGaitOffsets();
}

void fillPacket(uint8_t* out) {
    for (int i = 0; i < 19; ++i) {
        out[i * 2] = static_cast<uint8_t>(g_offset[i] & 0xFF);
        out[i * 2 + 1] = static_cast<uint8_t>((g_offset[i] >> 8) & 0xFF);
    }
    memcpy(out + 38, &footPoseDefLeft, 4);
    memcpy(out + 42, &footPoseDefRight, 4);
    out[46] = dirty() ? 1 : 0;
}

} // namespace calib
