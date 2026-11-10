#include "FootController.h"

bool FootController::IK(const Pose& targetPose, int mode, footJoint5& joint) {
    // 姿勢角は度で受け取り、三角関数と関節角は弧度法で計算する
    const float roll = targetPose.Roll * DEG_TO_RAD;
    const float yaw = targetPose.Yaw * DEG_TO_RAD;

    Eigen::Vector4d Pe = {targetPose.X, targetPose.Y, targetPose.Z, 1};
    Eigen::Matrix4d T2_1;
    T2_1 <<
    1, 0,                   0,                    -footLeng[1],
    0, cos(roll), sin(roll), 0,
    0, -sin(roll),cos(roll), -footLeng[0],
    0, 0,                   0,                    1;

    Eigen::Vector4d P2_e = T2_1 * Pe;
    joint.J1 = roll;

    Eigen::Vector4d P6_e = {cos(yaw) * footLeng[6], sin(yaw) * footLeng[6], -footLeng[7], 0};
    Eigen::Vector4d P2_6 = P2_e - P6_e;

    joint.J2 = atan2(P2_6(1), P2_6(0));
    joint.J5 = yaw - joint.J2;

    Eigen::Matrix4d T3_2;
    T3_2 <<
    cos(joint.J2),  sin(joint.J2), 0, -footLeng[2],
    -sin(joint.J2), cos(joint.J2), 0, 0,
    0,              0,             1, 0,
    0,              0,             0, 1;

    Eigen::Vector4d P3_6 = T3_2 * P2_6;
    Eigen::Vector4d P5_6 = {footLeng[5], 0, 0, 0};
    Eigen::Vector4d P3_5 = P3_6 - P5_6;

    float LengQ = length(P3_5(0), P3_5(2));

    // 到達可能範囲の判定（脛と腿の2リンク）。範囲外ではacosがNaNになり、
    // NaNがそのままICS値へ変換されてサーボが暴走するのを防ぐ。
    const float reachMax = footLeng[3] + footLeng[4];
    const float reachMin = fabsf(footLeng[3] - footLeng[4]);
    const float tol = 0.5f; // [mm] 伸び切り付近の丸め誤差許容
    if (!(LengQ > 1e-3f) || LengQ > reachMax + tol || LengQ < reachMin - tol) {
        return false;
    }

    if (mode == 0) {
        joint.J3 = atan2(P3_5(2), P3_5(0)) - calcAngle(footLeng[4], footLeng[3], LengQ);
        joint.J4 = PI - calcAngle(LengQ, footLeng[4], footLeng[3]);
    } else {
        joint.J3 = atan2(P3_5(2), P3_5(0)) + calcAngle(footLeng[4], footLeng[3], LengQ);
        joint.J4 = -PI + calcAngle(LengQ, footLeng[4], footLeng[3]);
    }

    return isfinite(joint.J1) && isfinite(joint.J2) && isfinite(joint.J3) &&
           isfinite(joint.J4) && isfinite(joint.J5);
}

float FootController::tread_y(float h, float T, float Duty, float ts_) {
    float A = 2.0f * h / (1.0f - Duty);
    float AngV = PI * 4.0f / ((1.0f - Duty) * T);
    float t_1 = Duty * T / 2.0f;
    float t_2 = T / 2.0f;
    float t_3 = T - Duty * T / 2.0f;

    float y_ = 0.0f;

    if (ts_ < t_1) {
        y_ = 0;
    } else if (ts_ < t_2) {
        y_ = -A / (2 * AngV) * sin(AngV * (ts_ - t_1)) + A / 2 * (ts_ - t_1);
    } else if (ts_ < t_3) {
        y_ = A / (2 * AngV) * sin(AngV * (ts_ - t_2)) - A / 2 * (ts_ - t_2) + (1.0f - Duty) * T * A / 4.0f;
    } else {
        y_ = 0;
    }
    return y_;
}

float FootController::tread_x(float Wd, float T, float Duty, float ts_) {
    float A = 2.0f * Wd / (1.0f - Duty);
    float AngV = PI * 2.0f / ((1.0f - Duty) * T);
    float t_1 = Duty * T / 2.0f;
    float t_2 = T - Duty * T / 2.0f;
    float x_ = 0.0f;

    if (ts_ < t_1) {
        x_ = -Wd * ts_;
    } else if (ts_ < t_2) {
        float dt = ts_ - t_1;
        x_ = -A / (2.0f * AngV) * sinf(AngV * dt) + (A * 0.5f - Wd) * dt - Wd * t_1;
    } else {
        x_ = -Wd * (ts_ - t_2) + Wd * t_1;
    }
    return x_;
}

Vector2 FootController::tread(float h, float Wd, float DutyX, float DutyY, float T, float ts) {
    Vector2 result;
    result.x = tread_x(Wd, T, DutyX, ts);
    result.y = tread_y(h, T, DutyY, ts);
    return result;
}

void FootController::FootMotorInvert(int J1_, int J2_, int J3_, int J4_, int J5_) {
    JointInv[0] = J1_;
    JointInv[1] = J2_;
    JointInv[2] = J3_;
    JointInv[3] = J4_;
    JointInv[4] = J5_;
}

FootController::FootController(servoICS::Servo* ServoArray_[], float footLeng_[]) {
    servoJ1 = ServoArray_[0];
    servoJ2 = ServoArray_[1];
    servoJ3 = ServoArray_[2];
    servoJ4 = ServoArray_[3];
    servoJ5 = ServoArray_[4];
    footLeng = footLeng_;
}

void FootController::setJointAngles(long J1_, long J2_, long J3_, long J4_, long J5_) {
    servoJ1->setPos(J1_);
    servoJ2->setPos(J2_);
    servoJ3->setPos(J3_);
    servoJ4->setPos(J4_);
    servoJ5->setPos(J5_);
}

bool FootController::computeJointPos(const Pose& targetPose, float kick, int mode, long out[5]) {
    footJoint5 joint;
    if (!IK(targetPose, mode, joint)) return false;
    const float kickRad = kick * DEG_TO_RAD;   // kick は度で受け取る

#ifdef SIMULATION
    out[0] = static_cast<long>(servoICS::fromRad_toIcs(joint.J1));
    out[1] = static_cast<long>(servoICS::fromRad_toIcs(joint.J2));
    out[2] = static_cast<long>(servoICS::fromRad_toIcs(joint.J3));
    out[3] = static_cast<long>(servoICS::fromRad_toIcs(joint.J4 + joint.J3)); // 平行リンク補正
    out[4] = static_cast<long>(servoICS::fromRad_toIcs(joint.J5 + kickRad));
#else
    out[0] = static_cast<long>(servoICS::fromRad_toIcs(joint.J1 * JointInv[0]));
    out[1] = static_cast<long>(servoICS::fromRad_toIcs(joint.J2 * JointInv[1]));
    out[2] = static_cast<long>(servoICS::fromRad_toIcs(-joint.J3 * JointInv[2]));
    out[3] = static_cast<long>(servoICS::fromRad_toIcs((joint.J4 + joint.J3) * JointInv[3])); // 平行リンク補正
    out[4] = static_cast<long>(servoICS::fromRad_toIcs((-joint.J5 - kickRad) * JointInv[4]));
#endif
    return true;
}

void FootController::setTargetPose(const Pose& targetPose, float kick, int mode) {
    long pos[5];
    // 到達不可の場合は送信せず、直前の指令を保持する（NaN指令の防止）。
    if (!computeJointPos(targetPose, kick, mode, pos)) return;
    setJointAngles(pos[0], pos[1], pos[2], pos[3], pos[4]);
}

float FootController::calcAngle(float LengA, float LengB, float LengC) {
    float v = (-LengA * LengA + LengB * LengB + LengC * LengC) / (2 * LengB * LengC);
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    return acos(v);
}

float FootController::length(float x, float y, float z) {
    return sqrt(x * x + y * y + z * z);
}