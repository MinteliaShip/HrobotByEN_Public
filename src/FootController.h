#ifndef FOOTCONTROLLER_H
#define FOOTCONTROLLER_H

#include "servoICS.h"
#include <ArduinoEigen.h>
#include <math.h>
#include "Config.h"   // Vector2 など

class FootController {
private:
    struct footJoint5 {
        float J1;
        float J2;
        float J3;
        float J4;
        float J5;
    };

public:
    struct Pose {
        float X;
        float Y;
        float Z;
        float Roll;    // [deg]
        float Pitch;   // [deg]
        float Yaw;     // [deg]
    };

private:
    float* footLeng;
    servoICS::Servo* servoJ1;
    servoICS::Servo* servoJ2;
    servoICS::Servo* servoJ3;
    servoICS::Servo* servoJ4;
    servoICS::Servo* servoJ5;

    bool IK(const Pose& targetPose, int mode, footJoint5& joint);  // false: 到達不可

    int JointInv[5] = {1, 1, 1, 1, 1};

public:
    FootController(servoICS::Servo* ServoArray_[], float footLeng_[]);

    float tread_y(float h, float T, float Duty, float ts_);
    float tread_x(float Wd, float T, float Duty, float ts_);
    Vector2 tread(float h, float Wd, float DutyX, float DutyY, float T, float ts);

    void FootMotorInvert(int J1_, int J2_, int J3_, int J4_, int J5_);

    void setJointAngles(long J1_, long J2_, long J3_, long J4_, long J5_);

    void setTargetPose(const Pose& targetPos, float kick = 0, int mode = 0);   // kick: 足首(J5)への追加角 [deg]
    // setTargetPoseと同じ変換を行うが、サーボへは送信しない。
    // out[5]にはsetPos()へ渡す値(J1..J5)が入る。到達不可/NaNならfalse。
    bool computeJointPos(const Pose& targetPose, float kick, int mode, long out[5]);

private:
    // 計算補助関数
    float calcAngle(float LengA, float LengB, float LengC);
    float length(float x, float y, float z = 0);
};

#endif // FOOTCONTROLLER_H