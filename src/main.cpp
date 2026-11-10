#include "Config.h"
#include "Motion.h"
#include "RobotController.h"
#include "servoICS.h"
#include <Wire.h>
#ifndef SIMULATION
#include <driver/uart.h>
#endif

// MPU6886 I2C設定
#define MPU6886_ADDRESS 0x68
#define SDA_PIN 25
#define SCL_PIN 21

// ローパスフィルタの平滑化係数 (0.0 < LPF_ALPHA <= 1.0)
// 値を小さくするほど振動ノイズに強くなります
static const float LPF_ALPHA = 0.2f;

// 内部ヘルパー関数：I2Cレジスタ書き込み
static void writeRegister(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(MPU6886_ADDRESS);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

bool setupIMU() {
    Wire.begin(SDA_PIN, SCL_PIN, 400000); // 400kHz Fast Mode
    delay(50);

    // WHO_AM_I レジスタの検証 (0x75 -> 0x19)
    Wire.beginTransmission(MPU6886_ADDRESS);
    Wire.write(0x75);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6886_ADDRESS, (size_t)1);
    if (Wire.read() != 0x19) {
        return false; // 通信エラーまたはデバイス不一致
    }

    writeRegister(0x6B, 0x00); // スリープ解除
    delay(10);
    writeRegister(0x1C, 0x10); // ACCEL_CONFIG: ±8g (4096 LSB/g)
    writeRegister(0x1A, 0x03); // 内蔵DLPF (~42Hz) でハードウェア帯域制限

    return true;
}


void updateIMU() {
#ifdef SKIP_IMU
    return;   // IMUなし: 表裏判定は初期値のまま
#endif
    Wire.beginTransmission(MPU6886_ADDRESS);
    Wire.write(0x3F);   // ACCEL_ZOUT_H（Z軸）
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6886_ADDRESS, (size_t)2);

    int16_t rawAccZ = (Wire.read() << 8) | Wire.read();
    float rawG = (float)rawAccZ / 4096.0f;

    // 1次ローパスフィルタ（LPF）演算
    g_lpfAcc = (1.0f - LPF_ALPHA) * g_lpfAcc + LPF_ALPHA * rawG;

    // ヒステリシス付き表裏判定
    if (g_lpfAcc > 0.1f) {
        g_isFaceUp = true;   // 表向き　仰向け
    } else if (g_lpfAcc < -0.1f) {
        g_isFaceUp = false;  // 裏向き　うつ伏せ
    }
}


void taskManager();

void loop2(void *p);

void loop2_begin(){
    xTaskCreatePinnedToCore(
        loop2,           // タスク関数名
        "Loop2Task",     // タスク名（デバッグ用文字列）
        4096,            // スタックサイズ（バイト単位。必要に応じて調整）
        NULL,            // タスク引数
        1,               // 優先度（1〜24、数値が大きいほど優先）
        NULL,            // タスクハンドル
        0                // 割り当てるコア番号（0 または 1）
    );
}

inline void checkMemoryUsage(unsigned long intervalMs = 1000) {
    static unsigned long lastCheck = 0;
    unsigned long now = millis();

    // 指定インターバル（デフォルト1000ms）経過していない場合は即復帰（極小負荷）
    if (now - lastCheck < intervalMs) {
        return;
    }
    lastCheck = now;

    uint32_t total = ESP.getHeapSize();
    uint32_t free = ESP.getFreeHeap();
    uint32_t minFree = ESP.getMinFreeHeap();
    uint32_t used = total - free;
    float usageRatio = ((float)used / (float)total) * 100.0f;

    // 1行で要点をまとめた軽量シリアル出力
    Serial.printf("[RAM] Used: %u B (%.1f%%) | Free: %u B | MinFree: %u B\n", 
                  used, usageRatio, free, minFree);

    // メモリ危険域（20KB以下）の警告
    if (free < 20000) {
        Serial.printf("[WARN] Critical Low RAM! Free: %u B\n", free);
    }
}


static char g_usingPartsBit = 0;

bool runExclusiveTask(motionBase* motion) {
    if (motion->isActive) {
        // 実行中の場合：モーションを継続し、終了したら解放
        if (!motion->motionLoop()) {
            motion->isActive = false;
            g_usingPartsBit &= ~(motion->motionArrUsingBit);
        }
        return true;
    } 
    else if (motion->canStart() && ((g_usingPartsBit & motion->motionArrUsingBit) == 0)) {
        // 停止中で、リソースが空いている場合：起動条件を満たしていれば開始
        motion->isActive = true;
        g_usingPartsBit |= motion->motionArrUsingBit;
        
        // 初回実行
        if (!motion->motionLoop()) {
            motion->isActive = false;
            g_usingPartsBit &= ~(motion->motionArrUsingBit);
        }
        return true;
    }
    return false; // 他のタスクが占有中、または起動条件を満たさないため実行不可
}

// モードごとの処理。モードの遷移そのものは ctrl::update()（RobotController）が行う。
void taskManager() {
    static uint8_t prevMode = 0xFF;
    const uint8_t mode = ctrl::mode();
    if (mode != prevMode) {
        // モードが変わったら部位の占有も解放（モーションの中断は ctrl 側で済んでいる）
        g_usingPartsBit = 0;
        prevMode = mode;
    }

    switch (mode) {
    case proto::MODE_PLAYBACK:
        if (!playbackUpdate()) ctrl::playbackFinished();
        break;
    case proto::MODE_TEACH:
        teachUpdate();
        break;
    case proto::MODE_ESTOP:
        // 緊急停止中: 自律モーションは動かさない（FREEにした軸以外はその場で保持）
        break;
    case proto::MODE_AUTO:
    default:
        for (int i = 0; i < motionList::motionArrNum; ++i) {
            runExclusiveTask(motionList::motionArr[i]);
        }
        break;
    }
}

FrameLimiter framelim;

void setup() {
    Serial.begin(serialPC_bps);
    Serial1.begin(serialServo_bps, SERIAL_8E1, rxPin, txPin);
    // 返り値がないサーボ1軸あたりの待ち時間。ICSの返信は1ms程度で来るので10msで十分。
    // （旧100msだと、応答しない軸のFBを読むたびに制御周期が100ms止まっていた）
    Serial1.setTimeout(10);
#ifndef SIMULATION
    // 受信バイトがソフトから見えるまでの遅れ(RX FIFOタイムアウト)を最小(1文字分≒0.1ms)にする。
    // 既定(約10文字≒1ms)だとサーボの返信が遅れて見え、返信待ちの判定を誤る原因になる。
    uart_set_rx_timeout(UART_NUM_1, 1);
#endif

#ifndef SKIP_IMU
    if (!setupIMU()) {
        Serial.println("IMU Initialization Failed!");
        while (1) delay(100);
    }
#else
    Serial.println("[SKIP_IMU] IMU init skipped.");
#endif

    loop2_begin();
    delay(500);

    // ★ 1. 先に PS4 コントローラ（Classic BT）の接続待ち受けを開始
    Dualshock4.begin(Config::ControllerMac);
    bondReset();

#ifndef SKIP_PS4
    Serial.println("Waiting for PS4 Controller...");
    while (1) {
        if (Dualshock4.isConnected()) {
            Dualshock4.setLed(255, 255, 255);
            Dualshock4.setRumble(0, 255);
            Dualshock4.sendToController();
            resetRumble(500);
            break;
        }
        delay(10);
    }
    Serial.println("PS4 Controller Connected!");
#else
    Serial.println("[SKIP_PS4] PS4 Controller wait skipped.");
    // 元の順序（PS4ライブラリのBluetooth名設定 "ESP Host" → BLE初期化 "ESP32_Robot"）を保つため、接続待ちの代わりに待つ。
    delay(1500);
#endif

    // 校正データとモーションはコンパイル時に組み込む（CalibData.h と ExportData.h）。
    // プレビュー用モーションはRAMだけに置くので、LittleFSは使わない（マウントもしない）。

    // ★ 2. PS4接続の完了後にBLE広告を開始
    setupBLE();

    #ifndef SIMULATION

        leftFoot.FootMotorInvert(1, 1, -1, -1, 1);
        rightFoot.FootMotorInvert(1, 1, 1, 1, 1);

        // オフセットと左右差補正は CalibData.h の calib_data から（フラッシュには保存しない）。
        calib::begin();
        for (int i = 0; i < 19; i++) {
            ServoArray[i]->setStretch(stretch);
            delay(5);
        }
        ServoArray[13]->setStretch(10);   // 足首5は柔らかく（モーション中も Motion.cpp の axisStretch() で維持）
        delay(5);
        ServoArray[18]->setStretch(10);
        delay(5);
        ServoArray[0]->setSkip(true);

        // ここから先、サーボへの送信はすべて ServoBus 経由（loop末尾の flush でまとめて送る）
        servoBus::begin();
    #endif

    if (Dualshock4.isConnected()) {   // SKIP_PS4 で未接続のときは送らない
        Dualshock4.setLed(255, 0, 0);
        Dualshock4.sendToController();
    }

    framelim.setInterval(1000 / G_fps);
}

void loop() {
    // [PERF] 計測: モーション処理と送信の時間を1秒ごとに表示する（Serial = USB。Serial1 のサーボバスには出さない）
    static uint32_t pN = 0, pSumTask = 0, pSumFlush = 0, pMaxTask = 0, pMaxFlush = 0, pLast = 0;
    static uint32_t pSumData = 0, pCalls = 0;

    ctrl::update();      // BLEからの要求・状態遷移
    const uint32_t tTask0 = micros();
    taskManager();       // モードごとの処理（目標値の登録だけ。送信はまだ）
    const uint32_t dTask = micros() - tTask0;

    updateIMU();
    {
        static int lastFace = -2;
        const int face = g_isFaceUp ? 1 : -1;
        if (face != lastFace) {          // 旧: 毎周期println。変化時だけにした
            Serial.println(face > 0 ? ">Status:1" : ">Status:-1");
            lastFace = face;
        }

        Serial.printf("g_isFaceUp:%d\n",g_isFaceUp);
    }

    updateBLE();         // ACK送信・再広告

    // この周期の残り時間を送信予算にする（送り切れない軸は次周期へ。周期は延ばさない）
    long remainMs = static_cast<long>(framelim.next_target_ms - millis()) - 2;
    if (remainMs < 3) remainMs = 3;
    if (remainMs > 20) remainMs = 20;
    const uint32_t tFlush0 = micros();
    servoBus::flush(static_cast<uint32_t>(remainMs) * 1000UL);
    const uint32_t dFlush = micros() - tFlush0;

    ++pN;
    pSumTask += dTask; pSumFlush += dFlush;
    if (dTask > pMaxTask) pMaxTask = dTask;
    if (dFlush > pMaxFlush) pMaxFlush = dFlush;
    if (millis() - pLast >= 1000) {
        pSumData += g_perfDataUs; pCalls += g_perfDataCalls;   // 1秒分のデータモーション時間
        Serial.printf("[PERF] n=%lu task avg=%luus max=%luus | flush avg=%luus max=%luus | data motion %lu calls, %luus/s\n",
            static_cast<unsigned long>(pN), static_cast<unsigned long>(pSumTask / pN), static_cast<unsigned long>(pMaxTask),
            static_cast<unsigned long>(pSumFlush / pN), static_cast<unsigned long>(pMaxFlush),
            static_cast<unsigned long>(pCalls), static_cast<unsigned long>(pSumData));
        pN = pSumTask = pSumFlush = pMaxTask = pMaxFlush = 0;
        pSumData = pCalls = 0;
        g_perfDataUs = 0; g_perfDataCalls = 0;
        pLast = millis();
    }

    ctrl::telemetry();   // STATE/FB を20Hzで送信

    const long overrun = framelim.sync();
    if (overrun > 0) Serial.printf("[LOOP] overrun %ld ms\n", overrun);
}

namespace loop2_task
{
    // resetRumble
    volatile unsigned long resetRumble_time=0;
    volatile unsigned long startTime = 0;
    volatile bool resetRumble_flag=false;
}

void resetRumble(long time_ms) {
    loop2_task::startTime = millis();
    loop2_task::resetRumble_time = time_ms;
    loop2_task::resetRumble_flag = true;
}

void loop2(void *p){
    while (true) {
        //[resetRumble]タスク
        // フラグが立っており、指定された時間が経過したか判定
        if (loop2_task::resetRumble_flag) {
            if (millis() - loop2_task::startTime >= loop2_task::resetRumble_time) {
                // 1. フラグを降ろす（2重実行防止）
                loop2_task::resetRumble_flag = false;

                // 2. コントローラーの振動をリセットして送信
                if (Dualshock4.isConnected()) {
                    Dualshock4.setRumble(0, 0);
                    Dualshock4.sendToController();
                }
            }
        }
        checkMemoryUsage();

        delay(1);
    }
}
