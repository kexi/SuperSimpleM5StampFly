// はじめの二十四歩
// IMU(BMI270)から加速度と角速度を読み、制御周期を400Hzに上げる
//
// 机の上で機体を傾けて、各軸の符号を確認する。ここで確認した向きが
// 姿勢推定の前提になる。あわせてループの処理時間を測り、400Hzで
// 間に合っているかを確かめる。

#include <Arduino.h>

// ../Common/Driver/include/...
#include "driver_flow.h"  // オプティカルフローセンサドライバ
#include "driver_imu.h"   // IMUドライバ

// StampFly/include/...
#include "attitude.h"      // 姿勢推定
#include "driver_led.h"    // LEDドライバ
#include "driver_sound.h"  // サウンドドライバ
#include "driver_timer.h"  // Timerドライバ
#include "mixer.h"         // モーター配分
#include "pid.h"           // PID
#include "safety.h"        // 安全装置

// 制御周期。姿勢制御は速い方が安定するので400Hz(2500us)にする。
// PIDのゲインは周期に依存するため、ゲインを決める前に確定させておく。
#define CONTROL_PERIOD_US 2500

// センサーと通信できたか
static bool is_imu_ready  = false;
static bool is_flow_ready = false;

// ループ処理時間の最大値(us)。400Hzで間に合っているかの確認用。
static int64_t max_elapsed_us = 0;

void setup() {
    USBSerial.begin(115200);
    delay(1000);  // シリアルモニタの接続待ち

    LED_init();
    Sound_init();
    Timer_init(CONTROL_PERIOD_US);

    is_imu_ready  = Imu_init();
    is_flow_ready = Flow_init();

    USBSerial.printf("BMI270  : %s (ID 0x%02X)\n", is_imu_ready ? "OK" : "NG",
                     Imu_getChipID());
    USBSerial.printf("PMW3901 : %s (ID 0x%02X)\n", is_flow_ready ? "OK" : "NG",
                     Flow_getProductID());

    if (is_imu_ready) {
        USBSerial.println("機体を傾けて各軸の符号を確認してください");
        USBSerial.println("ax ay az [m/s^2] / gx gy gz [rad/s] / max_us");
    }

    if (is_imu_ready && is_flow_ready) {
        Sound_play(SOUND_PRESET_BOOT);
    }
}

void loop() {
    Timer_sync();  // フレーム同期

    if (is_imu_ready) {
        Imu_update();
    }

    // 100Hz(4フレームに1回)で読む。フロー・ToFは姿勢ほど速く要らない。
    const bool is_slow_frame = (Timer_getFrameCount() % 4) == 0;
    if (is_flow_ready && is_slow_frame) {
        Flow_update();
    }

    // 処理時間を測る。周期を超えていたら400Hzに間に合っていない。
    const int64_t elapsed = Timer_getElapsedTime();
    if (elapsed > max_elapsed_us) {
        max_elapsed_us = elapsed;
    }

    // 10Hzで表示する。400Hzで出すとシリアルが詰まって計測が狂う。
    if (Timer_getFrameCount() % 40 == 0) {
        float accel[3];
        float gyro[3];
        Imu_getAccel(accel);
        Imu_getGyro(gyro);

        USBSerial.printf("%+6.2f %+6.2f %+6.2f  %+6.2f %+6.2f %+6.2f  %4lld\n",
                         accel[0], accel[1], accel[2], gyro[0], gyro[1],
                         gyro[2], max_elapsed_us);
        max_elapsed_us = 0;  // 次の区間の最大値を取り直す
    }

    const bool is_blink_on = (Timer_getFrameCount() % 200) < 100;
    if (is_imu_ready && is_flow_ready) {
        LED_setColor(0, 0, 100, 0);
    } else if (is_imu_ready || is_flow_ready) {
        LED_setColor(0, is_blink_on ? 100 : 0, is_blink_on ? 60 : 0, 0);
    } else {
        LED_setColor(0, is_blink_on ? 100 : 0, 0, 0);
    }

    LED_update();
    Timer_update();
    Sound_update();
}
