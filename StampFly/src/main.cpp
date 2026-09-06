// はじめの二十三歩
// SPIバスを共有ドライバに分けて、IMU(BMI270)とオプティカルフローセンサ
// (PMW3901)の両方と通信できることを確認する
//
// 2つのセンサーは同じSPIバスにぶら下がっていてCSピンだけで区別する。
// 片方を読んだ後にもう片方が化けないことが、ここでの確認事項。

#include <Arduino.h>

// ../Common/Driver/include/...
#include "driver_flow.h"   // オプティカルフローセンサドライバ
#include "driver_imu.h"    // IMUドライバ
#include "driver_led.h"    // LEDドライバ
#include "driver_sound.h"  // サウンドドライバ
#include "driver_timer.h"  // Timerドライバ

// センサーと通信できたか
static bool is_imu_ready  = false;
static bool is_flow_ready = false;

void setup() {
    USBSerial.begin(115200);
    delay(1000);  // シリアルモニタの接続待ち

    // 各ドライバーの初期化
    LED_init();         // フルカラーLEDの初期化
    Sound_init();       // サウンドの初期化
    Timer_init(10000);  // タイマーの初期化(10000us = 10ms)

    // 2つのセンサーの初期化(SPIバスは共有される)
    is_imu_ready  = Imu_init();
    is_flow_ready = Flow_init();

    USBSerial.printf("BMI270  Chip ID    : 0x%02X (expected 0x24) %s\n",
                     Imu_getChipID(), is_imu_ready ? "OK" : "NG");
    USBSerial.printf("PMW3901 Product ID : 0x%02X (expected 0x49) %s\n",
                     Flow_getProductID(), is_flow_ready ? "OK" : "NG");

    // 交互に読んで、お互いの通信が干渉していないことを確かめる。
    // CSの扱いを間違えていると、ここで値が化ける。
    USBSerial.println("--- 交互に読み直して確認 ---");
    for (int i = 0; i < 3; i++) {
        USBSerial.printf("%d: BMI270 0x%02X / PMW3901 0x%02X\n", i,
                         Imu_getChipID(), Flow_getProductID());
    }

    if (is_imu_ready && is_flow_ready) {
        Sound_play(SOUND_PRESET_BOOT);
    }
}

void loop() {
    Timer_sync();  // フレーム同期

    const bool is_blink_on = (Timer_getFrameCount() % 100) < 50;

    // 両方OKなら緑、片方だけなら黄色、どちらもダメなら赤で点滅させる
    if (is_imu_ready && is_flow_ready) {
        LED_setColor(0, 0, 100, 0);
    } else if (is_imu_ready || is_flow_ready) {
        LED_setColor(0, is_blink_on ? 100 : 0, is_blink_on ? 60 : 0, 0);
    } else {
        LED_setColor(0, is_blink_on ? 100 : 0, 0, 0);
    }

    LED_update();    // フルカラーLEDの更新
    Timer_update();  // タイマーの更新
    Sound_update();  // サウンドの更新
}
