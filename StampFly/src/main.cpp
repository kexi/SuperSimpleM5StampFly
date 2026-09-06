// はじめの二十二歩
// オプティカルフローセンサ(PMW3901)と通信できるか、製品IDを読んで確認する

#include <Arduino.h>

// ../Common/Driver/include/...
#include "driver_flow.h"   // オプティカルフローセンサドライバ
#include "driver_led.h"    // LEDドライバ
#include "driver_sound.h"  // サウンドドライバ
#include "driver_timer.h"  // Timerドライバ

// センサーと通信できたか
static bool is_flow_ready = false;

void setup() {
    USBSerial.begin(115200);
    delay(1000);  // シリアルモニタの接続待ち

    // 各ドライバーの初期化
    LED_init();         // フルカラーLEDの初期化
    Sound_init();       // サウンドの初期化
    Timer_init(10000);  // タイマーの初期化(10000us = 10ms)

    // オプティカルフローセンサの初期化
    is_flow_ready = Flow_init();

    uint8_t id     = Flow_getProductID();
    uint8_t inv_id = Flow_getInverseProductID();

    USBSerial.printf("PMW3901 Product ID     : 0x%02X (expected 0x49)\n", id);
    USBSerial.printf("PMW3901 Inv Product ID : 0x%02X (expected 0xB6)\n",
                     inv_id);

    if (is_flow_ready) {
        USBSerial.println("PMW3901 OK");
        Sound_play(SOUND_PRESET_BOOT);
    } else {
        USBSerial.println("PMW3901 NG");
    }
}

void loop() {
    Timer_sync();  // フレーム同期

    // 通信できていれば緑、できていなければ赤で点滅させる
    bool is_blink_on = (Timer_getFrameCount() % 100) < 50;
    if (is_flow_ready) {
        LED_setColor(0, 0, is_blink_on ? 100 : 0, 0);
    } else {
        LED_setColor(0, is_blink_on ? 100 : 0, 0, 0);
    }

    LED_update();    // フルカラーLEDの更新
    Timer_update();  // タイマーの更新
    Sound_update();  // サウンドの更新
}
