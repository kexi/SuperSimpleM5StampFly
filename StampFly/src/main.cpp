// はじめの二十三歩
// オプティカルフローセンサ(PMW3901)の移動量を読んでみる
//
// 机の上で機体を滑らせて、どちらに動かすとどちらの符号になるかを確認する。
// ここで確認した軸の向きが、後で速度推定を作るときの前提になる。

#include <Arduino.h>

// ../Common/Driver/include/...
#include "driver_flow.h"   // オプティカルフローセンサドライバ
#include "driver_led.h"    // LEDドライバ
#include "driver_sound.h"  // サウンドドライバ
#include "driver_timer.h"  // Timerドライバ

// センサーと通信できたか
static bool is_flow_ready = false;

// 移動量の累積(どれだけ動いたかを目で追うため)
static int32_t total_x = 0;
static int32_t total_y = 0;

void setup() {
    USBSerial.begin(115200);
    delay(1000);  // シリアルモニタの接続待ち

    // 各ドライバーの初期化
    LED_init();         // フルカラーLEDの初期化
    Sound_init();       // サウンドの初期化
    Timer_init(10000);  // タイマーの初期化(10000us = 10ms)

    // オプティカルフローセンサの初期化
    is_flow_ready = Flow_init();

    if (is_flow_ready) {
        USBSerial.println("PMW3901 OK");
        USBSerial.println("机の上で機体を滑らせてみてください");
        USBSerial.println("dx dy squal total_x total_y");
        Sound_play(SOUND_PRESET_BOOT);
    } else {
        USBSerial.printf("PMW3901 NG (ID:0x%02X INV:0x%02X)\n",
                         Flow_getProductID(), Flow_getInverseProductID());
    }
}

void loop() {
    Timer_sync();  // フレーム同期

    if (is_flow_ready) {
        Flow_update();  // 移動量の読み出し

        const int16_t dx    = Flow_getDeltaX();
        const int16_t dy    = Flow_getDeltaY();
        const uint8_t squal = Flow_getSqual();

        total_x += dx;
        total_y += dy;

        // 動いたときだけ出す。静止中に0が流れ続けると読みにくいため。
        if (Flow_hasMoved()) {
            USBSerial.printf("%5d %5d %4d %8ld %8ld\n", dx, dy, squal, total_x,
                             total_y);
        }

        // 検出品質が低いと速度推定に使えない。
        // 品質が出ていれば緑、低ければ黄色で知らせる。
        const bool is_good_surface = squal >= 32;
        if (is_good_surface) {
            LED_setColor(0, 0, 100, 0);
        } else {
            LED_setColor(0, 100, 60, 0);
        }
    } else {
        // 通信できていなければ赤で点滅させる
        const bool is_blink_on = (Timer_getFrameCount() % 100) < 50;
        LED_setColor(0, is_blink_on ? 100 : 0, 0, 0);
    }

    LED_update();    // フルカラーLEDの更新
    Timer_update();  // タイマーの更新
    Sound_update();  // サウンドの更新
}
