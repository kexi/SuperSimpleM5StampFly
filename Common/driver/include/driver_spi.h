#pragma once

// SPIバスのドライバー
//
// StampFlyでは IMU(BMI270) と オプティカルフローセンサ(PMW3901) が
// 同じSPIバスにぶら下がっていて、CSピンだけで区別する。
// バスを1箇所で所有し、両方のCSをHIGH(非選択)で初期化することで、
// どちらのデバイスを先に初期化しても他方が応答しない状態を保証する。
//
// Why not 各ドライバがSPIClassを持つ: 同じバスに対してbegin()が
//   二重に呼ばれる。どちらが先に初期化されるかで挙動が変わるのを避ける。

// SILS のテストではホスト側でスタブを用意するため、実体を読み込まない
#ifndef SF_HOST_TEST

#include <SPI.h>

#include "hardware_config.h"

static SPIClass* _spi          = NULL;
static bool      _spi_is_ready = false;

// SPIバスの初期化
void SPI_init() {
    // 二重初期化を防ぐ。デバイス側のドライバがそれぞれ呼んでも良いようにする。
    if (_spi_is_ready) {
        return;
    }

    // 各デバイスのCSを非選択(HIGH)にしてから、バスを立ち上げる。
    // 先にバスを動かすと、CSが不定のデバイスが勝手に応答しうる。
    pinMode(PIN_IMU_CS, OUTPUT);
    digitalWrite(PIN_IMU_CS, HIGH);
    pinMode(PIN_FLOW_CS, OUTPUT);
    digitalWrite(PIN_FLOW_CS, HIGH);

    _spi = new SPIClass(HSPI);
    // 第4引数のCSは使わない(各ドライバが自分のCSを叩く)ため-1を渡す
    _spi->begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);

    _spi_is_ready = true;

    delay(50);  // 電源とバスの安定待ち
}

// SPIバスを取得する
SPIClass* SPI_get() { return _spi; }

// SPIバスの更新(何もしない)
void SPI_update() {}

#endif  // SF_HOST_TEST
