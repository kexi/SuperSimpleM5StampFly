#pragma once

// オプティカルフローセンサ(PMW3901MB)のドライバー
//
// StampFlyでは PMW3901 と IMU(BMI270) が同じSPIバスにぶら下がっていて、
// CSピンだけで区別する。そのため通信前に IMU 側のCSをHIGH(非選択)に
// 固定しておかないと、両方が同時に応答してデータが壊れる。

#include <SPI.h>

#include "hardware_config.h"

// レジスタ ------------------------------------
#define FLOW_REG_PRODUCT_ID      0x00  // 製品ID
#define FLOW_REG_INVERSE_PROD_ID 0x5F  // 製品IDの反転値(読めたことの二重確認用)
#define FLOW_REG_POWER_UP_RESET  0x3A  // パワーアップリセット

// 期待値 --------------------------------------
#define FLOW_PRODUCT_ID         0x49  // PMW3901の製品ID
#define FLOW_INVERSE_PRODUCT_ID 0xB6  // 0x49 のビット反転

// SPI設定 -------------------------------------
// PMW3901はモード3(CPOL=1,CPHA=1)、最大2MHz(データシート)。
// 4MHzで動く個体も多いが、まずは確実に読める2MHzから始める。
#define FLOW_SPI_CLOCK 2000000

static SPIClass* _flow_spi = NULL;

// SPIトランザクションの開始(このセンサーを選択する)
static void _Flow_select() {
    _flow_spi->beginTransaction(
        SPISettings(FLOW_SPI_CLOCK, MSBFIRST, SPI_MODE3));
    digitalWrite(PIN_FLOW_CS, LOW);
    delayMicroseconds(50);  // CS確定待ち
}

// SPIトランザクションの終了(選択を解除する)
static void _Flow_deselect() {
    delayMicroseconds(50);  // 転送完了待ち
    digitalWrite(PIN_FLOW_CS, HIGH);
    _flow_spi->endTransaction();
}

// レジスタに1バイト書き込む
static void _Flow_writeRegister(uint8_t reg, uint8_t value) {
    _Flow_select();
    _flow_spi->transfer(reg | 0x80);  // 最上位ビットを立てると書き込み
    _flow_spi->transfer(value);
    _Flow_deselect();
    delayMicroseconds(200);  // 書き込み後の必要待ち時間
}

// レジスタから1バイト読み込む
static uint8_t _Flow_readRegister(uint8_t reg) {
    _Flow_select();
    _flow_spi->transfer(reg & ~0x80);  // 最上位ビットを落とすと読み込み
    delayMicroseconds(35);             // アドレス送信からデータ出力までの待ち
    uint8_t value = _flow_spi->transfer(0x00);
    _Flow_deselect();
    delayMicroseconds(100);
    return value;
}

// オプティカルフローセンサの初期化
// 戻り値: センサーと通信できたら true
bool Flow_init() {
    // SPIバスの初期化(HSPIをIMUと共用する)
    _flow_spi = new SPIClass(HSPI);
    _flow_spi->begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_FLOW_CS);

    // IMUのCSをHIGHに固定して、SPIバス上で黙らせておく
    pinMode(PIN_IMU_CS, OUTPUT);
    digitalWrite(PIN_IMU_CS, HIGH);

    // フローセンサのCSは非選択(HIGH)から始める
    pinMode(PIN_FLOW_CS, OUTPUT);
    digitalWrite(PIN_FLOW_CS, HIGH);

    delay(50);  // 電源安定待ち

    // パワーアップリセット
    _Flow_writeRegister(FLOW_REG_POWER_UP_RESET, 0x5A);
    delay(5);

    // 製品IDと、その反転値の両方を確認する。
    // 片方だけだと、配線が浮いていて偶然0x49に見えた場合を弾けない。
    uint8_t id     = _Flow_readRegister(FLOW_REG_PRODUCT_ID);
    uint8_t inv_id = _Flow_readRegister(FLOW_REG_INVERSE_PROD_ID);

    return (id == FLOW_PRODUCT_ID) && (inv_id == FLOW_INVERSE_PRODUCT_ID);
}

// 製品IDを読み出す(動作確認用)
uint8_t Flow_getProductID() { return _Flow_readRegister(FLOW_REG_PRODUCT_ID); }

// 製品IDの反転値を読み出す(動作確認用)
uint8_t Flow_getInverseProductID() {
    return _Flow_readRegister(FLOW_REG_INVERSE_PROD_ID);
}

// オプティカルフローセンサの更新(今は何もしない)
void Flow_update() {}
