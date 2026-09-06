#pragma once

// オプティカルフローセンサ(PMW3901MB)のドライバー
//
// 性能設定レジスタ列は Bitcraze_PMW3901 (MIT License,
// Copyright (c) 2017 Bitcraze AB) から取得した。
// https://github.com/bitcraze/Bitcraze_PMW3901
//
// PMW3901 と IMU(BMI270) は同じSPIバスにぶら下がっていて、CSピンだけで
// 区別する。バスの所有と両CSの初期化は driver_spi.h が持つ。

#include "driver_spi.h"

// レジスタ ------------------------------------
#define FLOW_REG_PRODUCT_ID      0x00  // 製品ID
#define FLOW_REG_INVERSE_PROD_ID 0x5F  // 製品IDの反転値(読めたことの二重確認用)
#define FLOW_REG_POWER_UP_RESET  0x3A  // パワーアップリセット
#define FLOW_REG_MOTION          0x02  // モーション検出フラグ
#define FLOW_REG_DELTA_X_L       0x03  // X移動量の下位バイト
#define FLOW_REG_DELTA_X_H       0x04  // X移動量の上位バイト
#define FLOW_REG_DELTA_Y_L       0x05  // Y移動量の下位バイト
#define FLOW_REG_DELTA_Y_H       0x06  // Y移動量の上位バイト
#define FLOW_REG_SQUAL           0x07  // 検出品質(Surface QUALity)

// モーションフラグ ------------------------------
#define FLOW_MOTION_OCCURRED 0x80  // bit7が立っていれば新しい移動量がある

// 期待値 --------------------------------------
#define FLOW_PRODUCT_ID         0x49  // PMW3901の製品ID
#define FLOW_INVERSE_PRODUCT_ID 0xB6  // 0x49 のビット反転

// SPI設定 -------------------------------------
// PMW3901はモード3(CPOL=1,CPHA=1)、最大2MHz(データシート)。
// 4MHzで動く個体も多いが、まずは確実に読める2MHzから始める。
#define FLOW_SPI_CLOCK 2000000

// Flow_init()が初期値を捨てるために呼ぶので、先に宣言しておく
void Flow_update();

// 最後に読み取った値
static int16_t _flow_delta_x = 0;
static int16_t _flow_delta_y = 0;
static uint8_t _flow_squal   = 0;
static bool    _flow_moved   = false;

// SPIトランザクションの開始(このセンサーを選択する)
static void _Flow_select() {
    SPI_get()->beginTransaction(
        SPISettings(FLOW_SPI_CLOCK, MSBFIRST, SPI_MODE3));
    digitalWrite(PIN_FLOW_CS, LOW);
    delayMicroseconds(50);  // CS確定待ち
}

// SPIトランザクションの終了(選択を解除する)
static void _Flow_deselect() {
    delayMicroseconds(50);  // 転送完了待ち
    digitalWrite(PIN_FLOW_CS, HIGH);
    SPI_get()->endTransaction();
}

// レジスタに1バイト書き込む
static void _Flow_writeRegister(uint8_t reg, uint8_t value) {
    _Flow_select();
    SPI_get()->transfer(reg | 0x80);  // 最上位ビットを立てると書き込み
    SPI_get()->transfer(value);
    _Flow_deselect();
    delayMicroseconds(200);  // 書き込み後の必要待ち時間
}

// レジスタから1バイト読み込む
static uint8_t _Flow_readRegister(uint8_t reg) {
    _Flow_select();
    SPI_get()->transfer(reg & ~0x80);  // 最上位ビットを落とすと読み込み
    delayMicroseconds(35);             // アドレス送信からデータ出力までの待ち
    uint8_t value = SPI_get()->transfer(0x00);
    _Flow_deselect();
    delayMicroseconds(100);
    return value;
}

// 性能設定レジスタ列の書き込み
//
// PMW3901はリセット直後の状態では移動量をまともに検出できず、
// メーカー指定の設定値を書き込んで初めて実用的な性能になる。
// 0x7Fはレジスタページの切り替えで、以降の書き込み先ページを選ぶ。
//
// Why not 各値に意味を書く: データシートに記載の無い非公開レジスタで、
//   PixArt/Bitcraze が実測で決めた値。個々の意味は公開されていないため
//   「なぜこの値か」を書けない。順序と値をそのまま守ることだけが要件。
// 出典: Bitcraze_PMW3901 (MIT License, Copyright (c) 2017 Bitcraze AB)
//       https://github.com/bitcraze/Bitcraze_PMW3901
static void _Flow_writeMagicRegisters() {
    // clang-format off
    static const uint8_t SETTINGS[][2] = {
        {0x7F, 0x00}, {0x61, 0xAD}, {0x7F, 0x03}, {0x40, 0x00},
        {0x7F, 0x05}, {0x41, 0xB3}, {0x43, 0xF1}, {0x45, 0x14},
        {0x5B, 0x32}, {0x5F, 0x34}, {0x7B, 0x08}, {0x7F, 0x06},
        {0x44, 0x1B}, {0x40, 0xBF}, {0x4E, 0x3F}, {0x7F, 0x08},
        {0x65, 0x20}, {0x6A, 0x18}, {0x7F, 0x09}, {0x4F, 0xAF},
        {0x5F, 0x40}, {0x48, 0x80}, {0x49, 0x80}, {0x57, 0x77},
        {0x60, 0x78}, {0x61, 0x78}, {0x62, 0x08}, {0x63, 0x50},
        {0x7F, 0x0A}, {0x45, 0x60}, {0x7F, 0x00}, {0x4D, 0x11},
        {0x55, 0x80}, {0x74, 0x1F}, {0x75, 0x1F}, {0x4A, 0x78},
        {0x4B, 0x78}, {0x44, 0x08}, {0x45, 0x50}, {0x64, 0xFF},
        {0x65, 0x1F}, {0x7F, 0x14}, {0x65, 0x60}, {0x66, 0x08},
        {0x63, 0x78}, {0x7F, 0x15}, {0x48, 0x58}, {0x7F, 0x07},
        {0x41, 0x0D}, {0x43, 0x14}, {0x4B, 0x0E}, {0x45, 0x0F},
        {0x44, 0x42}, {0x4C, 0x80}, {0x7F, 0x10}, {0x5B, 0x02},
        {0x7F, 0x07}, {0x40, 0x41}, {0x70, 0x00},
    };
    // 100ms待ちを挟んだ後に書く分
    static const uint8_t SETTINGS_AFTER_WAIT[][2] = {
        {0x32, 0x44}, {0x7F, 0x07}, {0x40, 0x40}, {0x7F, 0x06},
        {0x62, 0xF0}, {0x63, 0x00}, {0x7F, 0x0D}, {0x48, 0xC0},
        {0x6F, 0xD5}, {0x7F, 0x00}, {0x5B, 0xA0}, {0x4E, 0xA8},
        {0x5A, 0x50}, {0x40, 0x80},
    };
    // clang-format on

    for (size_t i = 0; i < sizeof(SETTINGS) / sizeof(SETTINGS[0]); i++) {
        _Flow_writeRegister(SETTINGS[i][0], SETTINGS[i][1]);
    }

    delay(100);  // メーカー指定の待ち時間

    for (size_t i = 0;
         i < sizeof(SETTINGS_AFTER_WAIT) / sizeof(SETTINGS_AFTER_WAIT[0]);
         i++) {
        _Flow_writeRegister(SETTINGS_AFTER_WAIT[i][0],
                            SETTINGS_AFTER_WAIT[i][1]);
    }
}

// オプティカルフローセンサの初期化
// 戻り値: センサーと通信できたら true
bool Flow_init() {
    // SPIバスの初期化(IMUと共用。両方のCSがHIGHで初期化される)
    SPI_init();

    // パワーアップリセット
    _Flow_writeRegister(FLOW_REG_POWER_UP_RESET, 0x5A);
    delay(5);

    // 製品IDと、その反転値の両方を確認する。
    // 片方だけだと、配線が浮いていて偶然0x49に見えた場合を弾けない。
    uint8_t    id     = _Flow_readRegister(FLOW_REG_PRODUCT_ID);
    uint8_t    inv_id = _Flow_readRegister(FLOW_REG_INVERSE_PROD_ID);
    const bool is_connected =
        (id == FLOW_PRODUCT_ID) && (inv_id == FLOW_INVERSE_PRODUCT_ID);

    // 繋がっていないのに設定を書いても意味がないので、ここで打ち切る
    if (!is_connected) {
        return false;
    }

    _Flow_writeMagicRegisters();

    // 起動直後に溜まっている値を捨てる。
    // 電源投入から初期化までの間の動きが最初の1回に乗ってくるため。
    Flow_update();
    _flow_delta_x = 0;
    _flow_delta_y = 0;
    _flow_moved   = false;

    return true;
}

// 製品IDを読み出す(動作確認用)
uint8_t Flow_getProductID() { return _Flow_readRegister(FLOW_REG_PRODUCT_ID); }

// 製品IDの反転値を読み出す(動作確認用)
uint8_t Flow_getInverseProductID() {
    return _Flow_readRegister(FLOW_REG_INVERSE_PROD_ID);
}

// オプティカルフローセンサの更新
//
// 前回の読み出しからの移動量をセンサー内部で累積しているので、
// 読み出すと同時にクリアされる。呼ぶ間隔がそのまま積分区間になる。
void Flow_update() {
    const uint8_t motion = _Flow_readRegister(FLOW_REG_MOTION);

    _flow_moved = (motion & FLOW_MOTION_OCCURRED) != 0;

    // 動きがなければレジスタの中身は前回値のままなので読まない。
    // 読むと累積がクリアされてしまうため、0を返して呼び出し側に任せる。
    if (!_flow_moved) {
        _flow_delta_x = 0;
        _flow_delta_y = 0;
        _flow_squal   = _Flow_readRegister(FLOW_REG_SQUAL);
        return;
    }

    // 下位・上位の順に読む。センサー内部で16bit値として確定するため、
    // 上位から読むと下位が次のサンプルに変わっている可能性がある。
    const uint8_t x_l = _Flow_readRegister(FLOW_REG_DELTA_X_L);
    const uint8_t x_h = _Flow_readRegister(FLOW_REG_DELTA_X_H);
    const uint8_t y_l = _Flow_readRegister(FLOW_REG_DELTA_Y_L);
    const uint8_t y_h = _Flow_readRegister(FLOW_REG_DELTA_Y_H);

    _flow_delta_x = (int16_t)(((uint16_t)x_h << 8) | x_l);
    _flow_delta_y = (int16_t)(((uint16_t)y_h << 8) | y_l);
    _flow_squal   = _Flow_readRegister(FLOW_REG_SQUAL);
}

// 前回のFlow_update()からのX方向移動量[ピクセル]を取得する
int16_t Flow_getDeltaX() { return _flow_delta_x; }

// 前回のFlow_update()からのY方向移動量[ピクセル]を取得する
int16_t Flow_getDeltaY() { return _flow_delta_y; }

// 検出品質(0-255)を取得する。
// 無地の床や暗所では下がる。速度推定を信じてよいかの判断に使う。
uint8_t Flow_getSqual() { return _flow_squal; }

// 前回のFlow_update()で動きが検出されたかを返す
bool Flow_hasMoved() { return _flow_moved; }
