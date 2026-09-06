#pragma once

// オプティカルフローセンサ(PMW3901MB)のドライバー
//
// 初期化手順は PixArt 公式の実装ガイドに従う。条件分岐と個体ごとの
// キャリブレーション値計算を含むため、固定のレジスタ列では代替できない。
// 出典: StampFly Ecosystem (MIT License, Copyright (c) 2026 Kouhei Ito)
//       firmware/vehicle/components/sf_hal_pmw3901/docs/how_to_use_pwm3901.md
//       https://github.com/M5Fly-kanazawa/stampfly_ecosystem
//
// PMW3901 と IMU(BMI270) は同じSPIバスにぶら下がっていて、CSピンだけで
// 区別する。バスの所有と両CSの初期化は driver_spi.h が持つ。

#include "driver_spi.h"  // SF_HOST_TEST 時は中身が無効化される

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
#define FLOW_REG_SHUTTER_UPPER   0x0C  // シャッター上位(データ有効性の判定に使う)

// モーションフラグ ------------------------------
#define FLOW_MOTION_OCCURRED 0x80  // bit7が立っていれば新しい移動量がある

// データ有効性のしきい値 ------------------------
// SQUALがこの値未満、かつShutter_Upperが0x1Fのフレームは信頼できないので捨てる
#define FLOW_SQUAL_MIN             0x19
#define FLOW_SHUTTER_UPPER_INVALID 0x1F

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

// 性能最適化レジスタの設定
//
// PMW3901はリセット直後の状態では移動量をまともに検出できず、この設定を
// 書き込んで初めて実用的な性能になる。値はPixArt社の非公開情報で、
// 個々の意味は公開されていない。順序と値をそのまま守ることだけが要件。
//
// Why not 固定のレジスタ列だけを書く: 検証(0x47)と条件分岐(0x67)、そして
//   個体ごとに違うキャリブレーション値(C1/C2)の計算を含むため、
//   固定列では代替できない。個体差を無視すると性能が出ない。
//
// 戻り値: 検証シーケンスに成功したら true
static bool _Flow_writeConfigRegisters() {
    // 初期設定
    _Flow_writeRegister(0x7F, 0x00);
    _Flow_writeRegister(0x55, 0x01);
    _Flow_writeRegister(0x50, 0x07);
    _Flow_writeRegister(0x7F, 0x0E);

    // 検証シーケンス。0x43に書いた結果が0x47に現れることを確認する。
    // ここが通らないセンサーは以降の設定を書いても正しく動かない。
    bool is_verified = false;
    for (int i = 0; i < 3; i++) {
        _Flow_writeRegister(0x43, 0x10);
        if (_Flow_readRegister(0x47) == 0x08) {
            is_verified = true;
            break;
        }
    }
    if (!is_verified) {
        return false;
    }

    // 個体によって0x67のbit7が違うので、書く値を変える
    const bool is_bit7_set = (_Flow_readRegister(0x67) & 0x80) != 0;
    _Flow_writeRegister(0x48, is_bit7_set ? 0x04 : 0x02);

    _Flow_writeRegister(0x7F, 0x00);
    _Flow_writeRegister(0x51, 0x7B);
    _Flow_writeRegister(0x50, 0x00);
    _Flow_writeRegister(0x55, 0x00);
    _Flow_writeRegister(0x7F, 0x0E);

    // キャリブレーション値(C1/C2)の調整。
    // 0x73が0以外なら調整済みなので触らない。
    if (_Flow_readRegister(0x73) == 0x00) {
        uint8_t c1 = _Flow_readRegister(0x70);
        uint8_t c2 = _Flow_readRegister(0x71);

        c1 = (c1 <= 28) ? (c1 + 14) : (c1 + 11);
        if (c1 > 0x3F) {
            c1 = 0x3F;
        }
        c2 = (uint8_t)((uint16_t)c2 * 45 / 100);

        _Flow_writeRegister(0x7F, 0x00);
        _Flow_writeRegister(0x61, 0xAD);
        _Flow_writeRegister(0x51, c1);
        _Flow_writeRegister(0x7F, 0x0E);
        _Flow_writeRegister(0x70, c1);
        _Flow_writeRegister(0x71, c2);
    }

    // clang-format off
    static const uint8_t SETTINGS[][2] = {
        {0x7F, 0x00}, {0x61, 0xAD}, {0x7F, 0x03}, {0x40, 0x00},
        {0x7F, 0x05}, {0x41, 0xB3}, {0x43, 0xF1}, {0x45, 0x14},
        {0x5B, 0x32}, {0x5F, 0x34}, {0x7B, 0x08}, {0x7F, 0x06},
        {0x44, 0x1B}, {0x40, 0xBF}, {0x4E, 0x3F}, {0x7F, 0x08},
        {0x65, 0x20}, {0x6A, 0x18}, {0x7F, 0x09}, {0x4F, 0xAF},
        {0x5F, 0x40}, {0x48, 0x80}, {0x49, 0x80}, {0x57, 0x77},
        {0x60, 0x78}, {0x61, 0x78}, {0x62, 0x08},
    };
    // 10ms待ちを挟んだ後に書く分
    static const uint8_t SETTINGS_AFTER_WAIT[][2] = {
        {0x32, 0x44}, {0x7F, 0x07}, {0x63, 0x50}, {0x7F, 0x0A},
        {0x45, 0x60}, {0x7F, 0x00}, {0x4D, 0x11}, {0x55, 0x80},
        {0x74, 0x1F}, {0x75, 0x1F}, {0x4A, 0x78}, {0x4B, 0x78},
        {0x44, 0x08}, {0x45, 0x50}, {0x64, 0xFF}, {0x65, 0x1F},
        {0x7F, 0x14}, {0x65, 0x67}, {0x66, 0x08}, {0x63, 0x70},
        {0x7F, 0x15}, {0x48, 0x48}, {0x7F, 0x07}, {0x41, 0x0D},
        {0x43, 0x14}, {0x4B, 0x0E}, {0x45, 0x0F}, {0x44, 0x42},
        {0x4C, 0x80}, {0x7F, 0x10}, {0x5B, 0x02}, {0x7F, 0x07},
        {0x40, 0x41}, {0x70, 0x00}, {0x40, 0x40}, {0x7F, 0x06},
        {0x62, 0xF0}, {0x63, 0x00}, {0x7F, 0x0D}, {0x48, 0xC0},
        {0x6F, 0xD5}, {0x7F, 0x00}, {0x5B, 0xA0}, {0x4E, 0xA8},
        {0x5A, 0x50}, {0x40, 0x80},
    };
    // clang-format on

    for (size_t i = 0; i < sizeof(SETTINGS) / sizeof(SETTINGS[0]); i++) {
        _Flow_writeRegister(SETTINGS[i][0], SETTINGS[i][1]);
    }

    delay(10);  // 公式手順で指定された待ち時間

    for (size_t i = 0;
         i < sizeof(SETTINGS_AFTER_WAIT) / sizeof(SETTINGS_AFTER_WAIT[0]);
         i++) {
        _Flow_writeRegister(SETTINGS_AFTER_WAIT[i][0],
                            SETTINGS_AFTER_WAIT[i][1]);
    }

    return true;
}

// オプティカルフローセンサの初期化
// 戻り値: センサーと通信できたら true
bool Flow_init() {
    // SPIバスの初期化(IMUと共用。両方のCSがHIGHで初期化される)
    SPI_init();

    delay(40);  // 電源が安定するまでの待ち(公式手順の指定は最低40ms)

    // SPIポートのリセット。CSをHIGH->LOW->HIGHと動かす。
    // 電源投入直後のセンサーはSPIの状態が不定なので、ここで揃える。
    digitalWrite(PIN_FLOW_CS, HIGH);
    delayMicroseconds(50);
    digitalWrite(PIN_FLOW_CS, LOW);
    delayMicroseconds(50);
    digitalWrite(PIN_FLOW_CS, HIGH);

    // パワーアップリセット
    _Flow_writeRegister(FLOW_REG_POWER_UP_RESET, 0x5A);
    delay(5);  // リセット完了待ち(公式手順の指定は最低1ms)

    // モーションレジスタを一度読み捨てる。
    // 電源投入から初期化までの動きがここに溜まっているため。
    _Flow_readRegister(FLOW_REG_MOTION);
    _Flow_readRegister(FLOW_REG_DELTA_X_L);
    _Flow_readRegister(FLOW_REG_DELTA_X_H);
    _Flow_readRegister(FLOW_REG_DELTA_Y_L);
    _Flow_readRegister(FLOW_REG_DELTA_Y_H);

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

    // 性能最適化レジスタの設定。検証に失敗したら初期化失敗とする。
    if (!_Flow_writeConfigRegisters()) {
        return false;
    }

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

    // 品質が低く、かつシャッターが開ききっているフレームは信用しない。
    // 無地の床や暗所で、動いていないのに移動量が出ることがある。
    const uint8_t shutter_upper = _Flow_readRegister(FLOW_REG_SHUTTER_UPPER);
    const bool    is_unreliable = (_flow_squal < FLOW_SQUAL_MIN) &&
                               (shutter_upper == FLOW_SHUTTER_UPPER_INVALID);
    if (is_unreliable) {
        _flow_delta_x = 0;
        _flow_delta_y = 0;
        _flow_moved   = false;
    }
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
