#pragma once

// I2C を使う場合は Wire ライブラリを使う
#include <Wire.h>

// I2Cのクロック。VL53L3CXは400kHzまで対応する。
// AtomJoyStickも400kHzで動くので、ボードによらず同じ値を使う。
#define I2C_CLOCK_HZ 400000

// I2Cの初期化
//
// ピンを明示するかどうかをボードごとに変える。
// StampFlyはSDA=G3/SCL=G4で、ESP32-S3の既定ピン(G8/G9)とは違うため、
// 引数なしのWire.begin()では通信できない。
// 一方AtomJoyStickは既定ピンで動いていて、ここでピンを決め打ちすると壊れる。
//
// Why not 常にピンを渡す: JoyStick側のhardware_config.hにはI2Cピンの定義が
//   無く、値を推測して書くと動いているものを壊すリスクだけが増える。
//   定義があるボードだけ明示する形にして、既存の挙動を保つ。
void I2C_init() {
#if defined(PIN_I2C_SDA) && defined(PIN_I2C_SCL)
    // ピンが定義されているボード(StampFly)は明示的に指定する
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
#else
    // 定義が無いボード(AtomJoyStick)は従来通り既定ピンに任せる
    Wire.begin();
#endif
    Wire.setClock(I2C_CLOCK_HZ);
}

// I2Cの更新(何もしない)
void I2C_update() {}

// I2Cでデータを読み込む
void I2C_read(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len) {
    // I2Cでデータを読み込む
    Wire.beginTransmission(addr);
    // レジスタを指定
    Wire.write(reg);
    Wire.endTransmission(false);

    // データを読み込む
    Wire.requestFrom(addr, len);
    for (int i = 0; i < len; i++) {
        data[i] = Wire.read();
    }
    Wire.endTransmission();
}

// I2Cで16bitレジスタアドレスを指定してデータを読み込む
//
// VL53L3CXのレジスタアドレスは16bitで、8bitのI2C_read()では届かない。
// 戻り値: 通信に成功したら true
//
// Why not I2C_read()を16bit対応に拡張する: 既存の呼び出し元(driver_joy.h /
//   driver_battery.h)が全て8bitで、引数を増やすと全箇所の変更になる。
//   使う側が限られるうちは別関数で足す方が影響範囲が小さい。
bool I2C_read16(uint8_t addr, uint16_t reg, uint8_t* data, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write((uint8_t)(reg >> 8));    // レジスタアドレス上位
    Wire.write((uint8_t)(reg & 0xFF));  // レジスタアドレス下位
    // レジスタ指定と読み出しの間でバスを離さない(リピーテッドスタート)。
    // 離すと他のマスタに割り込まれてアドレスが失われる。
    const bool is_addr_sent = (Wire.endTransmission(false) == 0);
    if (!is_addr_sent) {
        return false;
    }

    const uint8_t received = Wire.requestFrom(addr, len);
    if (received != len) {
        return false;
    }
    for (uint8_t i = 0; i < len; i++) {
        data[i] = Wire.read();
    }
    return true;
}

// I2Cで16bitレジスタアドレスを指定してデータを書き込む
// 戻り値: 通信に成功したら true
bool I2C_write16(uint8_t addr, uint16_t reg, const uint8_t* data, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write((uint8_t)(reg >> 8));
    Wire.write((uint8_t)(reg & 0xFF));
    for (uint8_t i = 0; i < len; i++) {
        Wire.write(data[i]);
    }
    return Wire.endTransmission() == 0;
}
