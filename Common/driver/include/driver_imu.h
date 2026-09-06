#pragma once

// 6軸IMU(BMI270)のドライバー
//
// BMI270 と オプティカルフローセンサ(PMW3901) は同じSPIバスにぶら下がっていて、
// CSピンだけで区別する。バスの所有と両CSの初期化は driver_spi.h が持つ。

#include "driver_spi.h"

// レジスタ ------------------------------------
#define IMU_REG_CHIP_ID 0x00  // チップID

// 期待値 --------------------------------------
#define IMU_CHIP_ID 0x24  // BMI270のチップID

// SPI設定 -------------------------------------
// BMI270はモード0(CPOL=0,CPHA=0)。
// PMW3901(モード3/2MHz)と同じバスを使うので、トランザクションごとに
// 設定を切り替える必要がある。
//
// Why not データシート上限の10MHz: M5Stack公式ファームウェアが同じ基板で
//   8MHzを使っている(lib/bmi270/common.c の SPI_MASTER_FREQ_8M)。
//   実績のある値に合わせる。
#define IMU_SPI_CLOCK 8000000

// SPIトランザクションの開始(このセンサーを選択する)
static void _Imu_select() {
    SPI_get()->beginTransaction(
        SPISettings(IMU_SPI_CLOCK, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_IMU_CS, LOW);
}

// SPIトランザクションの終了(選択を解除する)
static void _Imu_deselect() {
    digitalWrite(PIN_IMU_CS, HIGH);
    SPI_get()->endTransaction();
}

// レジスタから読み込む
//
// BMI270のSPI読み出しは、アドレスを送った後に必ずダミーが1バイト挟まる。
// PMW3901には無い作法なので、同じバスでも読み出し手順を共通化できない。
static void _Imu_readRegisters(uint8_t reg, uint8_t* data, uint8_t len) {
    _Imu_select();
    SPI_get()->transfer(reg | 0x80);  // 最上位ビットを立てると読み込み
    SPI_get()->transfer(0x00);        // ダミーバイト(読み飛ばす)
    for (uint8_t i = 0; i < len; i++) {
        data[i] = SPI_get()->transfer(0x00);
    }
    _Imu_deselect();
}

// レジスタから1バイト読み込む
static uint8_t _Imu_readRegister(uint8_t reg) {
    uint8_t value = 0;
    _Imu_readRegisters(reg, &value, 1);
    return value;
}

// IMUの初期化
// 戻り値: センサーと通信できたら true
bool Imu_init() {
    // SPIバスの初期化(オプティカルフローセンサと共用)
    SPI_init();

    // 電源投入直後のBMI270はI2Cモードで待っている。
    // CSの立ち下がり/立ち上がりを一度起こすとSPIモードに切り替わるので、
    // 捨てるための読み出しを1回入れる。これをしないとIDが読めない。
    _Imu_readRegister(IMU_REG_CHIP_ID);
    delayMicroseconds(500);

    const uint8_t id = _Imu_readRegister(IMU_REG_CHIP_ID);

    return id == IMU_CHIP_ID;
}

// チップIDを読み出す(動作確認用)
uint8_t Imu_getChipID() { return _Imu_readRegister(IMU_REG_CHIP_ID); }

// IMUの更新(今は何もしない)
void Imu_update() {}
