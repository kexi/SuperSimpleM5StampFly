#pragma once

// 6軸IMU(BMI270)のドライバー
//
// BMI270 と オプティカルフローセンサ(PMW3901) は同じSPIバスにぶら下がっていて、
// CSピンだけで区別する。バスの所有と両CSの初期化は driver_spi.h が持つ。

#include "bmi270_config_file.h"
#include "driver_spi.h"

// レジスタ ------------------------------------
#define IMU_REG_CHIP_ID         0x00  // チップID
#define IMU_REG_INTERNAL_STATUS 0x21  // 内部状態(設定書き込みの成否)
#define IMU_REG_ACC_DATA        0x0C  // 加速度データの先頭(X_L)
#define IMU_REG_ACC_CONF        0x40  // 加速度計の設定
#define IMU_REG_ACC_RANGE       0x41  // 加速度計のレンジ
#define IMU_REG_GYR_CONF        0x42  // ジャイロの設定
#define IMU_REG_GYR_RANGE       0x43  // ジャイロのレンジ
#define IMU_REG_INIT_CTRL       0x59  // 設定書き込みの制御
#define IMU_REG_INIT_ADDR_0     0x5B  // 書き込み先アドレス(下位)
#define IMU_REG_INIT_ADDR_1     0x5C  // 書き込み先アドレス(上位)
#define IMU_REG_INIT_DATA       0x5E  // 設定データの書き込み先
#define IMU_REG_PWR_CONF        0x7C  // 電源設定
#define IMU_REG_PWR_CTRL        0x7D  // 電源制御

// 期待値 --------------------------------------
#define IMU_CHIP_ID        0x24  // BMI270のチップID
#define IMU_INIT_STATUS_OK 0x01  // 設定書き込みが成功したときの内部状態

// 設定値 --------------------------------------
#define IMU_INIT_CTRL_PREPARE     0x00  // 設定書き込みの開始
#define IMU_INIT_CTRL_COMPLETE    0x01  // 設定書き込みの完了
#define IMU_PWR_CONF_ADV_SAVE_OFF 0x00  // 省電力を切る(設定書き込みに必要)
#define IMU_PWR_CONF_NORMAL       0x02  // 通常の電源モード
#define IMU_PWR_CTRL_ACC_GYR_EN   0x0E  // 加速度計・ジャイロ・温度を有効にする

// 設定を分割して書き込む単位。一度に全部は送れない。
#define IMU_CONFIG_CHUNK_SIZE 256

// 測定レンジ ----------------------------------
// ±8g / ±2000dps。ドローンは急な姿勢変化で軽く数百dpsに達するため、
// 角速度は広めに取る。
#define IMU_ACCEL_RANGE_G     8.0f
#define IMU_GYRO_RANGE_DPS    2000.0f
#define IMU_ACC_RANGE_SETTING 0x02  // ±8g
#define IMU_GYR_RANGE_SETTING 0x00  // ±2000dps

// 出力データレート。制御ループの400Hzに対して余裕を持たせる。
#define IMU_ACC_CONF_SETTING 0xA8  // 800Hz, ノーマルフィルタ, 性能優先
#define IMU_GYR_CONF_SETTING 0xE9  // 1600Hz, ノーマルフィルタ, 性能優先

// 生値から物理量への変換係数
#define IMU_ACCEL_SCALE (IMU_ACCEL_RANGE_G * 9.80665f / 32768.0f)
#define IMU_GYRO_SCALE  (IMU_GYRO_RANGE_DPS * 0.017453293f / 32768.0f)

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

// 最後に読み取った値(物理単位)
static float _imu_accel[3] = {0.0f, 0.0f, 0.0f};  // m/s^2
static float _imu_gyro[3]  = {0.0f, 0.0f, 0.0f};  // rad/s

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

// レジスタに1バイト書き込む
static void _Imu_writeRegister(uint8_t reg, uint8_t value) {
    _Imu_select();
    SPI_get()->transfer(reg & ~0x80);  // 書き込みは最上位ビットを落とす
    SPI_get()->transfer(value);
    _Imu_deselect();
}

// 連続した領域に書き込む
static void _Imu_writeBurst(uint8_t reg, const uint8_t* data, size_t len) {
    _Imu_select();
    SPI_get()->transfer(reg & ~0x80);
    for (size_t i = 0; i < len; i++) {
        SPI_get()->transfer(data[i]);
    }
    _Imu_deselect();
}

// 設定ファイルをチップに書き込む
//
// BMI270は電源投入だけでは加速度・角速度を出力しない。8192バイトの設定を
// 書き込んで内部状態がOKになって初めて動く。
//
// Why not 一度に全部送る: SPIのトランザクション長に限りがあるため分割する。
//   分割するとチップ側は書き込み位置を知らないので、チャンクごとに
//   INIT_ADDRで位置を教える必要がある。アドレスはワード単位(2バイト)。
//
// 戻り値: 内部状態がOKになったら true
static bool _Imu_uploadConfigFile() {
    // 省電力が効いていると設定を受け付けない
    _Imu_writeRegister(IMU_REG_PWR_CONF, IMU_PWR_CONF_ADV_SAVE_OFF);
    delayMicroseconds(450);

    // 書き込み開始を宣言する
    _Imu_writeRegister(IMU_REG_INIT_CTRL, IMU_INIT_CTRL_PREPARE);

    const size_t total = sizeof(BMI270_CONFIG_FILE);
    for (size_t written = 0; written < total;) {
        size_t chunk = total - written;
        if (chunk > IMU_CONFIG_CHUNK_SIZE) {
            chunk = IMU_CONFIG_CHUNK_SIZE;
        }

        // 書き込み位置をワード単位で伝える
        const size_t  word_index = written / 2;
        const uint8_t addr[2]    = {
            (uint8_t)(word_index & 0x0F),  // 下位は4bitだけ
            (uint8_t)(word_index >> 4),
        };
        _Imu_writeBurst(IMU_REG_INIT_ADDR_0, addr, 2);
        _Imu_writeBurst(IMU_REG_INIT_DATA, &BMI270_CONFIG_FILE[written], chunk);

        written += chunk;
    }

    // 書き込み完了を宣言する
    _Imu_writeRegister(IMU_REG_INIT_CTRL, IMU_INIT_CTRL_COMPLETE);

    // チップ内部で設定が展開されるのを待つ。
    // データシート上は20ms程度だが、余裕を見て確認しながら待つ。
    for (int i = 0; i < 50; i++) {
        delay(1);
        const uint8_t status = _Imu_readRegister(IMU_REG_INTERNAL_STATUS);
        if ((status & 0x0F) == IMU_INIT_STATUS_OK) {
            return true;
        }
    }
    return false;
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
    if (id != IMU_CHIP_ID) {
        return false;
    }

    // 設定ファイルを書き込まないと、加速度・角速度は出てこない
    if (!_Imu_uploadConfigFile()) {
        return false;
    }

    // 測定レンジと出力レートを決める
    _Imu_writeRegister(IMU_REG_ACC_CONF, IMU_ACC_CONF_SETTING);
    _Imu_writeRegister(IMU_REG_ACC_RANGE, IMU_ACC_RANGE_SETTING);
    _Imu_writeRegister(IMU_REG_GYR_CONF, IMU_GYR_CONF_SETTING);
    _Imu_writeRegister(IMU_REG_GYR_RANGE, IMU_GYR_RANGE_SETTING);

    // 加速度計とジャイロを動かす
    _Imu_writeRegister(IMU_REG_PWR_CTRL, IMU_PWR_CTRL_ACC_GYR_EN);
    delay(10);
    _Imu_writeRegister(IMU_REG_PWR_CONF, IMU_PWR_CONF_NORMAL);
    delay(10);

    return true;
}

// チップIDを読み出す(動作確認用)
uint8_t Imu_getChipID() { return _Imu_readRegister(IMU_REG_CHIP_ID); }

// IMUの更新
//
// 加速度3軸とジャイロ3軸は連続したレジスタに並んでいるので、
// 12バイトを一度に読む。個別に読むとSPIのオーバーヘッドが6倍になる。
void Imu_update() {
    uint8_t buf[12];
    _Imu_readRegisters(IMU_REG_ACC_DATA, buf, sizeof(buf));

    // リトルエンディアン(下位バイトが先)の16bit符号付き
    for (int i = 0; i < 3; i++) {
        const int16_t accel_raw =
            (int16_t)(((uint16_t)buf[i * 2 + 1] << 8) | buf[i * 2]);
        const int16_t gyro_raw =
            (int16_t)(((uint16_t)buf[6 + i * 2 + 1] << 8) | buf[6 + i * 2]);

        _imu_accel[i] = accel_raw * IMU_ACCEL_SCALE;
        _imu_gyro[i]  = gyro_raw * IMU_GYRO_SCALE;
    }
}

// 加速度を取得する[m/s^2]。out には3要素分の領域が要る。
void Imu_getAccel(float* out) {
    out[0] = _imu_accel[0];
    out[1] = _imu_accel[1];
    out[2] = _imu_accel[2];
}

// 角速度を取得する[rad/s]。out には3要素分の領域が要る。
void Imu_getGyro(float* out) {
    out[0] = _imu_gyro[0];
    out[1] = _imu_gyro[1];
    out[2] = _imu_gyro[2];
}
