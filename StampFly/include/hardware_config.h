#pragma once

#define STAMPFLY (1)

// ----------------------------------------------------------------
#if STAMPFLY

// ブザー関連 ----------------------------------
#define PIN_BEEP 40  // StampFlyのブザー(G40)

// LED関連 -------------------------------------
#define PIN_LED  39  // StampFlyのLED(G39)
#define NUM_LEDS 2   // StampFlyのLEDの数

// モーター関連 ---------------------------------
#define PIN_MOTOR_FL 5   // StampFlyの前左モーター(G5)
#define PIN_MOTOR_FR 42  // StampFlyの前右モーター(G42)
#define PIN_MOTOR_RL 10  // StampFlyの後左モーター(G10)
#define PIN_MOTOR_RR 41  // StampFlyの後右モーター(G41)

// SPI関連 -------------------------------------
// IMU(BMI270)とオプティカルフローセンサ(PMW3901)が同じSPIバスを共有し、
// CSピンだけで区別する
#define PIN_SPI_MOSI 14  // SPIのMOSI(G14)
#define PIN_SPI_MISO 43  // SPIのMISO(G43)
#define PIN_SPI_SCK  44  // SPIのSCK(G44)
#define PIN_IMU_CS   46  // IMU(BMI270)のCS(G46)
#define PIN_FLOW_CS  12  // オプティカルフローセンサ(PMW3901)のCS(G12)

// I2C関連 -------------------------------------
// ToF(VL53L3CX)と気圧センサ・電流センサが共有するバス。
// Wire.begin()を引数なしで呼ぶとESP32-S3の既定ピンになってしまうので、
// driver_i2c.h がこの定義を見て明示的にピンを指定する。
#define PIN_I2C_SDA 3  // I2CのSDA(G3)
#define PIN_I2C_SCL 4  // I2CのSCL(G4)

// ToF(VL53L3CX)関連 ---------------------------
// 前向きと下向きの2個が同じI2Cバスに同じアドレス(0x29)でぶら下がっている。
// XSHUTで片方ずつ黙らせないと区別できない。
#define PIN_TOF_BOTTOM_XSHUT 7  // 下向きToFのXSHUT(G7)
#define PIN_TOF_FRONT_XSHUT  9  // 前向きToFのXSHUT(G9)

#endif
