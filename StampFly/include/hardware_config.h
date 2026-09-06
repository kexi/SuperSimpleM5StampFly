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

#endif
