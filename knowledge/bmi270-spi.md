---
type: Reference
title: BMI270 の SPI 通信仕様
description: IMU(BMI270) のレジスタアクセス手順。読み出しにダミーバイトが1つ挟まる点と、電源投入直後の I2C→SPI モード切替が要点。
tags: [stampfly, sensor, spi]
status: draft
generated: { by: claude-opus-5/1m, at: 2026-09-06T03:45:00Z }
verified:
  - { by: claude-opus-5/1m, at: 2026-09-06T03:45:00Z }
sources:
  - id: official-fw
    resource: https://github.com/m5stack/M5StampFly (lib/bmi270/common.c, src/imu.cpp)
    title: M5StampFly 公式ファームウェア（同一基板での実装）
    author: team:m5stack
---

> **status: draft** — 実機未検証。公式ファームウェアの実装から読み取った仕様。

# Schema

| 項目 | 値 | 根拠 |
|---|---|---|
| モード | SPI_MODE0 (CPOL=0, CPHA=0) | 公式 `.mode = 0` [^official-fw] |
| クロック | 8MHz | 公式 `SPI_MASTER_FREQ_8M` [^official-fw] |
| チップ ID レジスタ | 0x00 | |
| チップ ID 期待値 | **0x24** | |
| 読み出しビット | アドレスに `\| 0x80` | 公式 `reg_addr\|0x80` [^official-fw] |

**PMW3901 とは読み出しビットの向きが逆。** PMW3901 は書き込みで
最上位ビットを立てるが、BMI270 は読み出しで立てる。同じバスでも
手順を共通化できない。

# ダミーバイトが1つ挟まる

**アドレスを送った後、実データの前にダミーが 1 バイト入る。**

```
送信: [reg|0x80] [dummy] [0x00] [0x00] ...
受信:  --------   -----   data0  data1 ...
```

公式実装は `Bmi270.dummy_byte = 1` を設定し、受信バッファを 1 バイト
前へずらしている。[^official-fw]

**PMW3901 にはこの作法が無い。** 読み出し手順を共通化しようとすると壊れる。

# 電源投入直後は I2C モード

BMI270 は電源投入直後 I2C モードで待っており、**CS の立ち下がり/立ち上がりを
一度起こすと SPI モードに切り替わる。**

そのため最初の 1 回の読み出しは捨てる必要がある。これをしないとチップ ID が
読めない。

```
_Imu_readRegister(0x00);   // 捨てる（モード切替のため）
delayMicroseconds(500);
id = _Imu_readRegister(0x00);  // これが 0x24 になる
```

# 未実装 — 設定ブロブ

加速度・角速度を読むには、起動時に **Bosch 配布の設定ファイル（約 8KB の
バイナリ配列）を書き込む**必要がある。手順は次の通り（未検証）:

1. `INIT_CTRL = 0`
2. `INIT_DATA` にブロブをバースト書き込み
3. `INIT_CTRL = 1`
4. `INTERNAL_STATUS == 1` になるまで待つ

ライセンスは BSD-3-Clause。本リポジトリは MIT なので表記を残せば同梱できる。

# 関連

バス共有の注意は [[stampfly-spi-pinout]] を参照。
同じバス上の PMW3901 の仕様は [[pmw3901-driver]]。

[^official-fw]: M5StampFly 公式ファームウェア (lib/bmi270/common.c)
