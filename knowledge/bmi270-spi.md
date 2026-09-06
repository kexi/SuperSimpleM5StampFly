---
type: Reference
title: BMI270 の SPI 通信仕様
description: IMU(BMI270) のレジスタアクセス手順。読み出しにダミーバイトが1つ挟まる点と、電源投入直後の I2C→SPI モード切替が要点。
tags: [stampfly, sensor, spi]
status: draft
generated: { by: claude-opus-5/1m, at: 2026-09-06T06:00:00Z }
verified:
  - { by: claude-opus-5/1m, at: 2026-09-06T03:45:00Z }
  - { by: claude-opus-5/1m, at: 2026-09-06T06:00:00Z }
sources:
  - id: ecosystem-bmi270
    resource: https://github.com/M5Fly-kanazawa/stampfly_ecosystem (firmware/vehicle/components/sf_hal_bmi270/)
    title: BMI270 ドライバと設定ファイル (MIT License, Copyright (c) 2025 Kouhei Ito)
    author: human:kouhei-ito
  - id: official-fw
    resource: https://github.com/m5stack/M5StampFly (lib/bmi270/common.c, src/imu.cpp)
    title: M5StampFly 公式ファームウェア（同一基板での実装）
    author: team:m5stack
---

> **status: draft** — 実機未検証。設定ファイルのバイト列一致とビルドは確認済みだが、
> **チップが実際に応答するかは未確認。** SILS には BMI270 の SPI レベルモデルが
> 無い（ラッパー層の差し替えのみ）ため、PMW3901 と違ってホスト検証もできない。

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

# 設定ファイル（8192 バイト）の書き込み

**電源投入だけでは加速度・角速度を出力しない。** 8192 バイトの設定を
書き込み、内部状態が OK になって初めて動く。

## 手順

| 順 | 内容 |
|---|---|
| 1 | `PWR_CONF(0x7C) = 0x00` で省電力を切る → **450µs 待つ** |
| 2 | `INIT_CTRL(0x59) = 0x00` で書き込み開始を宣言 |
| 3 | **256 バイトずつ**分割して書く（下記） |
| 4 | `INIT_CTRL(0x59) = 0x01` で完了を宣言 |
| 5 | `INTERNAL_STATUS(0x21) & 0x0F == 0x01` になるまで待つ（20ms 程度） |

## 分割書き込みの要点

**チップ側は書き込み位置を知らないので、チャンクごとに `INIT_ADDR` で
位置を教える必要がある。** そしてこのアドレスは**ワード単位（2 バイト）**。

```
word_index = written / 2
INIT_ADDR_0 (0x5B) = word_index & 0x0F    ← 下位は 4bit だけ
INIT_ADDR_1 (0x5C) = word_index >> 4
INIT_DATA   (0x5E) にチャンク本体をバースト書き込み
```

`INIT_ADDR_0` が 4bit しか使わない点に注意。ここをバイト単位や 8bit と
誤ると、設定が壊れた位置に書かれて `INTERNAL_STATUS` が OK にならない。

## 設定ファイルの入手

Bosch Sensortec の公式配布物。**意味のある単位に分解できないバイナリ**で、
チップのファームウェアそのもの。バイト列を変えると動作しない。

本リポジトリは [[stampfly-ecosystem-reference]] 経由で取得した
（MIT License, Copyright (c) 2025 Kouhei Ito）。[^ecosystem-bmi270]
コピー後に**バイト列が完全一致することを検証済み**（8192 バイト）。

# 設定値

| レジスタ | 値 | 意味 |
|---|---|---|
| `ACC_CONF(0x40)` | 0xA8 | 800Hz、ノーマルフィルタ、性能優先 |
| `ACC_RANGE(0x41)` | 0x02 | ±8g |
| `GYR_CONF(0x42)` | 0xE9 | 1600Hz、ノーマルフィルタ、性能優先 |
| `GYR_RANGE(0x43)` | 0x00 | ±2000dps |
| `PWR_CTRL(0x7D)` | 0x0E | 加速度計・ジャイロ・温度を有効化 |
| `PWR_CONF(0x7C)` | 0x02 | 通常の電源モード |

ジャイロを ±2000dps と広く取るのは、ドローンが急な姿勢変化で軽く
数百 dps に達するため。飽和させると姿勢推定が崩れる。

# データの読み出し

加速度 3 軸とジャイロ 3 軸は `0x0C` から連続して並ぶ。
**12 バイトを一度に読む**（個別に読むと SPI のオーバーヘッドが 6 倍）。

リトルエンディアン（下位バイトが先）の 16bit 符号付き。

```
accel[m/s^2] = raw * (8.0 * 9.80665 / 32768)
gyro[rad/s]  = raw * (2000 * π/180 / 32768)
```

# 関連

バス共有の注意は [[stampfly-spi-pinout]] を参照。
同じバス上の PMW3901 の仕様は [[pmw3901-driver]]。

[^official-fw]: M5StampFly 公式ファームウェア (lib/bmi270/common.c)
[^ecosystem-bmi270]: BMI270 ドライバと設定ファイル (StampFly Ecosystem)
