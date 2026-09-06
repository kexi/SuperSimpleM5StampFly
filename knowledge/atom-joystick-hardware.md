---
type: Reference
title: Atom JoyStick のハードウェア構成とピン配置
description: AtomS3 と STM32F030F4P6 の 2 チップ構成。ジョイスティックとボタンは STM32 側に繋がり、AtomS3 からは I2C(0x59) 越しに読む。
tags: [hardware, toolchain]
status: stable
generated: { by: claude-opus-5/1m, at: 2026-09-06T02:55:00Z }
verified:
  - { by: claude-opus-5/1m, at: 2026-09-06T02:55:00Z }
sources:
  - id: docs-atom-joystick
    resource: https://docs.m5stack.com/ja/app/Atom%20JoyStick
    title: Atom JoyStick 製品ドキュメント（m5-docs）
    author: team:m5stack
  - id: sch-atom-joystick
    resource: https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/products/app/Atom%20JoyStick/Sch_AtomJoystick_v0.3.pdf
    title: Sch_AtomJoystick_v0.3 回路図（M5Stack 公式）
    author: team:m5stack
---

# Schema

**2 チップ構成**であることが最大の特徴。[^docs-atom-joystick]

| 役割 | チップ |
|---|---|
| メインコントローラ（自分のコードが動く） | AtomS3 (ESP32-S3) |
| 入力の収集 | STM32F030F4P6 |

**ジョイスティックとボタンは AtomS3 の GPIO に直結していない。**
STM32 側のピンに繋がっていて、AtomS3 からは I2C 越しに値を読む。

## AtomS3 側の GPIO

| 信号 | GPIO |
|---|---|
| SCL（STM32 へ） | 39 |
| SDA（STM32 へ） | 38 |
| BEEP（ブザー） | 5 |
| WS2812C RGB LED | 6 |

STM32 の I2C アドレスは **0x59**。

## STM32 側のピン（直接は触れない）

| ピン | 機能 |
|---|---|
| PA1 / PA2 / PA3 | 左スティック X / Y / ボタン |
| PA6 / PA5 / PA7 | 右スティック X / Y / ボタン |
| PF0 / PF1 | 左ボタン / 右ボタン |
| PA0 / PA1 | バッテリ ADC1 / ADC2 |

**左スティック Y (PA2) と バッテリ ADC2 (PA1) の記載が重複している**ように
読める。公式ドキュメントの表記揺れの可能性があり、STM32 のファームウェアを
書き換えない限り実害は無いが、鵜呑みにしない。

# その他の仕様

| 項目 | 値 |
|---|---|
| バッテリ | 300mAh@1S、4.35V |
| 充電 IC | TP4067@4.35V |
| 充電時間 (5V/1A) | 約 55 分 |
| ブザー | オンボード passive @5020 |
| 動作温度 | 0〜40℃ |

# 既存コードとの一致

本リポジトリの `JoyStick/include/hardware_config.h` は公式仕様と一致している
（BEEP=5 / LED=6 / I2C アドレス=0x59）。

# StampFly との違いに注意

StampFly はセンサーが SoC に直結しているが、**JoyStick は STM32 を挟む。**
そのため「ピンを叩けば読める」という発想が通用せず、I2C プロトコル越しに
STM32 の用意したレジスタを読むことになる。

[^docs-atom-joystick]: Atom JoyStick 製品ドキュメント（m5-docs）
