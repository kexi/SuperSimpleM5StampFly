---
type: Reference
title: StampFly の SPI 結線と PMW3901 の CS ピン
description: IMU(BMI270) と オプティカルフローセンサ(PMW3901) が共有する SPI バスの GPIO 割り当て。CS2 は G12（Web 上の記載は G46 と食い違うが回路図で確定）。
tags: [stampfly, hardware, spi, sensor]
status: stable
generated: { by: claude-opus-5/1m, at: 2026-09-06T02:30:00Z }
verified:
  - { by: claude-opus-5/1m, at: 2026-09-06T02:30:00Z }
sources:
  - id: sch-stampfly
    resource: https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1069/Stamp_Fly_v1.0.pdf
    title: Stamp_Fly v1.0 回路図（M5Stack 公式）
    author: team:m5stack
  - id: sch-pmw3901
    resource: https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1069/Sch_PMW3901MB_SPI.pdf
    title: Sch_PMW3901MB_SPI 回路図（M5Stack 公式）
    author: team:m5stack
  - id: docs-stampfly
    resource: https://docs.m5stack.com/en/app/Stamp%20Fly
    title: StampFly 製品ドキュメント（m5-docs）
    author: team:m5stack
---

# Schema

IMU(BMI270) と オプティカルフローセンサ(PMW3901MB-TXQT) は**同じ SPI バスを共有**し、
CS ピンだけで区別する。

| 信号 | GPIO |
|---|---|
| MOSI | 14 |
| MISO | 43 |
| SCK | 44 |
| CS (BMI270) | 46 |
| CS2 (PMW3901) | **12** |

I2C は SDA=3 / SCL=4。

# CS2 が G12 である根拠

**Web ページの記載だけでは確定できなかった。** m5-docs の 2 ページを読むと
CS2 が G46 とも G12 とも読める記述になっていて食い違う。[^docs-stampfly]

回路図 PDF を実際に開いて確定させた。StampS3 モジュールのヘッダ上で
`CS2` ネットが `G12` に並んでおり、`SDA` が `G13` に並ぶ。[^sch-stampfly]
PMW3901 側の基板でも CS は `CS2` という独立したネット名で、
IMU の `CS` とは別線であることが確認できる。[^sch-pmw3901]

[^docs-stampfly]: StampFly 製品ドキュメント（m5-docs）
[^sch-stampfly]: Stamp_Fly v1.0 回路図（M5Stack 公式）
[^sch-pmw3901]: Sch_PMW3901MB_SPI 回路図（M5Stack 公式）

# バス共有時の注意

CS だけで区別する構成なので、**片方と通信する前にもう片方の CS を HIGH（非選択）に
固定する**必要がある。しないと両方が応答してデータが壊れる。

初期化順に依存しないよう、フロー側のドライバは自分の init の中で
IMU の CS を明示的に HIGH にしている。

# 効かなかった調査 — オリジナルファームウェアには結線情報が無い

`m5stack/M5StampFly`（公式ファームウェア）を clone して
`3901` / `optical` / `flow` を grep すれば CS ピンが分かると考えたが、**外れた**。

- 期待: 公式ファームウェアが PMW3901 を使っているはずで、そこにピン定義がある
- 実際: **PMW3901 に触れるコードが一行も無い。** ヒットするのは
  VL53L3CX ライブラリ内の無関係な語のみ。SPI で読んでいるのは BMI270 だけ
- なぜ外れたか: M5Stack 公式ファームウェア自体が Optical Flow 未対応だった。
  `src/sensor.hpp` の extern 変数も IMU・ToF・気圧・電圧系のみでフロー関連が無い

**結線情報の一次資料は回路図 PDF であって、ファームウェアではない。**
ただし公式ファームウェアの `src/sensor.hpp` には SPI バス側のピン
（MOSI/MISO/SCK と IMU の CS）は定義されており、そこは回路図と一致した。
