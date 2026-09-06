---
type: Reference
title: StampFly の SPI 結線と PMW3901 の CS ピン
description: IMU(BMI270) と オプティカルフローセンサ(PMW3901) が共有する SPI バスの GPIO 割り当て。CS2 は G12。ただし v1.1 では PMW3901 が実装されているか公式ドキュメント内で情報が矛盾しており、実機確認が必要。
tags: [stampfly, hardware, spi, sensor]
status: draft
generated: { by: claude-opus-5/1m, at: 2026-09-06T02:55:00Z }
verified:
  - { by: claude-opus-5/1m, at: 2026-09-06T02:30:00Z }
  - { by: claude-opus-5/1m, at: 2026-09-06T02:50:00Z }
  - { by: claude-opus-5/1m, at: 2026-09-06T02:55:00Z }
sources:
  - id: docs-stampfly-v11
    resource: https://docs.m5stack.com/ja/app/StampFly_v1.1
    title: StampFly v1.1 製品ドキュメント（m5-docs、ピン表あり）
    author: team:m5stack
  - id: shop-v11
    resource: https://shop.m5stack.com/products/m5stamp-fly-v1-1-with-m5stamps3a
    title: M5Stamp Fly v1.1 商品ページ（M5Stack 公式ストア）
    author: team:m5stack
  - id: docs-stamp-s3a
    resource: https://docs.m5stack.com/en/core/Stamp-S3A
    title: Stamp-S3A モジュール仕様（m5-docs）
    author: team:m5stack
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

**v1.0 と v1.1 で SPI ピン配置は同じ。**（根拠は下記「v1.1 での確認」）

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
[^docs-stampfly-v11]: StampFly v1.1 製品ドキュメント（m5-docs、ピン表あり）
[^docs-stamp-s3a]: Stamp-S3A モジュール仕様（m5-docs）
[^shop-v11]: M5Stamp Fly v1.1 商品ページ（M5Stack 公式ストア）
[^sch-stampfly]: Stamp_Fly v1.0 回路図（M5Stack 公式）
[^sch-pmw3901]: Sch_PMW3901MB_SPI 回路図（M5Stack 公式）

# v1.1 での確認

v1.1 はメインモジュールが **Stamp-S3 から Stamp-S3A に変わっている**ため、
ピン配置が動いた可能性を疑って確認した。**結論として SPI ピンは変わっていない。**

v1.1 の製品ドキュメントに明示的なピン表がある。[^docs-stampfly-v11]

| 部品 | G14 | G44 | G43 | G46 | G12 |
|---|---|---|---|---|---|
| BMI270 | MOSI | SCK | MISO | CS | — |
| PMW3901MB-TXQT | MOSI | SCK | MISO | — | CS2 |

Stamp-S3A の変更点はアンテナ設計・消費電力・ボタンサイズ・RGB LED の電源制御で、
**23 本の GPIO 配置自体は Stamp-S3 と同一**（G0-G15, G39-G44, G46）。[^docs-stamp-s3a]
G12 はこの一覧に含まれる。

LED(G39) とブザー(G40) も v1.1 で変わっていない。[^docs-stampfly-v11]

> **注意**: **v1.1 専用の回路図 PDF は公開されていない。**
> 公開されているのは `Stamp_Fly_v1.0.pdf` のみ。v1.1 の根拠はピン表であって
> 回路図ではない。より確実にしたいなら実機で疎通確認する。

## v1.1 に PMW3901 が載っているかは未確定 (2026-09-06)

**公式ドキュメント内で情報が矛盾している。** 同じ v1.1 のページ内ですら
記載が食い違うため、ドキュメントからは判定できない。

| 情報源（すべて v1.1 公式） | PMW3901 |
|---|---|
| SPI ピンマップ表 [^docs-stampfly-v11] | **記載あり**（G12 = CS2） |
| 回路図リンク [^docs-stampfly-v11] | **記載あり** |
| データシートリンク [^docs-stampfly-v11] | **記載あり** |
| 製品特徴リスト [^docs-stampfly-v11] | **記載なし** |
| 公式ストア商品説明 [^shop-v11] | **記載なし** |

v1.0 の製品ページには PMW3901 がセンサー一覧に明記されていた。[^docs-stampfly]
v1.1 でその記載が消えている。

考えられる解釈は次のどちらかで、**どちらかは実機でしか分からない**:

1. **撤去された** — 特徴リストとストア説明が正しく、ピンマップが v1.0 からの
   コピー漏れ
2. **載っているが宣伝していない** — ピンマップと回路図リンクが正しく、
   特徴リストが「主要センサーのみ」を挙げている

**判定方法**: 実機で G12 を CS として Product ID (0x00) を読む。
`0x49` かつ反転値 `0xB6` が返れば実装されている。`0x00` / `0xFF` が返れば
載っていないか、CS ピンが違う。

なお **M5Stack 公式ファームウェアは PMW3901 を一切使っていない**ため、
「公式が使っていないから載っていない」とは言えない（v1.0 でも使っていなかった）。

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
