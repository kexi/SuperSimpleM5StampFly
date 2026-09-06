---
type: Reference
title: PMW3901 の SPI 通信仕様と疎通確認
description: オプティカルフローセンサ PMW3901 のレジスタアクセス手順、SPI モード・クロック、製品 ID による疎通確認の方法。
tags: [stampfly, sensor, spi]
status: draft
generated: { by: claude-opus-5/1m, at: 2026-09-06T02:30:00Z }
verified:
  - { by: claude-opus-5/1m, at: 2026-09-06T02:30:00Z }
sources:
  - id: bitcraze-pmw3901
    resource: https://github.com/bitcraze/Bitcraze_PMW3901
    title: Bitcraze_PMW3901 ドライバ実装
    author: team:bitcraze
  - id: ds-pmw3901
    resource: https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/products/app/Stamp%20Fly/PMW3901MB-TXQT.PDF
    title: PMW3901MB-TXQT データシート
    author: team:pixart
---

> **status: draft** — 実機での疎通確認はまだ行っていない。
> ビルドが通ることまでが確認済みの範囲。

# Schema

## SPI 設定

| 項目 | 値 |
|---|---|
| モード | SPI_MODE3 (CPOL=1, CPHA=1) |
| ビット順 | MSB first |
| クロック | 2 MHz |

**クロックは 2 MHz を採用した。** Bitcraze 実装は 4 MHz を使うが[^bitcraze-pmw3901]、
データシート上の上限は 2 MHz。[^ds-pmw3901]
4 MHz で動く個体もあるようだが、疎通確認の段階では確実に読める側を選ぶ。

## レジスタアクセス

| レジスタ | 用途 | 期待値 |
|---|---|---|
| 0x00 | 製品 ID | 0x49 |
| 0x5F | 製品 ID の反転値 | 0xB6 |
| 0x3A | パワーアップリセット | 0x5A を書く |

- **書き込み**: アドレスの最上位ビットを立てる（`reg | 0x80`）
- **読み込み**: 最上位ビットを落とす（`reg & ~0x80`）

## タイミング

| 箇所 | 待ち |
|---|---|
| CS 確定待ち | 50 µs |
| 転送完了待ち（CS を戻す前） | 50 µs |
| 書き込み後 | 200 µs |
| 読み込みのアドレス送信後 | 35 µs |
| 読み込み完了後 | 100 µs |
| リセット後 | 5 ms |

# 疎通確認は反転値も見る

製品 ID (0x49) **だけ**を見る実装が多いが、それだと配線が浮いていて
偶然 0x49 に見えるケースを弾けない。

反転値レジスタ 0x5F の 0xB6 と**両方**が一致することを条件にすると、
バスが実際に生きていることまで確認できる。

```
0x49 = 0b01001001
0xB6 = 0b10110110   ← ビット反転
```

# 関連

結線とバス共有の注意は [StampFly の SPI 結線](stampfly-spi-pinout.md) を参照。

[^bitcraze-pmw3901]: Bitcraze_PMW3901 ドライバ実装
[^ds-pmw3901]: PMW3901MB-TXQT データシート
