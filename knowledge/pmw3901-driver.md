---
type: Reference
title: PMW3901 の SPI 通信仕様と疎通確認
description: オプティカルフローセンサ PMW3901 のレジスタアクセス手順、初期化シーケンス(条件分岐とキャリブレーション計算を含む)、移動量の読み出しとデータ有効性フィルタ。
tags: [stampfly, sensor, spi]
status: stable
generated: { by: claude-opus-5/1m, at: 2026-09-06T05:30:00Z }
verified:
  - { by: claude-opus-5/1m, at: 2026-09-06T02:30:00Z }
  - { by: claude-opus-5/1m, at: 2026-09-06T03:25:00Z }
  - { by: claude-opus-5/1m, at: 2026-09-06T05:30:00Z }
sources:
  - id: pixart-guide
    resource: https://github.com/M5Fly-kanazawa/stampfly_ecosystem (firmware/vehicle/components/sf_hal_pmw3901/docs/how_to_use_pwm3901.md)
    title: PMW3901MB 実装ガイド（PixArt 公式手順）
    author: human:kouhei-ito
  - id: bitcraze-pmw3901
    resource: https://github.com/bitcraze/Bitcraze_PMW3901
    title: Bitcraze_PMW3901 ドライバ実装
    author: team:bitcraze
  - id: ds-pmw3901
    resource: https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/products/app/Stamp%20Fly/PMW3901MB-TXQT.PDF
    title: PMW3901MB-TXQT データシート
    author: team:pixart
---

> **status: stable** — 実機での疎通確認はまだ行っていない。
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

# 移動量の読み出し

| レジスタ | 内容 |
|---|---|
| 0x02 | Motion。bit7 が立っていれば新しい移動量がある |
| 0x03 / 0x04 | Delta X の下位 / 上位バイト |
| 0x05 / 0x06 | Delta Y の下位 / 上位バイト |
| 0x07 | SQUAL（検出品質 0-255） |

**読み出すと累積がクリアされる。** センサーは前回の読み出しからの移動量を
内部で累積しているので、`Flow_update()` を呼ぶ間隔がそのまま積分区間になる。

**下位バイトから先に読む。** 上位から読むと、その間に下位が次のサンプルに
変わっている可能性がある。

Motion の bit7 が立っていないときは Delta レジスタを読まない。
読むと累積がクリアされてしまうため。

# 初期化シーケンス（固定レジスタ列では不十分）

**リセット直後の状態では移動量をまともに検出できない。**
性能最適化レジスタの設定が必須で、これは**単なる固定列ではない。**

## 電源投入手順

1. 電源安定待ち **40ms 以上**
2. **NCS を HIGH→LOW→HIGH** して SPI ポートをリセット
3. `0x3A` に `0x5A` を書く（パワーアップリセット）
4. **1ms 以上**待つ
5. `0x02`〜`0x06` を一度ずつ読み捨てる（動きの有無に関わらず）

## 性能最適化（条件分岐を含む）

| 段階 | 内容 |
|---|---|
| 初期設定 | `0x7F=0x00, 0x55=0x01, 0x50=0x07, 0x7F=0x0E` |
| **検証** | `0x43=0x10` を書き `0x47` が `0x08` を返すか確認。**最大3回リトライ**、失敗ならエラー |
| **条件分岐** | `0x67` の bit7 が立っていれば `0x48=0x04`、でなければ `0x48=0x02` |
| **キャリブレーション** | `0x73` が 0 のときだけ C1/C2 を計算して書く |
| 固定列 | 約 70 個。途中に **10ms の待ち** |

C1/C2 の計算:

```
C1 = read(0x70) ; C2 = read(0x71)
new_C1 = (C1 <= 28) ? C1 + 14 : C1 + 11   （上限 0x3F）
new_C2 = C2 * 45 / 100
```

**個体ごとに違う値**なので、固定列で代替すると性能が出ない。

出典: PixArt 公式実装ガイド（`how_to_use_pwm3901.md`）。
[[stampfly-ecosystem-reference]] 経由で入手した。

> **落とし穴**: この設定を書かずに Product ID だけ読めても「動いている」とは
> 言えない。**ID の読み出しは設定前でも成功する。**

## Bitcraze 版との違い

[Bitcraze_PMW3901](https://github.com/bitcraze/Bitcraze_PMW3901) は
**約 70 個の固定列のみ**で、検証・条件分岐・キャリブレーションが無い。
本リポジトリは当初こちらを採用していたが、公式手順に切り替えた。

固定列の中身も微妙に違う（`0x65`: 0x60 vs 0x67、`0x63`: 0x78 vs 0x70、
`0x48`: 0x58 vs 0x48 など）。

# データ有効性のフィルタ

**SQUAL < 0x19 かつ Shutter_Upper (0x0C) == 0x1F** のフレームは破棄する。
公式ガイドが「極めて重要」として挙げているベストプラクティス。

無地の床や暗所で、動いていないのに移動量が出ることがある。

# 関連

結線とバス共有の注意は [StampFly の SPI 結線](stampfly-spi-pinout.md) を参照。

[^bitcraze-pmw3901]: Bitcraze_PMW3901 ドライバ実装
[^ds-pmw3901]: PMW3901MB-TXQT データシート
