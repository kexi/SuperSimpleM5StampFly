---
type: Playbook
title: SILS エミュレータ — 実機なしで PMW3901 を検証できる
description: StampFly Ecosystem の SILS は実ファームウェアを無改変で PC 上で走らせる。PMW3901 のチップモデルがあり、Product ID・回転成分除去・最小高度ゲートまで実機なしで検証できる。
tags: [stampfly, sensor, control, reference, toolchain]
status: stable
generated: { by: claude-opus-5/1m, at: 2026-09-06T05:00:00Z }
verified:
  - { by: claude-opus-5/1m, at: 2026-09-06T05:00:00Z }
sources:
  - id: sils
    resource: https://github.com/M5Fly-kanazawa/stampfly_ecosystem (simulator/sils/)
    title: StampFly SILS (MIT License, Copyright (c) 2026 Kouhei Ito)
    author: human:kouhei-ito
  - id: probe-run
    resource: 本マシンでの pmw3901_probe 実行（2026-09-06、macOS 26 / aarch64-darwin、MuJoCo 3.9.0）
    title: エミュレータ実行の実測
---

# 結論 — エミュレータは存在し、実際に動く

**StampFly のエミュレータは存在する。** `simulator/sils/` の SILS
(Software-in-the-Loop) が、**実ファームウェアを無改変でコンパイルして
PC 上で走らせる。**[^sils]

- ESP-IDF / FreeRTOS のホスト用スタブ（`compat/`）
- 決定論的な協調 RTOS エミュレータ（`rtos/`）
- MuJoCo による物理（`plant/`）
- **センサーのチップモデル**（`devices/pmw3901_device.cpp`、`vl53_device.cpp`）

> **訂正**: 本作業の初期に「StampFly 専用のエミュレータは存在しない」
> 「CS ピンや Product ID はエミュレータでは検証できない」と結論したが、
> **どちらも誤りだった。** 下記「何を見誤ったか」を参照。

# 実行して確認できたこと

`./build/pmw3901_probe` の実行結果（0 failures）:[^probe-run]

```
(1) SPI register protocol
  [PASS] product ID == 0x49
  [PASS] inverse product ID == 0xB6
  [PASS] init-verify 0x47 == 0x08
  [PASS] config write/read-back
(2) flow round-trip (translational + rotational)
  vx=0.50 vy=0.00 h=0.60 gx=0.00 gy=0.00 → dx=0  dy=-4   ← 並進
  vx=0.00 vy=0.00 h=0.60 gx=0.40 gy=0.00 → dx=-2 dy=0    ← 回転のみで流れる
(3) below min height → no flow
  [PASS] no motion below min height
```

**「動いていないのに傾けると流れる」が再現できている。**
[[position-hold-design]] で「回転成分を除去しないと発振する」と書いた現象を、
実機なしで観察・検証できる。

`emu_vehicle` も完走した（ARM → モーター起動 → シナリオ実行 → フェイルセーフ検出）。

# 手順（macOS / Nix 環境）

```console
$ cd simulator/sils
$ nix shell nixpkgs#cmake nixpkgs#ninja
$ cmake -S . -B build -G Ninja      # 初回は MuJoCo 3.9.0 を取得（数分）
$ cmake --build build
$ ./build/pmw3901_probe             # PMW3901 のプローブ
```

**落とし穴 1**: `-DSILS_BUILD_MUJOCO_SMOKE=OFF` は使えない。
このフラグは MuJoCo の取得ごと止めるが、`emu_vehicle` 本体も MuJoCo を要求する
ため `fatal error: 'mujoco/mujoco.h' file not found` で落ちる。
一度 OFF で configure すると build ディレクトリごと消して再構成が要る。

**落とし穴 2**: モデルのパスがリポジトリルート相対。
`emu_vehicle` 等は**リポジトリルートから実行する**。

```console
$ ./simulator/sils/build/emu_vehicle <model.xml> <duration_us> <scenario.scn>
$ ./simulator/sils/build/emu_vehicle simulator/sils/models/stampfly.xml 30000000 \
      simulator/sils/scenarios/pos_flight.scn
```

**落とし穴 3**: `hover_smoke` は 2 項目 FAIL する（`max_alt` が伸びない）。
README に既知として記載があり、このテストが StateManager をバイパスする作りに
起因する。**正規の検証経路は `.scn` シナリオ**の方。

# 初期化シーケンスが2系統ある

**本リポジトリの `driver_flow.h` が使っている Bitcraze 版とは別系統がある。**

| | Bitcraze 版（本リポジトリが採用） | PixArt 公式マニュアル版（SILS/ecosystem） |
|---|---|---|
| 構成 | 約 70 個の固定レジスタ列 | **条件分岐と検証を含む手順** |
| 検証 | なし | `0x47 == 0x08` を最大 3 回リトライ |
| 分岐 | なし | `0x67` の bit7 で `0x48` に書く値を変える |
| 出典 | Bitcraze_PMW3901 | `docs/how_to_use_pwm3901.md` |

**チップモデルは PixArt 版を前提にしている**（`0x47` が `0x08` を返すよう
モデル化されている）。本リポジトリの実装をこのエミュレータで検証するなら、
初期化を PixArt 版に寄せる必要があるかもしれない。**未検証。**

読み書きの規約自体は一致していた（読み `reg & 0x7F` / 書き `reg | 0x80`）。

# 何を見誤ったか

「エミュレータは存在しない」と結論した根拠は、**M5Stack 公式ドキュメントと
汎用 ESP32 エミュレータ（Wokwi / QEMU）しか探していなかった**こと。
コミュニティ側の実装を探していなかった。

さらに「PMW3901 のモデルが無いから原理的に検証できない」と述べたが、
**実際にはモデルが存在した。** 「無い」ことの確認は「見つからなかった」
ことでしかないのに、原理的な不可能性の主張にすり替えていた。

**探索範囲を言わずに「存在しない」と言わない。**
「公式ドキュメントには見当たらなかった」と「存在しない」は別の主張。

# 関連

実測定数と ESKF の実装は [[stampfly-ecosystem-reference]]。
設計は [[position-hold-design]]。

[^sils]: StampFly SILS (MIT License, Copyright (c) 2026 Kouhei Ito)
[^probe-run]: 本マシンでの pmw3901_probe 実行
