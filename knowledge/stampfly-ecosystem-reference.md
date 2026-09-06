---
type: Reference
title: StampFly Ecosystem — Optical Flow の実測値が得られる参照実装
description: Kouhei Ito 氏(オリジナルファームウェア作者)による教育・研究プラットフォーム。PMW3901 の完全な実装と、速度変換に必要な実測定数が揃っている。
tags: [stampfly, sensor, control, reference]
status: stable
generated: { by: claude-opus-5/1m, at: 2026-09-06T04:30:00Z }
verified:
  - { by: claude-opus-5/1m, at: 2026-09-06T04:30:00Z }
sources:
  - id: ecosystem
    resource: https://github.com/M5Fly-kanazawa/stampfly_ecosystem
    title: StampFly Ecosystem (MIT License, Copyright (c) 2026 Kouhei Ito)
    author: human:kouhei-ito
  - id: ecosystem-pmw3901
    resource: firmware/vehicle_old/components/sf_hal_pmw3901/include/pmw3901.h
    title: PMW3901 ドライバヘッダ（ピン定義・レジスタ・タイミング）
  - id: ecosystem-eskf
    resource: firmware/vehicle_old/components/sf_algo_eskf/eskf.cpp (updateFlowRaw)
    title: ESKF の Flow 更新（速度変換の実装）
---

# 何があるか

StampFly のオリジナルファームウェアを書いた Kouhei Ito 氏による
教育・研究プラットフォーム。**MIT License**。[^ecosystem]

- **PMW3901 の完全な実装**（`sf_hal_pmw3901`）
- **ESKF による姿勢・速度・位置推定**（`sf_algo_eskf`）
- 4 つの制御モード（ACRO / STABILIZE / 高度維持 / **位置保持**）
- シミュレータ、Web 書き込み、ビルド済みファームウェア

**本リポジトリが目指すポジションホールドが、既に動く形で存在する。**
ゼロから書く方針は変わらないが、**実測定数と実装の答え合わせに使える。**

# CS = GPIO 12 の三度目の裏付け

```c
#define PMW3901_DEFAULT_PIN_CS      12  // PMW3901 CS (Note: GPIO 46 is BMI270 IMU CS)
```
[^ecosystem-pmw3901]

MISO=43 / MOSI=14 / SCLK=44 も一致。これで根拠は 3 つになった:

1. 公式回路図 PDF
2. 公式ファームウェアが G12 の CS を HIGH にしている（[[stampfly-spi-pinout]]）
3. この実装のピン定義（**BMI270 が G46 という注記まで一致**）

# 速度変換の実測定数

**これが最大の収穫。** 自分で実測する前に妥当な初期値が手に入る。

| 定数 | 実測値 | 本リポジトリの当初の推定 |
|---|---|---|
| `flow_rad_per_pixel` | **0.00222 rad/px** | 0.02 rad/px（**約9倍のずれ**） |
| `flow_min_height` | 0.02 m | 0.10 m |
| `flow_gyro_scale` | 1.0 | 1.0（一致） |
| `flow_cam_to_body` | [0.943, 0, 0, 1.015] | 未考慮 |
| `flow_noise` | 0.005232 | — |
| SQUAL 最小閾値 | **0x19 (25)** | 32 |

> **訂正**: [[position-hold-design]] に書いた `K_px ≈ 0.02 rad/px` は
> **1 桁間違っていた**。視野角と画素数からの概算で出した値だが、
> 実測は 0.00222。**条件の書かれていない概算値をそのまま使うと危険**という
> 実例になった。

`flow_cam_to_body` が単位行列でない（0.943 / 1.015）のは、
**センサーの取り付け角と軸のスケール誤差を吸収している**ため。
自分で実装するときも、軸ごとにスケールが違いうると考えておく。

# 速度変換の手順

`ESKF::updateFlowRaw()` の構成。[^ecosystem-eskf]

```
1. ピクセル → 角速度      flow_cam = (Δpx − offset) · rad_per_pixel / dt
2. 回転成分の除去          flow_rot_x =  gyro_scale · gyro_y
                          flow_rot_y = −gyro_scale · gyro_x
                          flow_trans = flow_cam − flow_rot
3. カメラ座標 → 機体座標   cam_to_body 行列を掛ける
4. 並進速度                v_body = flow_trans · distance
5. 機体座標 → NED          ヨーで回す
6. ESKF の観測として更新    イノベーションクランプあり
```

**回転成分の除去（2）が実装されている**ことが確認できた。
[[position-hold-design]] で「省くと発振する」と書いた処理は必須で正しい。

**符号に注意**: x 側は `+gyro_y`、y 側は `−gyro_x`。
軸の対応と符号が入れ替わる。

# 本リポジトリとの実装の一致

`Common/driver/include/driver_flow.h` と照合した結果、以下が一致:

- レジスタ配置（0x00 / 0x02-0x07 / 0x3A / 0x5F / 0x7F）
- 期待値 0x49 / 0xB6
- **SPI クロック 2MHz**（"max 2MHz for PMW3901" と明記されている）

# 未採用だが知っておくべきもの

- **Motion Burst (0x16)** — レジスタを個別に読む代わりに一括で読める。
  本リポジトリは個別読み出しにしているが、400Hz で回すなら検討に値する
- **ESKF** — [[position-hold-design]] では相補フィルタを選んだ。
  この実装は ESKF で、ジャイロバイアスまで推定している。
  「シンプルに」方針を捨てるならこちらが参照先になる

# 注意

- 実装は `firmware/vehicle_old/` にある。**old** の名の通り旧世代の可能性があり、
  現行の実装は別の場所にあるかもしれない（未調査）
- 定数は**この機体・このレンズでの実測値**。個体差があれば合わせ込みが要る
- v1.1 についての言及は見つからなかった

[^ecosystem]: StampFly Ecosystem (MIT License, Copyright (c) 2026 Kouhei Ito)
[^ecosystem-pmw3901]: PMW3901 ドライバヘッダ
[^ecosystem-eskf]: ESKF の Flow 更新
