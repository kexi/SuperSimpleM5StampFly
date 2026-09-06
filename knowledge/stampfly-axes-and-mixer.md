---
type: Reference
title: 機体の軸・ミキサー配分・ToF ピン（実測/一次資料ベース）
description: IMU チップ軸から機体軸(FRD)への変換、X 配置クアッドのミキサー式とモーター回転方向、下向き/前向き ToF の XSHUT ピン。すべて参照実装の実証済み値。
tags: [stampfly, hardware, control, sensor]
status: stable
generated: { by: claude-opus-5/1m, at: 2026-09-06T10:15:00Z }
verified:
  - { by: claude-opus-5/1m, at: 2026-09-06T10:15:00Z }
sources:
  - id: eco-bmi270
    resource: https://github.com/M5Fly-kanazawa/stampfly_ecosystem (firmware/vehicle/components/sf_hal_bmi270/src/bmi270_wrapper.cpp)
    title: BMI270 ラッパー（軸マッピングは実機で確認済みと明記）
    author: human:kouhei-ito
  - id: eco-actuator
    resource: https://github.com/M5Fly-kanazawa/stampfly_ecosystem (firmware/vehicle/components/sf_actuator/include/actuator.hpp)
    title: X 配置クアッドのミキサー式
    author: human:kouhei-ito
  - id: eco-tof
    resource: https://github.com/M5Fly-kanazawa/stampfly_ecosystem (firmware/vehicle/components/sf_hal_vl53l3cx/Kconfig)
    title: ToF の XSHUT GPIO 既定値
    author: human:kouhei-ito
---

# IMU チップ軸 → 機体軸 (FRD)

```
body.x =  chip.y      （前）
body.y =  chip.x      （右）
body.z = -chip.z      （下）
```

加速度・角速度の**両方に同じ変換**を適用する。[^eco-bmi270]

参照実装のコメントに「実証済みハードで確認」と明記されている。
**自分で実測する前の初期値として、そのまま使える。**

> 座標系は FRD（x 前・y 右・z 下）。ドローンの慣例で、重力は +z 方向。
> 静置時の加速度は body で (0, 0, +9.8) 付近になる。

# ミキサー（X 配置クアッド）

```
        Front
   FL(CW)   FR(CCW)
       \\   /
        \\ /
        / \\
       /   \\
   RL(CCW)  RR(CW)
        Rear
```

| モーター | GPIO | 回転 |
|---|---|---|
| FL | 5 | CW |
| FR | 42 | CCW |
| RL | 10 | CCW |
| RR | 41 | CW |

配分式（T=推力、τφ=ロール、τθ=ピッチ、τψ=ヨー）:[^eco-actuator]

```
T_FR = ¼(T − τφ + τθ + τψ)
T_RR = ¼(T − τφ − τθ − τψ)
T_RL = ¼(T + τφ − τθ + τψ)
T_FL = ¼(T + τφ + τθ − τψ)
```

**符号の読み方**:
- ロール(τφ): 右側(FR/RR)が負、左側(FL/RL)が正 → 正のロールで左が上がる
- ピッチ(τθ): 前側(FR/FL)が正、後側(RR/RL)が負 → 正のピッチで前が上がる
- ヨー(τψ): CCW のモーター(FR/RL)が正、CW(RR/FL)が負

**ヨーの符号は回転方向とセット**。反トルクで機体が回るので、CCW を強めると
機体は CW に回る。ここを間違えるとヨーが発散する。

# ToF の XSHUT ピン

| センサー | XSHUT GPIO |
|---|---|
| 下向き（高度用） | **7** |
| 前向き（障害物用） | **9** |

[^eco-tof]

2 個の VL53L3CX は**同じ I2C アドレス (0x29)** なので、両方使うには XSHUT で
片方ずつ起こしてアドレスを書き換える儀式が要る。
ポジションホールドに前向きは不要なので、**前向きは XSHUT を LOW に固定して
黙らせる**のが簡単。

> **訂正**: [[position-hold-design]] に「XSHUT_BOTTOM=G7 / XSHUT_FRONT=G9 と
> 記憶しているが未確認」と書いていた。**一次資料で確認でき、記憶は正しかった。**
> ただし INT ピン（G6/G8 と記憶）はまだ未確認。

# 関連

全体設計は [[position-hold-design]]。定数の実測値は [[stampfly-ecosystem-reference]]。

[^eco-bmi270]: BMI270 ラッパー（軸マッピング）
[^eco-actuator]: X 配置クアッドのミキサー式
[^eco-tof]: ToF の XSHUT GPIO 既定値
