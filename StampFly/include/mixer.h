#pragma once

// ミキサー(制御配分)
//
// スロットル(総推力の代わり)と3軸の姿勢トルクを、4つのモーターのduty比に
// 配分する。姿勢制御(角速度PID・角度PID)の出力とモータードライバの間に立つ
// 唯一の層で、Mixer_update()を呼ぶとMotor_setSpeed()までを行う。
// 実際にPWMへ書き出すのはMotor_update()なので、呼び出し側で続けて呼ぶこと。
//
// 座標系は FRD(x前・y右・z下)。符号の意味は右手系で:
//   roll  正 = 右に傾く(右側が下がる)
//   pitch 正 = 機首が上がる
//   yaw   正 = 上から見て時計回り(機首が右を向く)
//
// 機体配置(前から見て、X配置のクアッド):
//   FL(G5)   FR(G42)
//       \   /
//       /   \
//   RL(G10)  RR(G41)
//
// Why not 物理量(推力[N]・トルク[Nm])を入力にする: 参照実装は B^-1 の制御配分を
//   使い、推力をモータ曲線でdutyに変換している。そのためには推力係数Ct、
//   トルク/推力比κ、モータ電圧曲線、そして実電池電圧が要る。参照実装でさえ
//   Ctは「暫定値」扱いで、実測なしに持ってきても正しい配分にはならない。
//   ここではPIDのゲインが単位を吸収する前提で、正規化した無次元量
//   (throttle/roll/pitch/yaw ともに概ね -1.0〜1.0 のスケール)を受け取る。
//   PIDゲインを詰める段(二十七〜二十八歩)で、このスケールごと決まる。

#include "driver_motor.h"  // Motor_setSpeed / MOTOR_FL..MOTOR_RR

// 出力上限 --------------------------------------
// 最終dutyの上限。1.0まで回すと飽和の余地がゼロになり、姿勢トルクを足す先が
// なくなる。0.8にして2割を姿勢制御の余力として残す。
// 出典: knowledge/position-hold-design.md「出力上限: 最終dutyを0.8でクランプ」
//
// Why not 1.0:
// 全開付近ではモーターの応答も推力の線形性も落ちる。上限に張り付いた
//   モーターは「これ以上強くできない」ので、その軸の姿勢制御が片効きになる。
#define MOTOR_DUTY_MAX 0.8f

// ヨーの回転方向 ---------------------------------
// 対角ペアでプロペラの回転方向が逆になっている。同じ向きのペアを強めると、
// その反トルクで機体が逆向きに回る。
//   FR / RL … CCW(反時計回り) → 機体には CW(+yaw) の反トルク
//   FL / RR … CW (時計回り)   → 機体には CCW(-yaw) の反トルク
// 出典: StampFly Ecosystem (MIT License, Copyright (c) 2026 Kouhei Ito)
//       firmware/vehicle/components/sf_actuator/actuator.cpp の mixerCompute()
//       および sf_hal_motor/include/motor_driver.hpp のコメント
//       (M1=FR:CCW / M2=RR:CW / M3=RL:CCW / M4=FL:CW)
//       https://github.com/M5Fly-kanazawa/stampfly_ecosystem
//
// Why not 自分で決める: 回す向きを間違えると、ヨーの指令が正帰還になって
//   スピンし始める。実機で確かめるにはプロペラを付ける必要があり、
//   間違ったまま飛ばすのが最も危険。実績のある実装に合わせる。
#define MIXER_YAW_SIGN_FL (-1.0f)  // CW  プロペラ
#define MIXER_YAW_SIGN_FR (+1.0f)  // CCW プロペラ
#define MIXER_YAW_SIGN_RL (+1.0f)  // CCW プロペラ
#define MIXER_YAW_SIGN_RR (-1.0f)  // CW  プロペラ

// 最後に計算した各モーターのduty(ログ・デバッグ用に保持する)
static float _mixer_duty[MOTOR_MAX] = {0};

// スロットルをどれだけ下げたか[duty]。0以外なら飽和していたということ。
static float _mixer_throttle_cut = 0.0f;

// 値を[lo, hi]に収める
static float _Mixer_clamp(float value, float lo, float hi) {
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

// 全モーターを停止する
//
// ディスアーム時・失陥時にここを通す。内部の保持値もゼロにするので、
// ログに「止めた」ことがそのまま出る。
void Mixer_stop() {
    for (int i = 0; i < MOTOR_MAX; i++) {
        _mixer_duty[i] = 0.0f;
        Motor_setSpeed(i, 0.0f);
    }
    _mixer_throttle_cut = 0.0f;
}

// ミキサーの初期化
//
// Why not 何もしないなら関数を作らない: 他のドライバが Xxx_init を持つので
//   作法を揃える。呼び出し側の setup() の並びが素直になる。
void Mixer_init() { Mixer_stop(); }

// スロットルと3軸トルクを4つのモーター出力に配分する
//
// throttle : 0.0〜1.0 の総出力。ホバリングは概ね 0.4〜0.6(要実測)
// roll     : 正で右に傾く
// pitch    : 正で機首が上がる
// yaw      : 正で機首が右を向く
//
// X配置なので、roll も pitch も4つ全部が寄与する。左右で分けるのが roll、
// 前後で分けるのが pitch、対角ペアの差が yaw。
void Mixer_update(float throttle, float roll, float pitch, float yaw) {
    // roll 正 = 右に傾く = 左側を強める。
    // pitch 正 = 機首上げ = 前側を強める。
    // yaw は同じ回転方向のプロペラ2個(対角ペア)をまとめて強める。
    // 出典: 上記 actuator.cpp mixerCompute() の B^-1 と同じ符号
    //       (FR = -roll +pitch +yaw / RR = -roll -pitch -yaw /
    //        RL = +roll -pitch +yaw / FL = +roll +pitch -yaw)
    float duty[MOTOR_MAX];
    duty[MOTOR_FL] = throttle + roll + pitch + yaw * MIXER_YAW_SIGN_FL;
    duty[MOTOR_FR] = throttle - roll + pitch + yaw * MIXER_YAW_SIGN_FR;
    duty[MOTOR_RL] = throttle + roll - pitch + yaw * MIXER_YAW_SIGN_RL;
    duty[MOTOR_RR] = throttle - roll - pitch + yaw * MIXER_YAW_SIGN_RR;

    // 飽和処理: スロットルを下げて姿勢トルクを優先する ------------------
    //
    // どれかが MOTOR_DUTY_MAX を超えたとき、単純にクランプすると、そのモーター
    // だけ指令より弱くなる。4個の差(=姿勢トルク)が崩れ、姿勢が意図しない方向へ
    // 傾く。4個とも飽和すれば差が完全に消え、姿勢制御が効かなくなる。
    //
    // そこで、はみ出した分を全モーターから等しく引く。全部から同じ量を引いても
    // モーター間の差は変わらないので、姿勢トルクはそのまま残り、
    // 失うのは総推力(=高度)だけになる。高度は数十cm沈むだけで済むが、
    // 姿勢を失うと墜落するため、この優先順位にする。
    // 出典: knowledge/position-hold-design.md
    //       「飽和時はスロットルを下げて姿勢トルクを優先(4個とも飽和すると
    //         姿勢が保てない)」
    //
    // Why not はみ出した分だけクランプする: 上記のとおり姿勢トルクが崩れる。
    // Why not 姿勢トルクの方をスケールダウンする: 総推力を守れるが、
    //   姿勢を犠牲にする。優先順位が逆。
    float max_duty = duty[0];
    for (int i = 1; i < MOTOR_MAX; i++) {
        if (duty[i] > max_duty) {
            max_duty = duty[i];
        }
    }

    _mixer_throttle_cut     = 0.0f;
    const bool is_saturated = max_duty > MOTOR_DUTY_MAX;
    if (is_saturated) {
        _mixer_throttle_cut = max_duty - MOTOR_DUTY_MAX;
        for (int i = 0; i < MOTOR_MAX; i++) {
            duty[i] -= _mixer_throttle_cut;
        }
    }

    // 下側の飽和は救えない。逆推力を出せない(プロペラは片方向にしか回らない)ので、
    // 負のdutyは0にするしかない。上と同じように全体を持ち上げると、今度は
    // スロットルが指令より増えて上昇してしまうため、ここは素直にクランプする。
    //
    // Why not 下も同じように全体をシフトする: 総推力が勝手に増える。
    //   姿勢トルクを守るために高度を「上げる」のは、天井や人に近づく方向で危険。
    //   そもそも throttle が十分低いときにしか起きないので、
    //   スロットルを上げてから姿勢を作る(=先に浮かせる)方が筋が良い。
    for (int i = 0; i < MOTOR_MAX; i++) {
        _mixer_duty[i] = _Mixer_clamp(duty[i], 0.0f, MOTOR_DUTY_MAX);
        Motor_setSpeed(i, _mixer_duty[i]);
    }
}

// 最後に配分した各モーターのdutyを取得する(CSVログ・デバッグ用)
float Mixer_getDuty(int index) {
    if (index < 0 || index >= MOTOR_MAX) {
        return 0.0f;
    }
    return _mixer_duty[index];
}

// 直前のMixer_update()で飽和のために引いたスロットル量[duty]を取得する
//
// 0でなければ出力が足りていない。ホバリング中に常に0でないなら、
// 機体が重すぎるかPIDゲインが大きすぎる。ログに出して見張るための値。
float Mixer_getThrottleCut() { return _mixer_throttle_cut; }

// 直前のMixer_update()で出力が飽和したかを返す
bool Mixer_isSaturated() { return _mixer_throttle_cut > 0.0f; }
