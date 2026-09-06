#pragma once

// 姿勢推定(相補フィルタ)
//
// IMU(BMI270)の加速度・角速度から、機体のロール・ピッチ・ヨーを推定する。
// 座標系は FRD (x:前 / y:右 / z:下)。角度はオイラー角[rad]で、
// ホバリング時の動作域(±15°程度)しか使わないのでジンバルロックは考えない。
//
// 符号(物理から検証済み。mixer.h と一致している):
//   roll  正 = 右側が下がる
//   pitch 正 = 機首が上がる
//   yaw   正 = 上から見て時計回り(機首が右を向く)
//
// pitch の符号は次のように確認した。FRD では静置時 accel=(0,0,+g)。
// 機首を θ 上げると重力は機体座標で x の負側に傾き ax = -g*sin(θ) となるので、
//   atan2(-ax, sqrt(ay^2+az^2)) = +θ
// つまり機首上げが正。mixer.h の「pitch 正 = 前側を強める = 機首上げ」と
// 揃っているので、**角度PIDで符号を反転してはいけない。**
//
// Why not FRD 右手系の +y まわり正回転(=機首下げ)を正とする: 右手系に忠実に
//   取ると mixer.h と符号が逆になり、角度PIDのどこかで反転が要る。
//   反転を1か所でも書き忘れると、機首上げ指令で機首を下げる正帰還になり
//   離陸直後に裏返る。推定・配分・制御の3者で符号を揃える方が安全。
//
// Why not EKF / Madgwick: ホバリングは ±15° 以内の狭い動作域で、この範囲では
//   相補フィルタと Kalman の精度差が実用上出ない。EKF は破綻したときに
//   何が悪いのか追えず、Madgwick はクォータニオンを持ち込むぶん
//   ロール・ピッチが「読めば分かる値」でなくなる。
//   詳細は knowledge/position-hold-design.md の「推定はすべて相補フィルタ」。
//
// Why not 磁気センサ(BMM150)でヨーを補正する:
// ヨーの絶対値はポジションホールドに
//   不要(ヨーレート0を保てば足りる)。室内では鉄筋や家電で狂ううえ、
//   キャリブレーションの手順が増える。ヨーはジャイロ積分のみとし、
//   ドリフトは許容する。
//
// 前提: 呼び出し側が毎周期 Imu_update() を呼んでから Attitude_update() を呼ぶ。

#include <math.h>

#include "driver_imu.h"

// 制御周期 -------------------------------------
// 400Hz(2500us)。main.cpp の CONTROL_PERIOD_US と揃える。
#define ATTITUDE_UPDATE_HZ 400.0f

// 相補フィルタの時定数[s] ----------------------
// α = τ / (τ + dt) の関係で、τ=0.25s, dt=1/400s とすると
//   α = 0.25 / (0.25 + 0.0025) = 0.990099...  ≒ 0.99
// この τ より速い変化はジャイロが、遅い変化(=重力方向)は加速度が担う。
//
// Why not α を直接 0.99 と書く: α は制御周期に依存する数字なので、
//   周期を変えた瞬間に嘘になる。時定数[s]は周期によらない物理量なので、
//   こちらを定数にして α は dt から毎回計算する。
//
// Why not τ をもっと長く(=accel をもっと信じない): 長くするとジャイロの
//   残留バイアスが効いてゆっくり傾いていく。0.25s は参照実装の
//   相補フィルタ実装(α=0.98 @400Hz、τ≒0.125s)と同じ桁で、そこから
//   accel ノイズ側に少し寄せた値。**要実測**(±30°傾けて追従と収束を見る)。
// 出典: StampFly Ecosystem (MIT License, Copyright (c) 2026 Kouhei Ito)
//       firmware/workshop/lessons/lesson_09_estimation/solution.cpp
//       https://github.com/M5Fly-kanazawa/stampfly_ecosystem
#define ATTITUDE_TIME_CONSTANT 0.25f

// ジャイロバイアス較正 --------------------------
// 起動時に静置して平均を取る時間[s]と、そのサンプル数。
// 2秒 = 800サンプル。BMI270のジャイロノイズは平均でほぼ消える長さ。
//
// Why not 起動のたびに較正せず定数を持つ: ジャイロのゼロ点は個体差と
//   温度で動く。定数にすると温度が変わった日にヨーが回り続ける。
#define ATTITUDE_CALIB_SECONDS 2.0f
#define ATTITUDE_CALIB_SAMPLES \
    ((uint32_t)(ATTITUDE_CALIB_SECONDS * ATTITUDE_UPDATE_HZ))

// 重力加速度[m/s^2]。加速度ノルムの妥当性判定に使う。
#define ATTITUDE_GRAVITY 9.80665f

// accel補正を弱める判定のしきい値 ---------------
// 加速度のノルムが 1g からこれ以上ずれていたら、accel は重力方向を
// 指していないとみなして補正の重みを落とす。
//
// Why これを入れるか: 相補フィルタは「加速度計は重力だけを測っている」と
//   仮定している。機体が加速している間(離陸・急な操舵・着地の衝撃)は
//   並進加速度が混ざり、atan2 の結果が実際の姿勢とは違う角度を指す。
//   そのまま混ぜると、加速するたびに姿勢推定が傾く方向へ引っ張られ、
//   姿勢制御がそれを打ち消そうとしてさらに加速する、という悪循環になる。
//
// Why not 完全に捨てる(ゲート方式): 0/1で切り替えると、しきい値の境目で
//   補正が急にON/OFFして推定角が段差を持つ。段差はD項に乗ってモーターの
//   ガタつきになる。ずれの大きさに応じて連続的に弱める方式にする。
//   参照実装のESKFも、観測ノイズRを gravity_diff^2 に比例して増やす
//   (棄却ではなく重みを下げる)方式を採っている。
// 出典: StampFly Ecosystem (MIT License, Copyright (c) 2026 Kouhei Ito)
//       firmware/vehicle_old/components/sf_algo_eskf/eskf.cpp
//       ESKF::updateAccelAttitude() の Adaptive R
//
// 1.5m/s^2 ≒ 0.15g。ホバリング中の振動で常時弱めてしまわない程度に広く、
// 明らかな加減速は捉えられる幅。**要実測**(ホバリング中のノルム分布を見る)。
#define ATTITUDE_ACCEL_TRUST_MARGIN 1.5f

// 重みが 1/2 になるずれの大きさ[m/s^2]。
// 信頼度 = margin^2 / (margin^2 + diff^2) の形にするので、
// diff = margin のとき 0.5、diff が大きいほど 0 に近づく。
#define ATTITUDE_ACCEL_TRUST_SCALE ATTITUDE_ACCEL_TRUST_MARGIN

// 推定値(すべてFRD機体軸) ---------------------
static float _att_roll  = 0.0f;  // ロール[rad] 右下がりが正
static float _att_pitch = 0.0f;  // ピッチ[rad] 機首上げが正(mixer.hと一致)
static float _att_yaw   = 0.0f;  // ヨー[rad]   右回りが正(ジャイロ積分のみ)

static float _att_roll_rate  = 0.0f;  // ロール角速度[rad/s](バイアス補正済み)
static float _att_pitch_rate = 0.0f;  // ピッチ角速度[rad/s]
static float _att_yaw_rate   = 0.0f;  // ヨー角速度[rad/s]

// ジャイロバイアス[rad/s](機体軸)
static float _att_gyro_bias[3] = {0.0f, 0.0f, 0.0f};

// 較正の進捗
static uint32_t _att_calib_count   = 0;
static float    _att_calib_sum[3]  = {0.0f, 0.0f, 0.0f};
static bool     _att_is_calibrated = false;

// IMUのチップ軸 → 機体軸(FRD)への変換
//
// BMI270は基板への実装向きの都合でチップ軸と機体軸が一致しない。
// ドライバ(driver_imu.h)はチップ軸のまま返すので、ここで吸収する。
//
//   body.x(前) =  chip.y
//   body.y(右) =  chip.x
//   body.z(下) = -chip.z
//
// この対応は実機で機体を傾けて確認した実測値(二十四歩)。
// 加速度・角速度で同じ変換が使える(どちらも同じチップの軸で出るため)。
//
// Why not ドライバ側で変換する: driver_imu.h は「BMI270というチップの
//   ドライバ」で、StampFlyという機体の実装向きを知る立場にない。
//   基板が変われば変換だけが変わるので、機体側のこのファイルに置く。
static void _Attitude_chipToBody(const float* chip, float* body) {
    body[0] = chip[1];
    body[1] = chip[0];
    body[2] = -chip[2];
}

// 姿勢推定の初期化
//
// Imu_init() が成功した後に呼ぶ。この時点では較正は済んでおらず、
// Attitude_update() を静置したまま2秒ぶん回して初めて Attitude_isCalibrated()
// が true になる。
//
// Why not ここで2秒ブロックして較正する: init の中で delay すると、
//   その間 LED もブザーも更新されず「固まったのか較正中なのか」が
//   外から分からない。ループの中で進めて、状態を問い合わせられる形にする。
void Attitude_init() {
    _att_roll  = 0.0f;
    _att_pitch = 0.0f;
    _att_yaw   = 0.0f;

    _att_roll_rate  = 0.0f;
    _att_pitch_rate = 0.0f;
    _att_yaw_rate   = 0.0f;

    for (int i = 0; i < 3; i++) {
        _att_gyro_bias[i] = 0.0f;
        _att_calib_sum[i] = 0.0f;
    }

    _att_calib_count   = 0;
    _att_is_calibrated = false;
}

// ジャイロバイアスの較正が終わっているか
//
// false の間は姿勢が信用できないので、呼び出し側はアームを許可しない。
bool Attitude_isCalibrated() { return _att_is_calibrated; }

// 較正の進捗(0.0-1.0)。LEDの点滅などに使う。
float Attitude_getCalibrationProgress() {
    if (_att_is_calibrated) {
        return 1.0f;
    }
    return (float)_att_calib_count / (float)ATTITUDE_CALIB_SAMPLES;
}

// 姿勢推定の更新
//
// dt[s] は前回の呼び出しからの経過時間。Timer で同期しているので
// ほぼ 1/400s で一定だが、ジッタを吸収できるよう引数で受ける。
//
// 較正が終わるまでは角速度の平均を取るだけで、角度は更新しない。
// 静置中に積分しても意味がなく、較正中の振動をそのまま角度に入れると
// 較正完了の瞬間から傾いた状態で始まるため。
void Attitude_update(float dt) {
    float chip_accel[3];
    float chip_gyro[3];
    Imu_getAccel(chip_accel);
    Imu_getGyro(chip_gyro);

    float accel[3];
    float gyro[3];
    _Attitude_chipToBody(chip_accel, accel);
    _Attitude_chipToBody(chip_gyro, gyro);

    // --- 較正フェーズ ---------------------------------------------
    if (!_att_is_calibrated) {
        for (int i = 0; i < 3; i++) {
            _att_calib_sum[i] += gyro[i];
        }
        _att_calib_count++;

        const bool is_calib_done = _att_calib_count >= ATTITUDE_CALIB_SAMPLES;
        if (!is_calib_done) {
            return;
        }

        for (int i = 0; i < 3; i++) {
            _att_gyro_bias[i] = _att_calib_sum[i] / (float)_att_calib_count;
        }
        _att_is_calibrated = true;

        // 較正が終わった時点の姿勢を accel から一発で決める。
        // Why not 0から相補フィルタで寄せる: 時定数0.25sだと真値に届くまで
        //   1秒近くかかる。その間の「まだ寄っている途中の角度」で
        //   姿勢制御を始めると、傾いていないのに傾き補正が入る。
        const float ax = accel[0];
        const float ay = accel[1];
        const float az = accel[2];
        _att_roll      = atan2f(ay, az);
        _att_pitch     = atan2f(-ax, sqrtf(ay * ay + az * az));
        _att_yaw       = 0.0f;  // ヨーは絶対基準を持たないので起動姿勢を0とする
        return;
    }

    // --- 角速度(バイアス補正済み) ---------------------------------
    _att_roll_rate  = gyro[0] - _att_gyro_bias[0];
    _att_pitch_rate = gyro[1] - _att_gyro_bias[1];
    _att_yaw_rate   = gyro[2] - _att_gyro_bias[2];

    // --- ジャイロ積分による予測 -----------------------------------
    // Why not オイラー角の厳密な運動学(tan・secを含む変換行列)を使う:
    //   ±15°の動作域では補正項は数%で、姿勢制御のゲイン余裕に埋もれる。
    //   一方 tan は ±90° で発散し、転倒時に推定が壊れる経路を増やす。
    const float roll_gyro  = _att_roll + _att_roll_rate * dt;
    const float pitch_gyro = _att_pitch + _att_pitch_rate * dt;

    // ヨーはジャイロ積分のみ。accel は鉛直軸まわりの回転を観測できないので、
    // そもそも補正する材料がない(だから磁気センサが要る、という話になる)。
    _att_yaw += _att_yaw_rate * dt;

    // --- 加速度による補正 -----------------------------------------
    const float ax = accel[0];
    const float ay = accel[1];
    const float az = accel[2];

    const float accel_norm = sqrtf(ax * ax + ay * ay + az * az);

    // 自由落下や配線断でノルムが0に近いと atan2 の向きが定まらない。
    // 割り算はしていないが、意味のない角度を混ぜないためにここで捨てる。
    const bool is_accel_usable = accel_norm > 1.0f;
    if (!is_accel_usable) {
        _att_roll  = roll_gyro;
        _att_pitch = pitch_gyro;
        return;
    }

    // FRDでは静置時 az ≒ +1g(重力は下向き = +z)。
    // roll は y-z 平面、pitch は x と水平面成分の比で出る。
    const float roll_acc  = atan2f(ay, az);
    const float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az));

    // 1g からのずれが大きいほど accel を信じない。
    // trust = s^2 / (s^2 + diff^2) は diff=0 で1、diff=s で0.5、
    // diff→∞ で0 に連続的に落ちる。段差が出ないのでD項に乗らない。
    const float gravity_diff = fabsf(accel_norm - ATTITUDE_GRAVITY);
    const float scale_sq =
        ATTITUDE_ACCEL_TRUST_SCALE * ATTITUDE_ACCEL_TRUST_SCALE;
    const float accel_trust =
        scale_sq / (scale_sq + gravity_diff * gravity_diff);

    // α = τ / (τ + dt)。dt を毎回使うので、周期が揺れても時定数は保たれる。
    const float alpha = ATTITUDE_TIME_CONSTANT / (ATTITUDE_TIME_CONSTANT + dt);

    // accel 側の重みを trust で縮める。trust=0 なら純粋なジャイロ積分になる。
    const float accel_weight = (1.0f - alpha) * accel_trust;
    const float gyro_weight  = 1.0f - accel_weight;

    _att_roll  = gyro_weight * roll_gyro + accel_weight * roll_acc;
    _att_pitch = gyro_weight * pitch_gyro + accel_weight * pitch_acc;
}

// ロール角[rad]を取得する。右下がりが正。
float Attitude_getRoll() { return _att_roll; }

// ピッチ角[rad]を取得する。機首上げが正。
float Attitude_getPitch() { return _att_pitch; }

// ヨー角[rad]を取得する。右回りが正。
//
// ジャイロ積分のみなので絶対方位ではなく、起動時を0とした相対角。
// 時間とともにドリフトする。ヨーレート0を保つ用途にだけ使う。
float Attitude_getYaw() { return _att_yaw; }

// ロール角速度[rad/s]を取得する。PIDのD項に使う。
//
// 角度の差分ではなくジャイロの生値(バイアス補正済み)を返す。
// Why not 角度を微分する: 微分はノイズを増幅するうえ、相補フィルタを
//   通った後の角度は accel のノイズも含んでいる。ジャイロは元から
//   角速度を測っているので、そのまま使うのが最もノイズが少ない。
float Attitude_getRollRate() { return _att_roll_rate; }

// ピッチ角速度[rad/s]を取得する。
float Attitude_getPitchRate() { return _att_pitch_rate; }

// ヨー角速度[rad/s]を取得する。
float Attitude_getYawRate() { return _att_yaw_rate; }

// ジャイロバイアス[rad/s]を取得する(動作確認用)。out には3要素分の領域が要る。
void Attitude_getGyroBias(float* out) {
    out[0] = _att_gyro_bias[0];
    out[1] = _att_gyro_bias[1];
    out[2] = _att_gyro_bias[2];
}
