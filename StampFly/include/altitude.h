#pragma once

// 高度推定と高度制御(三十歩)
//
// 下向きToFと加速度から高度[m]と昇降速度[m/s]を推定し、その高度を保つ
// スロットル(0.0-1.0)を作る。ミキサーの throttle 入力にそのまま渡せる。
//
// 座標系は FRD (x前・y右・z下)。ただし**このファイルが外に出す高度と速度は
// 上向きを正**にする。ToFが返す「地面までの距離」も、操縦者の「上げたい」も
// 上向きで考える量で、FRD の z(下向き正)に合わせると読むたびに符号を
// 反転することになるため。内部でFRDの加速度を扱う箇所だけ符号を返している。
//
// 構成:
//   予測(400Hz)     : 加速度の鉛直成分を積分して vz と h を進める
//   補正(ToF新データ): e = h_tof - h ; h += k1*e ; vz += k2*e
//   制御            : 高度PID。D項には推定した vz を使う
//
// Why not ToFの生値をそのまま高度制御に使う: ToFは30Hz・数mm量子化で、
//   400Hzの制御ループから見ると「13周期に1回しか動かない階段状の信号」に
//   なる。P項に使うだけならまだしも、差分でD項を作ると階段の段差が
//   1/dt = 400倍されてモーターに乗る。加速度で周期間を埋めることで、
//   400Hzで滑らかな h と vz が得られる。
//   出典: knowledge/position-hold-design.md「高度(30歩)」
//         「却下: ToF差分で vz を作る。30Hz・数mm量子化の差分は
//           D項に使えるノイズ品質でない」
//
// Why not 気圧計(BMP280)を混ぜる: BMP280は絶対標高(海面基準)を返すのに対し、
//   下向きToFは対地高度を返す。ゼロ点合わせをせずに同じ状態へ融合すると、
//   両者が引っ張り合って設置場所の標高ぶん高度保持がずれる。参照実装も
//   同じ理由で鉛直はToFのみにしている。室内0.3〜1.5mでは気圧の分解能
//   (数十cm)そのものが足りない。
//   出典: StampFly Ecosystem (MIT License, Copyright (c) 2026 Kouhei Ito)
//         firmware/vehicle/components/sf_estimator_complementary/
//         complementary_estimator.cpp の updateBaro()
//         https://github.com/M5Fly-kanazawa/stampfly_ecosystem

#include <math.h>
#include <stdint.h>

#include "attitude.h"    // Attitude_getRoll / getPitch [rad]
#include "driver_imu.h"  // Imu_getAccel [m/s^2](チップ軸)
#include "driver_tof.h"  // Tof_getDistance [m] / Tof_isValid
#include "pid.h"         // Pid / Pid_init / Pid_update / PID_DT_MAX

// 重力加速度[m/s^2]
#define ALTITUDE_GRAVITY 9.80665f

// 推定のゲイン ----------------------------------
//
// ToFの新データが来たときだけ、誤差 e = h_tof - h を
//   h  += ALTITUDE_K_HEIGHT   * e
//   vz += ALTITUDE_K_VELOCITY * e
// の形で戻す。共分散を持たない固定ゲインの2状態カルマン(定常ゲイン版)で、
// 「高度には強く、速度にはやんわり」という重み付けになっている。
//
// 高度側 0.30 は「ToF 1回で誤差の3割を消す」ということ。30Hz で来るなら
// 時定数はおよそ 1/(30*0.30) ≒ 0.11s。速い外乱は加速度積分が拾い、
// ゆっくりしたドリフト(加速度バイアスの二重積分)をToFが押さえる。
//
// 速度側 0.05 が小さいのは、高度の誤差から速度を推定するのが
// 「1回ぶんの差から傾きを当てる」という無理のある推定だから。
// 大きくすると、ToFが1回外れた(床の段差・反射率の変化)だけで
// 速度が跳ね、D項を通ってモーターが蹴る。
//
// Why not 誤差の大きさで共分散を回す本物のカルマンにする: 定常状態では
//   カルマンゲインは一定値に収束するので、その収束値を直接置くのと
//   結果は変わらない。共分散更新の行列演算とチューニング項目(Q, R)が
//   増えるぶん、何が効いているのか追えなくなる。
//   出典: knowledge/position-hold-design.md
//         「高度だけKF → 固定ゲイン2状態相補がKFの定常ゲイン版と等価」
//
// 値の出典: StampFly Ecosystem (MIT License, Copyright (c) 2026 Kouhei Ito)
//   firmware/vehicle/components/sf_estimator_complementary/
//   complementary_estimator.cpp の kTofAltGain = 0.30f / kTofVelGain = 0.05f
//   **要実測**(ToF更新レートが30Hzでない場合はこの値のままでは合わない。
//   一定高度に手で保持して h の追従と vz のノイズ幅を同時に見る)
#define ALTITUDE_K_HEIGHT   0.30f
#define ALTITUDE_K_VELOCITY 0.05f

// ToF補正を捨てる傾き[rad] -----------------------
// これ以上傾くとビームが真下の床から外れ、cos補正では直せない距離を
// 測る(隣の壁や家具までの斜距離になる)。補正をスキップして予測だけで進める。
//
// Why not cos補正だけで押し通す: cos補正は「ビームの当たっている面が
//   真下の床と同じ平面である」ことを前提にしている。傾いてビームが
//   壁を向いた瞬間、その前提が崩れて補正が嘘になる。
// 出典: 上記 complementary_estimator.cpp の kTofMaxTiltRad = 0.5f
#define ALTITUDE_TOF_MAX_TILT_RAD 0.5f  // 約29度

// ToFの鮮度[us] ---------------------------------
// 最後に有効なToFが来てからこれを超えたら、高度推定は信用できないとみなす。
// 30Hz(33ms)に対して6回分の猶予。
// 出典: knowledge/position-hold-design.md の失陥表
//       「ToF無効 | 200ms | 高度ホールド→手動スロットルに降格」
#define ALTITUDE_TOF_TIMEOUT_US 200000  // 200ms

// 高度制御のPIDゲイン ---------------------------
// 単位: 誤差[m] → スロットル[duty 0.0-1.0]。
//
// kp = 0.6 は「10cm低ければ duty を 0.06 足す」という置き方。
// ホバーduty 0.676 に対して約9%の増減で、上下方向の加速度としては
// おおよそ 0.9m/s^2 に相当する(推力がdutyに比例すると仮定した粗い換算)。
//
// ki は定常偏差(電池が減る・機体が重くなる)を消すためだけに使う。
// 高度は「じわじわずれる」量なので小さくてよい。
//
// kd は vz に掛かる。0.25 なら 0.4m/s の降下で duty +0.10。
// 落ち始めを早めに止める役をこれが担う。
//
// **すべて要実測。** 実機ではテザーを付け、
// 目標50cmで±5cm(position-hold-design.md の三十歩の検証条件)に
// 収まるところまで kd → kp → ki の順に上げる。
//
// Why not 参照実装のゲインを持ってくる: あちらは推力[N]を操作量にし、
//   質量とモータ曲線で duty に直している。こちらの操作量は正規化した
//   duty そのものなので、単位が違う数字を写しても意味がない。
//   ミキサーが無次元量を受け取る設計(mixer.h の Why not)の裏返し。
#define ALTITUDE_PID_KP 0.6f
#define ALTITUDE_PID_KI 0.2f
#define ALTITUDE_PID_KD 0.25f

// I項の上限[duty]。ホバーdutyの1割強。
// これを超えて足りないなら、ゲインではなく機体か電池の問題。
#define ALTITUDE_PID_I_LIMIT 0.10f

// ホバー分からの増減の上限[duty]。
// 上限に張り付いても、ホバー + 0.20 = 約0.88 になる。ミキサー側で
// MOTOR_DUTY_MAX(0.8) にクランプされるので、実際にはそこで頭打ちになる。
//
// Why not もっと大きく取る: 高度PIDが暴れたときの上昇量を、
//   姿勢制御の余力(mixer.h が残す2割)より小さく抑えておく。
#define ALTITUDE_PID_OUT_LIMIT 0.20f

// D項の合計上限[duty] ----------------------------
// P項とI項は Pid が out_limit で切ってくれるが、自前で作るD項には
// 切る主体がいない。vz が飛べば D項も飛ぶ。
//
// Why not P・I と同じ ALTITUDE_PID_OUT_LIMIT に含めて一度に切る:
//   PIDの出力を受け取ってから足すので、Pid の中のクランプは通らない。
//   D項だけを独立に切っておかないと、合計を切ったときに
//   「D項が大きすぎてP項が消える」形になる。
#define ALTITUDE_PID_D_LIMIT 0.15f

// ホバリングに必要なスロットル[duty] --------------
//
// 高度PIDの出力はこの値からの「増減」として扱う。PIDが0を出しても
// 機体が浮いたままになるので、I項が重力ぶんを積み上げるのを待たなくてよい。
//
// 0.676 は参照実装のSILSワークショップ環境での実測ホバーduty。
// **この機体・この電池での値ではない。** 電池電圧が下がれば同じdutyでも
// 推力は落ちるし、プロペラの摩耗でも変わる。あくまで初期値の目安。
// 出典: StampFly Ecosystem (MIT License, Copyright (c) 2026 Kouhei Ito)
//       docs/events/stampfly_workshop/workshop_guide.md
//       「workshop_acro.scn のスティック指令(duty)は …
//         実測ホバー duty(~0.676)付近で中立に収束する前提」
//       https://github.com/M5Fly-kanazawa/stampfly_ecosystem
//
// **要実測**(二十八歩の手動ホバリングで、水平を保って高度が変わらない
// ときのスティック位置を読む。Altitude_setHoverThrottle() で差し替える)
#define ALTITUDE_HOVER_THROTTLE_DEFAULT 0.676f

// 高度制御の状態 --------------------------------
// 呼び出し側が「高度ホールドを続けてよいか」を判断できるようにする。
//
// Why not bool 1つ(使える/使えない)にする: ToFが一度も来ていないのと、
//   飛行中に見失ったのとでは、操縦者に伝えるべきことが違う。前者は
//   離陸前の配線・初期化の問題で、後者は今すぐスロットルを握る合図。
enum AltitudeStatus {
    ALTITUDE_STATUS_NO_DATA,  // ToFがまだ一度も来ていない(離陸前)
    ALTITUDE_STATUS_OK,       // 推定が生きている。高度ホールドしてよい
    ALTITUDE_STATUS_STALE,    // ToFが途絶えた。手動スロットルへ降格する
};

// 推定値(上向き正) ------------------------------
static float _alt_height   = 0.0f;  // 推定高度[m] 地面からの高さ
static float _alt_velocity = 0.0f;  // 推定昇降速度[m/s] 上昇が正

// 制御 ------------------------------------------
static float _alt_target         = 0.0f;  // 目標高度[m]
static float _alt_throttle       = 0.0f;  // 出力スロットル[duty]
static float _alt_hover_throttle = ALTITUDE_HOVER_THROTTLE_DEFAULT;
static Pid   _alt_pid;

// ToFの状態 -------------------------------------
static int64_t        _alt_last_tof_us = 0;  // 最後に有効なToFを受けた時刻[us]
static AltitudeStatus _alt_status      = ALTITUDE_STATUS_NO_DATA;

// 前回取り込んだToFの距離[m]。新データの検出に使う。
static float _alt_prev_tof_distance = -1.0f;

// 現在時刻[us]を取る。SILSのホストテストでは esp_timer が無いので分ける。
// safety.h と同じ作法。
static int64_t _Altitude_nowUs() {
#ifdef SF_HOST_TEST
    extern int64_t AltitudeTest_nowUs();  // テスト側が時刻を進める
    return AltitudeTest_nowUs();
#else
    return esp_timer_get_time();
#endif
}

// 値を ±limit に収める
static float _Altitude_clamp(float value, float limit) {
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

// 高度推定・高度制御の初期化
//
// Imu_init() / Attitude_init() / Tof_init() の後に呼ぶ。ToFはまだ読めて
// いない前提で、状態は ALTITUDE_STATUS_NO_DATA から始まる。
void Altitude_init() {
    _alt_height   = 0.0f;
    _alt_velocity = 0.0f;

    _alt_target         = 0.0f;
    _alt_throttle       = 0.0f;
    _alt_hover_throttle = ALTITUDE_HOVER_THROTTLE_DEFAULT;

    _alt_last_tof_us       = 0;
    _alt_status            = ALTITUDE_STATUS_NO_DATA;
    _alt_prev_tof_distance = -1.0f;

    // kd は 0 で渡す。D項は Altitude_update() が推定した vz から自前で作る
    // (理由は Altitude_update() のD項のコメント)。ALTITUDE_PID_KD は
    // そちらで使う。
    Pid_init(&_alt_pid, ALTITUDE_PID_KP, ALTITUDE_PID_KI, 0.0f,
             ALTITUDE_PID_I_LIMIT, ALTITUDE_PID_OUT_LIMIT);

    // D項を使わないので、Pid 内部の一次遅れフィルタも要らない。
    // position.h の外側ループと同じ扱い。
    Pid_setDCutoff(&_alt_pid, 0.0f);
}

// 推定と制御のリセット
//
// ディスアーム時と着地時に呼ぶ。高度・速度・I項を全てゼロに戻す。
//
// Why not 高度だけ残す: 地上に置いている間も予測は回り続け、加速度の
//   残留バイアスが二重積分で溜まる。次に離陸するとき「浮く前から
//   30cm 浮いていることになっている」状態から始まってしまう。
//   接地中は毎周期ここを通して0に固定するのが安全。
//   出典: 参照実装の holdPositionVelocity()(接地中は毎周期
//         高度・速度をピン留めする)
//         firmware/vehicle/components/sf_estimator_complementary/
//         complementary_estimator.cpp
void Altitude_reset() {
    _alt_height   = 0.0f;
    _alt_velocity = 0.0f;
    _alt_throttle = 0.0f;
    Pid_reset(&_alt_pid);
}

// 目標高度[m]を設定する
//
// 上向き正。スティックの上下を「高度そのもの」ではなく
// 「目標高度の増減」として入れる想定(呼び出し側で積分する)。
void Altitude_setTarget(float height) { _alt_target = height; }

// 目標高度[m]を取得する
float Altitude_getTarget() { return _alt_target; }

// ホバリングに必要なスロットル[duty]を設定する
//
// 実測で分かったら差し替える。0.0-1.0 の範囲外は無視する。
void Altitude_setHoverThrottle(float duty) {
    const bool is_in_range = (duty >= 0.0f) && (duty <= 1.0f);
    if (!is_in_range) {
        return;
    }
    _alt_hover_throttle = duty;
}

// ホバリングに必要なスロットル[duty]を取得する
float Altitude_getHoverThrottle() { return _alt_hover_throttle; }

// ToFを取り込んで推定値を補正する
//
// Altitude_update() の中から呼ぶ。ドライバが新しい値を持っていたときだけ
// 補正を掛け、そうでなければ何もしない。
//
// Why not 毎周期 Tof_getDistance() の値で無条件に補正する: ドライバは
//   新データが無ければ前回値をそのまま返す(driver_tof.h の Tof_update()
//   の作り)。400Hzで同じ値を13回入れると、その1回ぶんの誤差に
//   13回ぶんのゲインが掛かる。実効ゲインが13倍になって推定が振動する。
//   値が変わった周期だけ補正することで、設計どおり「ToF 1回につき1回」
//   の補正になる。
//
// Why not ドライバ側に「新データが来た」フラグを足す: driver_tof.h は
//   二十九歩で実機確認済みの成果物で、三十歩の都合で触ると
//   そちらの検証をやり直すことになる。値の変化を見るだけで足りる。
//
// Why not 値が変わらない=静止だから補正して構わない、とする: 静止中に
//   同じ値が正しく13回来ているのか、センサーが固まって同じ値を返して
//   いるのかは、ここからは区別できない。区別できないなら、実効ゲインが
//   跳ねない側に倒す。
static void _Altitude_correctWithTof() {
    if (!Tof_isValid()) {
        return;
    }

    const float distance_m = Tof_getDistance();

    // ドライバが値を更新した周期だけ補正する。
    const bool is_new_sample = (distance_m != _alt_prev_tof_distance);
    if (!is_new_sample) {
        return;
    }
    _alt_prev_tof_distance = distance_m;

    // レンジ判定はドライバ(TOF_DISTANCE_MIN / MAX)が済ませているので
    // ここでは繰り返さない。NaN だけは通ってしまうと推定が全て NaN に
    // なるので落とす(NaN は比較が全て false になり is_finite が false)。
    const bool is_finite = isfinite(distance_m);
    if (!is_finite) {
        return;
    }

    const float roll  = Attitude_getRoll();
    const float pitch = Attitude_getPitch();

    // 傾きが大きいとビームが真下の床を外れる。cos補正は「当たっている面が
    // 真下の床と同じ平面」を前提にしているので、外れた時点で補正は嘘になる。
    // 補正を捨てて予測だけで進める(数周期なら加速度積分が持つ)。
    //
    // 鮮度時刻は更新しない。傾いたまま飛び続ければ 200ms で STALE に
    // 落ちて手動スロットルへ降格する。**「補正できていないのに
    // OK を出し続ける」状態を作らない。**
    const float tilt          = sqrtf(roll * roll + pitch * pitch);
    const bool  is_tilt_small = tilt <= ALTITUDE_TOF_MAX_TILT_RAD;
    if (!is_tilt_small) {
        return;
    }

    // --- 傾き補正: 斜距離 → 高度 ----------------------------------
    // ToFは機体に固定されているので、機体が傾けばビームも傾き、
    // 床までの「斜め distance」を測る。真下の高さは
    //   h = d * cos(roll) * cos(pitch)
    // 15度傾けば cos は 0.966、両軸なら 0.933 で、50cm に対して3cm以上の
    // 差になる。ホールドの目標精度(±5cm)と同じ桁なので、補正しないと
    // 「傾けると高度が下がったことになって上昇する」挙動になる。
    //
    // cos は偶関数なので、roll/pitch の符号によらず同じ補正になる。
    // pitch 正=機首上げでも負=機首下げでも、ビームが床から離れる量は同じ。
    // 出典: knowledge/position-hold-design.md「高度(30歩)」
    //       h_tof = d_tof * cos(roll) * cos(pitch)
    const float h_tof = distance_m * cosf(roll) * cosf(pitch);

    // --- 補正: 固定ゲインの2状態相補フィルタ ----------------------
    // e は「ToFが言う高さ」と「加速度で進めてきた高さ」のずれ。
    // 高度には強く(k1)、速度にはやんわり(k2)戻す。
    const float e = h_tof - _alt_height;
    _alt_height += ALTITUDE_K_HEIGHT * e;
    _alt_velocity += ALTITUDE_K_VELOCITY * e;

    _alt_last_tof_us = _Altitude_nowUs();
}

// 高度推定と高度制御の更新
//
// dt[s] は前回の呼び出しからの経過時間。
//
// 呼ぶ順序: Imu_update() → Attitude_update() → Tof_update()
//           → Altitude_update()。
// 姿勢を使って加速度を鉛直方向に投影するので、姿勢が先でなければならない。
//
// 予測(加速度の積分)は 400Hz の制御ループから毎周期呼ぶのが本来だが、
// main.cpp の置き場所は 100Hz の遅いフレームになっている。どちらでも
// 動くよう、周期は dt でしか見ていない。**要実測**(100Hz で積分誤差が
// 許容内かを CSV で確かめる。足りなければ予測だけ 400Hz に上げる)。
void Altitude_update(float dt) {
    // --- 予測: 加速度の鉛直成分を積分 -----------------------------
    //
    // 加速度計が測るのは比力(specific force)で、静止していても重力の
    // 反作用ぶん +1g が出ている(FRDで az ≒ +9.8)。機体の実際の加速度は
    // ここから重力を取り除いたぶん。
    //
    // 姿勢角で機体軸の加速度を鉛直軸へ投影する:
    //   a_down = -ax*sin(pitch) + ay*sin(roll)*cos(pitch)
    //            + az*cos(roll)*cos(pitch)
    // これは機体→NED の回転行列の第3行(下方向の行)そのもの。
    // pitch 正=機首上げのとき、機首側の加速度 ax は「上向き」に傾くので
    // 下向き成分への寄与は負になる。attitude.h の pitch の符号と一致する。
    // ここから重力 g を引くと機体の下向き加速度になり、符号を反転して
    // 上向き正に直す。
    //
    // Why not 完全な回転行列を組んで掛ける: 必要なのは鉛直成分1つだけで、
    //   3x3 の他の8成分は捨てることになる。行を1つだけ書く方が
    //   「どの三角関数がどこから来たか」が読める。
    //
    // Why not 加速度を使わずToFだけで高度を出す: ToFは30Hzで、制御周期
    //   400Hzの13周期に1回しか動かない。周期間を埋める材料が要る。
    float chip_accel[3];
    Imu_getAccel(chip_accel);

    // attitude.h と同じチップ軸→機体軸(FRD)の変換。
    // Why not attitude.h の _Attitude_chipToBody() を呼ぶ: あれは
    //   ファイル内部の実装詳細(_ 始まりの static)で、外から呼ぶ前提の
    //   関数ではない。変換の出典は attitude.h のコメントを参照。
    const float ax = chip_accel[1];   // body.x(前) = chip.y
    const float ay = chip_accel[0];   // body.y(右) = chip.x
    const float az = -chip_accel[2];  // body.z(下) = -chip.z

    const float roll  = Attitude_getRoll();
    const float pitch = Attitude_getPitch();

    const float sin_roll  = sinf(roll);
    const float cos_roll  = cosf(roll);
    const float sin_pitch = sinf(pitch);
    const float cos_pitch = cosf(pitch);

    // 機体軸の比力を鉛直下向きへ投影する。
    // 静置(roll=pitch=0)なら a_down_sf = az ≒ +g になる。
    const float a_down_sf =
        -ax * sin_pitch + ay * sin_roll * cos_pitch + az * cos_roll * cos_pitch;

    // 重力を取り除いて、上向き正に直す。
    // 静置なら a_down_sf - g ≒ 0 なので a_up ≒ 0 で積分器は動かない。
    const float a_up = -(a_down_sf - ALTITUDE_GRAVITY);

    // IMUが NaN を返したら積分器を汚さない。
    //
    // Why not そのまま積分して後段で弾く: 一度 _alt_height が NaN に
    //   なると、以降の全ての比較が false になって範囲チェックもクランプも
    //   すり抜ける。pid.h が入口で NaN を止めているのと同じ理由で、
    //   状態を持つ側が自分の状態を守る。
    const bool is_accel_finite = isfinite(a_up);

    // dt が異常なときは積分しない。pid.h と同じ考え方で、
    // 落ちたフレームの間に何が起きたか分からない区間を
    // 「積分していたことにする」より、その周期を捨てる。
    const bool is_dt_usable = (dt > 0.0f) && (dt <= PID_DT_MAX);

    const bool can_predict = is_dt_usable && is_accel_finite;
    if (can_predict) {
        // 速度を先に進めてから高度に足す(半陰的オイラー)。
        //
        // Why not 高度を先に進める(陽的オイラー): 陽的だと1周期ぶん
        //   古い速度で高度を進めることになり、上下に揺れる入力で
        //   積分が発散する側に誤差が溜まる。半陰的なら誤差が振動して
        //   打ち消し合う。コストは行の順番だけ。
        _alt_velocity += a_up * dt;
        _alt_height += _alt_velocity * dt;
    }

    // --- 補正: ToFが新しい値を持っていれば取り込む ------------------
    _Altitude_correctWithTof();

    // --- ToFの鮮度判定 --------------------------------------------
    //
    // Why not dt を積算して判定する: 制御ループが詰まって周期が崩れると
    //   dt の積算は「時間が進んでいない」ように見える。絶対時刻の差なら
    //   ループが止まっていても次に呼ばれた瞬間に気づく。safety.h と同じ。
    const int64_t now         = _Altitude_nowUs();
    const bool    has_any_tof = (_alt_last_tof_us != 0);
    if (!has_any_tof) {
        _alt_status = ALTITUDE_STATUS_NO_DATA;
    } else {
        const bool is_tof_fresh =
            (now - _alt_last_tof_us) < ALTITUDE_TOF_TIMEOUT_US;
        _alt_status = is_tof_fresh ? ALTITUDE_STATUS_OK : ALTITUDE_STATUS_STALE;
    }

    // --- 制御 ------------------------------------------------------
    //
    // 推定が生きていないときは高度制御を諦める。
    // スロットルは0を返し、呼び出し側が手動スロットルへ降格する。
    //
    // Why not ホバーdutyを出し続けて粘る: ToFが途絶えた原因が
    //   「機体が高く上がりすぎてレンジ外」なのか「センサーが死んだ」なのか、
    //   ここからは区別できない。高度が分からないまま浮かせ続けるより、
    //   操縦者の手に戻す方が結果を予測できる。
    //   降格の設計は knowledge/position-hold-design.md の失陥表
    //   「ToF無効 | 200ms | 高度ホールド→手動スロットルに降格」
    //
    // Why not ここで自動降下する: 高度が信用できない状態で降下させると、
    //   推定が外れていた場合に地面へ加速して突っ込む。safety.h が
    //   リンク断で自律降下を選ばなかったのと同じ理由。
    const bool is_estimate_usable = (_alt_status == ALTITUDE_STATUS_OK);
    if (!is_estimate_usable) {
        // I項を溜めたまま復帰すると、溜まったぶんが一気に出て跳ねる。
        Pid_reset(&_alt_pid);
        _alt_throttle = 0.0f;
        return;
    }

    // 推定が NaN に落ちていたら制御しない。
    //
    // Pid_update() は入力の NaN を弾いてくれるが、自前のD項
    // (-kd * _alt_velocity)にはその守りが無い。ここで一度だけ見る。
    const bool is_estimate_finite =
        isfinite(_alt_height) && isfinite(_alt_velocity);
    if (!is_estimate_finite) {
        // 推定が壊れている。作り直すまで高度ホールドはできない。
        Altitude_reset();
        _alt_status = ALTITUDE_STATUS_STALE;
        return;
    }

    // 高度PID。**D項には推定した vz を使う。**
    //
    // P項とI項は pid.h に任せ、D項だけ自前で作る。
    //
    // Pid_update() のD項は「測定値の微分」なので、測定値に高度 h を渡すと
    // (h[k]-h[k-1])/dt になる。h はToF補正のたびに ALTITUDE_K_HEIGHT * e
    // ぶん飛ぶので、その差分は段差になり、1/dt = 400倍されてモーターに乗る。
    // 一方 vz は推定器が状態として直接持っていて、予測ループで滑らかに動く。
    //
    // Why not Pid_update(&pid, target, height, dt) 一発で済ませる:
    //   上記のとおりD項が h の段差を拾う。ToFの生データを制御に使わない
    //   (このファイル冒頭の Why not)のと同じ理由が、微分にも当てはまる。
    //
    // Why not Pid を使わずP・I項も自前で書く: 積分のクランプ・dt異常時の
    //   扱い・NaN対策を pid.h が既に一箇所で解いている。書き直すと
    //   直すときに2箇所直すことになる。
    //
    // Pid の使い方: kd=0 で初期化してあり(Altitude_init)、目標に高度を、
    // 測定値に推定高度を渡す。kd=0 なので内部のD項は0のまま動かない。
    //
    // Why not 誤差を target に・0 を measured に渡す: 誤差を渡すと
    //   Pid_getPTerm() 等を読んだときに「何と何の差だったか」が
    //   分からなくなる。目標と測定をそのまま渡せば、ログに出す値が
    //   他のPIDと同じ意味を持つ。
    const float pi_out = Pid_update(&_alt_pid, _alt_target, _alt_height, dt);

    // D項: 推定した昇降速度から。上昇中(vz>0)なら出力を下げる向き。
    //
    // Why not ToFの差分から vz を作る: 30Hz・数mm量子化の差分は
    //   1/dt 倍されると D項がノイズだけで埋まる。推定器が加速度で
    //   埋めた vz は予測ループで滑らかに動く。これが「高度推定を作った
    //   最大の理由」でもある。
    //
    // Why not pid.h の一次遅れフィルタを通す: あのフィルタは
    //   Pid の内部状態として持っていて、上のとおりD項を使わない使い方を
    //   しているので通せない。vz はToF補正のたびに
    //   ALTITUDE_K_VELOCITY * e の段差を持つが、k2=0.05 と小さいので
    //   段差も小さい。**要実測**(CSVで vz とD項を見て、必要なら
    //   ここに一次遅れを1つ足す)。
    const float d_out =
        _Altitude_clamp(-ALTITUDE_PID_KD * _alt_velocity, ALTITUDE_PID_D_LIMIT);

    // ホバー分 + 増減。全体を 0.0-1.0 に収める。
    // 上限で切っても、ミキサー側が MOTOR_DUTY_MAX(0.8) でさらに切る。
    float throttle = _alt_hover_throttle + pi_out + d_out;
    if (throttle < 0.0f) {
        throttle = 0.0f;
    }
    if (throttle > 1.0f) {
        throttle = 1.0f;
    }
    _alt_throttle = throttle;
}

// 推定高度[m]を取得する。上向き正、地面が0。
float Altitude_getHeight() { return _alt_height; }

// 推定昇降速度[m/s]を取得する。上昇が正。
//
// 高度PIDのD項に使っているのと同じ値。CSVログに出して、
// ToF補正のたびに段差が出ていないかを見るのにも使う。
float Altitude_getVelocity() { return _alt_velocity; }

// 高度制御の出力スロットル[duty 0.0-1.0]を取得する
//
// Mixer_update() の throttle にそのまま渡せる。
// Altitude_getStatus() が ALTITUDE_STATUS_OK でないときは 0 が返るので、
// 呼び出し側は状態を見て手動スロットルに切り替えること。
//
// なお、モーターへ実際に書くのは main.cpp の Safety_isArmed() の中だけ。
// この値が非0でも、ディスアーム中は Mixer_stop() しか通らない。
float Altitude_getThrottle() { return _alt_throttle; }

// 高度推定の状態を取得する
//
// ALTITUDE_STATUS_OK 以外のとき、Altitude_getThrottle() は 0 を返す。
// 呼び出し側はこれを見て高度ホールドを諦め、手動スロットルへ降格する。
AltitudeStatus Altitude_getStatus() { return _alt_status; }

// 高度推定の状態を人が読める文字列にする(シリアル出力用)
const char* Altitude_getStatusName(AltitudeStatus status) {
    switch (status) {
        case ALTITUDE_STATUS_NO_DATA:
            return "NO_DATA";
        case ALTITUDE_STATUS_OK:
            return "OK";
        case ALTITUDE_STATUS_STALE:
            return "STALE";
        default:
            return "UNKNOWN";
    }
}
