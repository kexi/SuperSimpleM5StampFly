#pragma once

// 汎用PID制御器
//
// 姿勢制御(角速度・角度)、高度制御、速度制御が全てこれを使う。
// カスケードの各段で「目標値と測定値の差から1つの操作量を作る」という
// 計算は同じなので、段ごとに書き分けない。
// カスケード構造は knowledge/position-hold-design.md を参照。
//
// ゲインはここに埋め込まない。同じ式でも角速度[rad/s]と高度[m]では
// 単位も適正値も違うので、決められるのは呼び出し側だけ。
//
// Why not クラス + メンバ関数: 既存の driver_*.h が全て
//   構造体 + Xxx_init / Xxx_update の C 風スタイルで揃っている。
//   ここだけ C++ の作法にすると読む側が二つの流儀を覚えることになる。
//
// Why not 参照実装(StampFly Ecosystem)の Ti/Td 形式: あちらは
//   C(s) = Kp(1 + 1/(Ti·s) + Td·s/(η·Td·s+1)) の「時定数」表現で、
//   Kp を動かすと I も D も一緒に動く。教材としては ki/kd を直接置く
//   並列形の方が「どの項がどれだけ効いたか」を見たまま追える。
//   ただし**微分の一次遅れフィルタだけは真似る**(理由は D項の項を参照)。
//   出典: StampFly Ecosystem (MIT License, Copyright (c) 2026 Kouhei Ito)
//         firmware/vehicle/components/sf_controller_pid/include/pid.hpp
//         firmware/vehicle_old/components/sf_algo_pid/include/pid.hpp
//         https://github.com/M5Fly-kanazawa/stampfly_ecosystem

#include <math.h>  // isfinite

// D項フィルタの既定のカットオフ周波数[Hz] -------------------
//
// 生の差分 (y[k]-y[k-1])/dt は 1/dt = 400 倍の利得を持つ純粋な微分器で、
// センサーノイズをそのまま400倍する。角速度ループでは入力が既にジャイロの
// 生値なので、それを微分すると「角加速度」になり、機体の弾性振動と
// モーター振動が最も出る帯域を拾う。フィルタ無しではD項がノイズだけで
// 埋まり、ゲインを上げられない。
//
// 30Hz は「姿勢制御の帯域(〜10Hz程度)より十分高く、ナイキスト(200Hz)より
// 十分低い」という置き方。参照実装の η=0.125 は D の折れ点を 1/(η·Td) に
// 置く形で、Td=0.02s なら 400rad/s ≒ 64Hz に相当する。同じ桁に収まる。
// **要実測**(ホバリング中のD項をCSVに出し、機体の振動が乗っていないか見る)。
// 出典: 参照実装の既定値 η=0.125
//       firmware/vehicle/components/sf_controller_pid/include/pid.hpp
#define PID_D_CUTOFF_HZ_DEFAULT 30.0f

// dt の上限[s] ---------------------------------------------
//
// これを超える dt が来たら、その周期は積分・微分を進めない(下記 Pid_update)。
// 400Hz(2.5ms)の40倍 = 0.1s。正常に回っていれば決して届かない値で、
// 届いたということは制御周期が守れていないということ。
// **要実測**(二十四歩の負荷測定で実際のジッタ幅を見て決め直す)。
#define PID_DT_MAX 0.1f

// PID制御器の状態
//
// ゲインと制限値、そして呼び出しをまたいで持ち越す内部状態を持つ。
// インスタンスは制御軸ごとに1つ用意する(ロール角速度、ピッチ角速度、…)。
struct Pid {
    // ゲイン(呼び出し側が Pid_init で与える)
    float kp;  // 比例ゲイン
    float ki;  // 積分ゲイン[1/s]。誤差に dt を掛けて積むのでこの単位
    float kd;  // 微分ゲイン[s]

    // 制限値
    float i_limit;    // I項の絶対値上限(ワインドアップ防止)
    float out_limit;  // 出力の絶対値上限

    // D項の一次遅れフィルタのカットオフ[Hz]。0以下でフィルタ無効(生の微分)。
    float d_cutoff_hz;

    // 内部状態(Pid_reset でゼロに戻す)
    float integral;       // I項の累積値(ki を掛けた後の値)
    float prev_measured;  // 前回の測定値。D項の差分に使う
    float d_filtered;     // フィルタ通過後の微分値[measured の単位/s]
    bool  has_prev;       // prev_measured が有効か(初回の微分キック防止)

    // 直近の各項(ログ・チューニング用。制御には使わない)
    float p_term;
    float i_term;
    float d_term;
};

// 値を ±limit に収める
//
// limit が 0 以下なら制限なしとして素通しする。
// Why not 常にクランプする: 「上限を決めていない」を表す値が要る。
//   0 を「常に0を出力」と解釈すると、ゲインだけ設定して制限を書き忘れた
//   ときに制御器が黙って死ぬ。
static float _Pid_clamp(float value, float limit) {
    const bool is_unlimited = (limit <= 0.0f);
    if (is_unlimited) {
        return value;
    }
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

// PID制御器の初期化
//
// i_limit / out_limit は絶対値の上限。0以下を渡すと制限なしになる。
// D項のフィルタは PID_D_CUTOFF_HZ_DEFAULT で入る。変えたい軸だけ
// Pid_setDCutoff() で上書きする。
//
// Why not カットオフも引数に足す: 引数が6個を超えると呼び出し側が
//   位置を数えることになり、ゲインの並びを取り違えても型では気づけない。
//   既定で妥当に動き、必要な軸だけ後から変える形にする。
void Pid_init(Pid* pid, float kp, float ki, float kd, float i_limit,
              float out_limit) {
    if (pid == nullptr) {
        return;
    }

    pid->kp        = kp;
    pid->ki        = ki;
    pid->kd        = kd;
    pid->i_limit   = i_limit;
    pid->out_limit = out_limit;

    pid->d_cutoff_hz = PID_D_CUTOFF_HZ_DEFAULT;

    pid->integral      = 0.0f;
    pid->prev_measured = 0.0f;
    pid->d_filtered    = 0.0f;
    pid->has_prev      = false;

    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
}

// D項フィルタのカットオフ[Hz]を変える。0以下でフィルタ無効。
//
// 外側のループ(高度・速度・位置)は元の信号が既に推定器を通っていて
// 滑らかなので、内側ほど強くフィルタする必要がない。
void Pid_setDCutoff(Pid* pid, float cutoff_hz) {
    if (pid == nullptr) {
        return;
    }
    pid->d_cutoff_hz = cutoff_hz;
}

// 内部状態のリセット
//
// I項の累積・前回測定値・微分フィルタの状態をゼロに戻す。
// ゲインと制限値、カットオフは保持する。
//
// ディスアーム時に必ず呼ぶ。地上で機体を押さえている間、姿勢誤差は
// 消えないまま積み続けるので、そのままアームすると溜まったI項が
// 一気に出て機体が跳ねる。
void Pid_reset(Pid* pid) {
    if (pid == nullptr) {
        return;
    }

    pid->integral      = 0.0f;
    pid->prev_measured = 0.0f;
    pid->d_filtered    = 0.0f;
    pid->has_prev      = false;

    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
}

// PIDの更新。目標値と測定値から操作量を1つ返す。
//
// dt は前回呼び出しからの経過時間[s]。400Hzループなら 0.0025。
// 呼び出し側が実測値を渡してもよい(Timer_getElapsedTime を使う)。
//
// D項は「測定値の微分」で計算する。誤差 (target - measured) の微分にすると、
// 目標値がステップで飛んだ瞬間に微分が発散し、出力が一瞬跳ねる
// (微分キック)。スティック操作や外側ループの切り替えで目標値は普通に飛ぶ。
// 測定値の微分なら目標値の変化はD項に現れない。
//
// 符号: d(error)/dt = -d(measured)/dt (目標値一定のとき)なので、
// 測定値の差分にはマイナスを付ける。
float Pid_update(Pid* pid, float target, float measured, float dt) {
    if (pid == nullptr) {
        return 0.0f;
    }

    // NaN/Inf を内部状態に入れない。
    //
    // I項もフィルタ状態も前回値を持ち越すので、一度 NaN が入ると
    // Pid_reset するまで出力が NaN のままになる。NaN は比較が全て false に
    // なるためクランプもすり抜け、ミキサーまで素通りする。
    // センサー異常時に「動かない」ではなく「止まらない」側に倒れるのを防ぐ。
    //
    // Why not 呼び出し側で弾く: 弾き忘れた1箇所が機体を落とす。
    //   状態を持つ側が自分の状態を守る。
    const bool is_input_finite = isfinite(target) && isfinite(measured);
    if (!is_input_finite) {
        return _Pid_clamp(pid->p_term + pid->i_term + pid->d_term,
                          pid->out_limit);
    }

    const float error = target - measured;

    // P項
    pid->p_term = pid->kp * error;

    // dt が異常なときは積分・微分を進めず、P項と保持しているI項だけで返す。
    //
    // dt <= 0 : 同一フレームで二度呼ばれた場合。0除算になる。
    // dt が大きすぎる: フレーム落ちや初回呼び出しで、この dt をそのまま
    //   使うと I項が一度に大量に積まれ、微分も長時間の平均になる。
    //   1周期(2.5ms)の40倍=0.1s を上限にする。400Hz が守れていれば
    //   決して超えない値で、超えたときは制御より復帰を優先する状況。
    //   **要実測**(二十四歩の負荷測定で実際のジッタ幅を見て決め直す)。
    //
    // Why not 大きい dt をクランプして進める: 落ちたフレームの間に機体が
    //   何をしていたかは分からない。分からない区間を「制御していたことに
    //   する」より、その周期を捨てて次の正常な周期から再開する方が安全。
    const bool is_dt_usable = (dt > 0.0f) && (dt <= PID_DT_MAX);
    if (!is_dt_usable) {
        pid->d_term = 0.0f;
        // 次の周期で巨大な差分が出ないよう、測定値だけは追従させておく。
        pid->prev_measured = measured;
        pid->has_prev      = true;
        return _Pid_clamp(pid->p_term + pid->i_term, pid->out_limit);
    }

    // I項。積んでからクランプする。
    //
    // Why not クランプ前の値を保持して後で切る: 制限を超えた分まで内部に
    //   溜めると、誤差の符号が反転しても「溜まった分を吐き出す」まで
    //   出力が戻らない。実際に使える範囲だけを持つ。
    //
    // Why not 参照実装のバックカルキュレーション: 出力の飽和量をI項に
    //   戻す方式で、確かにこちらの方が飽和からの復帰は滑らか。ただし
    //   追従時定数 Tt という調整項目がもう1つ増える。i_limit による
    //   単純なクランプでも、上限に張り付いた分は積まれないので
    //   ワインドアップは止まる。教材としては項目が少ない方を採る。
    pid->integral += pid->ki * error * dt;
    pid->integral = _Pid_clamp(pid->integral, pid->i_limit);
    pid->i_term   = pid->integral;

    // D項(測定値の微分に一次遅れフィルタ)。
    //
    // 初回は前回値が無いので0にする。prev_measured の初期値0との差分を
    // 取ると、測定値の大きさそのものが微分として出てしまう。
    if (!pid->has_prev) {
        pid->d_filtered    = 0.0f;
        pid->d_term        = 0.0f;
        pid->prev_measured = measured;
        pid->has_prev      = true;
    } else {
        const float measured_rate = (measured - pid->prev_measured) / dt;
        pid->prev_measured        = measured;

        // 一次遅れ(RCローパス)を離散化した形。
        //   a = dt / (RC + dt),  RC = 1/(2π·fc)
        // a が1に近いほど素通し、0に近いほど強く鈍る。
        // dt から毎回計算するので、周期が揺れてもカットオフは保たれる。
        //
        // Why not 移動平均: 同じ遅れを得るのに過去N点の配列が要り、
        //   軸の数だけメモリを持つ。一次遅れは状態1個で済む。
        // Why not 参照実装の双一次変換(Tustin): 折れ点の精度は上がるが、
        //   fc がナイキストに対して十分低い今の使い方では差が出ない。
        const bool is_filter_enabled = (pid->d_cutoff_hz > 0.0f);
        if (!is_filter_enabled) {
            pid->d_filtered = measured_rate;
        } else {
            const float rc = 1.0f / (2.0f * (float)M_PI * pid->d_cutoff_hz);
            const float a  = dt / (rc + dt);
            pid->d_filtered += a * (measured_rate - pid->d_filtered);
        }

        pid->d_term = -pid->kd * pid->d_filtered;
    }

    // 出力もクランプする。
    // クランプしないと、飽和した操作量がミキサーで他軸を押しつぶす。
    const float output = pid->p_term + pid->i_term + pid->d_term;
    return _Pid_clamp(output, pid->out_limit);
}

// 直近のP項を取得する(チューニング時のログ用)
float Pid_getPTerm(const Pid* pid) {
    return (pid == nullptr) ? 0.0f : pid->p_term;
}

// 直近のI項を取得する(チューニング時のログ用)
float Pid_getITerm(const Pid* pid) {
    return (pid == nullptr) ? 0.0f : pid->i_term;
}

// 直近のD項を取得する(チューニング時のログ用)
float Pid_getDTerm(const Pid* pid) {
    return (pid == nullptr) ? 0.0f : pid->d_term;
}
