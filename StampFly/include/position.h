#pragma once

// 水平速度推定と位置制御(ポジションホールド)
//
// Optical Flow(PMW3901)の移動量[px]と高度[m]から機体の水平速度[m/s]を作り、
// 「その場に留まる」ための目標ロール角・目標ピッチ角[rad]を出す。
// カスケードの最も外側で、出力は角度制御(姿勢PID)の目標値になる。
//
//   位置P → 速度PI → (ここまでが position.h) → 角度P → 角速度PID → ミキサー
//
// 座標系は FRD(x:前 / y:右 / z:下)。速度・位置は機体固定座標のまま持つ。
// 設計の出典: knowledge/position-hold-design.md「速度(32歩)」
//             knowledge/stampfly-ecosystem-reference.md「速度変換の手順」
//
// Why not 推定と制御を別ファイルに分ける: この2つは「Flowが有効な間だけ
//   意味がある」という同じ条件で生き死にする。無効になったら速度も位置も
//   凍結し、目標角も0に落とす、という判断が1箇所に無いと、
//   「速度は凍結したが目標角は前の値のまま出続ける」ような穴ができる。
//   position-hold-design.md の「200行を超えたら分ける」の範囲内。
//
// 前提: 呼び出し側が Flow_update() を呼んでから Position_update(dt) を呼ぶ。
//   dt は Flow_update() の間隔(100Hz なら 0.01)であって、制御周期ではない。
//   Flow の移動量は「前回の読み出しからの累積」なので、積分区間は
//   読み出し間隔そのものになる(driver_flow.h の Flow_update() のコメント)。
//
// **このファイルはモーターに触らない。** 出す値は目標角[rad]だけで、
//   それを使うかどうかは呼び出し側が Safety_isArmed() を見て決める
//   (main.cpp の「モーターへ書く経路は if (Safety_isArmed()) だけ」)。
//   ただし推定は常に走るので、アーム前に位置誤差が溜まらないよう
//   Position_notifyArmed() で原点を引き直す。下の Position_notifyArmed()
//   を参照。

#include <math.h>
#include <stdint.h>

#include "attitude.h"     // Attitude_getRollRate / getPitchRate [rad/s]
#include "driver_flow.h"  // Flow_getDeltaX / getDeltaY / getSqual / hasMoved
#include "pid.h"

// ピクセル → 角度の変換係数[rad/px] --------------
//
// PMW3901 の 1 ピクセルが視野角の何 rad に相当するか。
// 出典: StampFly Ecosystem (MIT License, Copyright (c) 2026 Kouhei Ito)
//       firmware/vehicle_old/components/sf_algo_eskf/include/eskf.hpp
//       cfg.flow_rad_per_pixel = 0.00222f
//       https://github.com/M5Fly-kanazawa/stampfly_ecosystem
//
// Why not 視野角÷画素数で概算する: 一度それをやって 0.02 rad/px という
//   9倍ずれた値を置いた(knowledge/position-hold-design.md の訂正節)。
//   視野角の値そのものを確認しないまま割り算しても、条件の無い概算は
//   条件が違えば嘘になる。実測値のある参照実装に合わせる。
// **要実測**(既知の距離を一定高度で滑らせ、積分値が実距離に合うか見る)。
#define POSITION_FLOW_RAD_PER_PIXEL 0.00222f

// カメラ座標 → 機体座標のスケール補正 -------------
//
// 参照実装の cam_to_body は 2x2 行列だが、非対角は 0 なので軸の入れ替えは
// 起きない。ここでは軸ごとのスケール係数2つだけを持つ。
// 取り付け角のわずかなずれと、レンズ・センサーの軸ごとのスケール誤差を
// まとめて吸収するための係数。
// 出典: 上記 eskf.hpp の cfg.flow_cam_to_body = {0.943, 0, 0, 1.015}
//
// x と y で 7% 違うぶんは「同じ速度で流れているのに軸によって推定値が
// 違う」という形で残り、位置制御が斜めに寄る癖になる。定数2つで消せる。
//
// Why not 単位行列にして無視する: 上記の癖は「なぜか一方向にだけ寄る」
//   という、原因の当たりを付けにくい症状で出る。既に実測された数字が
//   あるなら、最初から入れておく方が切り分けが減る。
// Why not 非対角も持てるように 2x2 行列にする: 参照実装でも非対角は 0 で、
//   軸の入れ替えは無い。使わない自由度を持つと、実測で詰めるときに
//   触る場所が増える。取り付け向きが変わったら足す。
//
// **重要**: これは**参照実装の機体・レンズでの実測値**であって、
//   この機体のものではない。三十一歩で Δx / Δy の符号と大きさを実測する
//   まで、この2つは「参照実装ではこうだった」以上の意味を持たない。
//   **要実測**(knowledge/stampfly-ecosystem-reference.md「注意」)。
#define POSITION_FLOW_CAM_TO_BODY_XX 0.943f
#define POSITION_FLOW_CAM_TO_BODY_YY 1.015f

// Flow を信じる条件 ------------------------------
//
// 高度がこれ未満だと、床が近すぎてピントが合わず、また h を掛ける式の
// h が小さすぎて速度がほぼ 0 に潰れる。
// 出典: 上記 eskf.hpp の cfg.flow_min_height = 0.02f
#define POSITION_FLOW_MIN_HEIGHT 0.02f

// 検出品質(SQUAL)の下限。これ未満のフレームは床の模様を掴めていない。
// 出典: 上記 eskf.hpp の SQUAL 最小閾値 0x19
//       driver_flow.h の FLOW_SQUAL_MIN と同じ値
//
// Why not driver_flow.h の FLOW_SQUAL_MIN をそのまま使う: あちらは
//   「ドライバが値を捨てるか」の閾値で、こちらは「制御に使ってよいか」の
//   閾値。同じ値から始めるが、実測で別々に動かしたくなる可能性がある。
//   同じ数字でも意味が違うものは別の名前で持つ。
#define POSITION_FLOW_SQUAL_MIN 0x19

// 高度[m]の鮮度[us] ------------------------------
//
// Position_setHeight() が最後に呼ばれてからこれを超えたら、高度は
// 信用できないとみなして Flow を無効にする。
// 100Hz(10ms)に対して10回分の猶予。
//
// Why not 高度の鮮度を見ない(古い値を使い続ける): 速度は
//   「見かけの角速度 × h」で作るので、h が古いと**速度そのものが丸ごと
//   間違った倍率になる**。高度ホールドが失陥して降格しても、こちらが
//   古い h を持ち続ければ位置制御は何事もなかったように動き続ける。
//   altitude.h が ToF に対して同じことをしている(ALTITUDE_TOF_TIMEOUT_US)。
// 出典: knowledge/position-hold-design.md の失陥表と同じ考え方
#define POSITION_HEIGHT_TIMEOUT_US 100000  // 100ms

// 動きが無い状態を許す時間[us] -------------------
//
// Flow_hasMoved() が false のフレームが続いてよい上限。
//
// **なぜこの判定が要るか。** driver_flow.h の Flow_update() は、モーション
// ビットが立っていないとき Δx / Δy に **0 を入れる**。この 0 は
// 「動いていない」ではなく「センサーが動きを報告していない」であって、
// 本当に静止しているのか、センサーが止まった・床を見失ったのかを
// 区別していない。これを速度0の観測として扱うと、機体が流されている間も
// 推定速度が0に張り付き、位置積分は止まったまま、制御器は
// 「完璧にホバリングできている」と判断して何もしない。
//
// Why not Flow_hasMoved() が false の周期を即座に無効にする: 静止に近い
//   ホバリングでは、モーションビットが立たない周期が普通に混ざる。
//   1周期でも欠けたら降格する作りにすると、最も安定している状態で
//   最も頻繁に降格する。数周期の空白は「本当に動いていない」として通す。
// **要実測**(ホバリング中に hasMoved が連続で false になる長さを見る)。
#define POSITION_FLOW_STALL_TIMEOUT_US 100000  // 100ms

// 1回の更新で許す移動量[px]の上限 ----------------
//
// これを超えたフレームは外れ値として捨てる。PMW3901 は床の切れ目や
// 反射で、実際には動いていないのに大きな値を返すことがある。
// 100Hz・高度0.5m で 100px は 0.00222*100*100*0.5 = 11m/s に相当し、
// 室内ホバリングでは決して出ない。
//
// Why not 速度の側でクランプする: クランプすると「上限に張り付いた速度」が
//   正しい観測として位置積分に入り続ける。外れ値は無かったことにする方が、
//   一瞬の跳ねを位置に残さない。
// **要実測**(ホバリング中の Δpx の分布を CSV に出して決め直す)。
#define POSITION_FLOW_DELTA_MAX 100.0f

// 速度の一次遅れフィルタのカットオフ[Hz] ----------
//
// Δpx は整数量子化されているうえ、100Hz では 1px の差が
// 0.00222*100*h [m/s](h=0.5m なら 0.11m/s)という粗さになる。
// このまま速度PIのP項に入れるとモーターが常にガタつく。
//
// 5Hz は「ホバリングの水平ドリフト(〜1Hz)より十分速く、量子化ノイズは
// 鈍らせる」という置き方。位置ループの帯域(〜0.5Hz)にも干渉しない。
//
// Why not Δpx を数フレーム移動平均する: 遅れは同じだけ入るのに、
//   軸ごとに配列を持つ必要がある。一次遅れなら状態1個で済む
//   (pid.h の D項フィルタと同じ理由)。
// **要実測**(速度の生値とフィルタ後を並べてログに出す)。
#define POSITION_VEL_CUTOFF_HZ 5.0f

// 目標角のクランプ[rad] --------------------------
//
// 位置・速度ループが出せる目標ロール/ピッチ角の上限。
// 出典: knowledge/position-hold-design.md「位置/速度ループの目標角は ±10°」
//
// **これは安全装置。** ゲインを1桁打ち間違えても、機体は10度までしか
// 傾かない。10度の傾きなら水平加速度は g*tan(10°) ≒ 1.7m/s^2 で、
// 1m 離れる前に人が気づいてディスアームできる。
// クランプが無いと、Flow の外れ値1つで機体が45度傾いて壁へ飛ぶ。
//
// 注: safety.h の SAFETY_ARM_TILT_MAX_RAD(アーム時の水平判定)と同じ
//     10度だが別物。あちらは「アームしてよい姿勢か」、こちらは
//     「制御が要求してよい姿勢か」。値が一致しているのは偶然。
#define POSITION_ANGLE_LIMIT_RAD (10.0f * 0.017453293f)

// 速度目標のクランプ[m/s] ------------------------
//
// 位置ループの出力(=速度目標)と、操縦者からの速度指令の両方に掛ける上限。
// 出典: knowledge/position-hold-design.md「速度 ±0.3m/s」
//
// 0.3m/s は「1m の部屋を横切るのに3秒以上かかる」速さ。室内で人が
// 追いつける速度に抑える。
#define POSITION_VEL_LIMIT 0.3f

// ホールドとみなす速度目標のしきい値[m/s] ---------
//
// 速度目標の絶対値がこれ未満なら「スティック中立 = ホールドしたい」と
// 解釈して位置ループを入れる。
//
// Why not 厳密に 0.0f と比較する: スティックの生値は中立でも
//   ±1 カウント程度揺れ、キャリブレーションのオフセットも乗る。
//   それがそのまま速度目標に入ると、中立に戻したつもりでも
//   位置ループが入らず、機体が延々と微速で流れ続ける。
// **要実測**(中立時に Position_setVelTarget へ来る値の幅を見て決める)。
#define POSITION_VEL_HOLD_EPSILON 0.01f

// PIDゲインの初期値 ------------------------------
//
// **すべて要実測。** 三十三歩(速度制御)→三十四歩(位置制御)の順に、
// 外側を切ったまま内側だけを詰める。
//
// 速度PI: 誤差[m/s] → 目標角[rad]。
//   0.3m/s の誤差に対して P だけで 0.15rad(8.6度)出る大きさから始める。
//   → kp = 0.15 / 0.3 = 0.5 [rad/(m/s)]
//   I は定常的な重心ずれ・機体の歪みによる片流れを消すためだけに入れる。
//   ki を大きくすると、位置ループと二重積分になって周期の長い揺れになる。
//
// Why not 速度にも D を入れる: 速度の微分は加速度で、それは既に
//   Flow の量子化ノイズを微分することになる。姿勢の内側ループが
//   十分速い(400Hz)ので、外側の減衰はPだけでも足りる想定。
//
// 位置P: 誤差[m] → 速度目標[m/s]。
//   0.3m ずれたら上限の 0.3m/s で戻る大きさ。→ kp = 1.0 [1/s]
//   これは時定数1秒の一次遅れで原点へ戻る形になる。
//
// Why not 位置にも I を入れる: 位置の定常偏差は速度PIのI項が既に
//   吸収している。二重に積むと、どちらのI項が溜まっているのか
//   ログを見ても分からなくなる。
#define POSITION_VEL_KP      0.5f
#define POSITION_VEL_KI      0.2f
#define POSITION_VEL_KD      0.0f
#define POSITION_VEL_I_LIMIT (POSITION_ANGLE_LIMIT_RAD * 0.5f)

#define POSITION_POS_KP 1.0f
#define POSITION_POS_KI 0.0f
#define POSITION_POS_KD 0.0f

// 位置ループのI項上限。
//
// ki = 0 なので I項は積まれないが、**0 は渡さない**。pid.h の
// _Pid_clamp() は「limit <= 0 なら制限なし」と解釈するので、0 は
// 「制限しない」の意味になる。後で ki を入れたときに、上限の無い
// 積分器が黙って出来上がるのを避ける。
// 出典: pid.h の _Pid_clamp() の Why not
#define POSITION_POS_I_LIMIT POSITION_VEL_LIMIT

// 速度制御器(x:前後 → pitch / y:左右 → roll)
static Pid _pos_vel_pid_x;
static Pid _pos_vel_pid_y;

// 位置制御器
static Pid _pos_pos_pid_x;
static Pid _pos_pos_pid_y;

// 推定値(すべて機体固定座標 FRD) ----------------
static float _pos_vel_x = 0.0f;  // 前方向速度[m/s]
static float _pos_vel_y = 0.0f;  // 右方向速度[m/s]
static float _pos_x     = 0.0f;  // 前方向の移動量[m]
static float _pos_y     = 0.0f;  // 右方向の移動量[m]

// 操縦者/上位からの速度目標[m/s]。中立(0,0)がポジションホールド。
static float _pos_vel_target_x = 0.0f;
static float _pos_vel_target_y = 0.0f;

// 出力(目標姿勢角[rad])
static float _pos_roll_target  = 0.0f;
static float _pos_pitch_target = 0.0f;

// 直近の更新で Flow を信用できたか
static bool _pos_is_valid = false;

// 高度[m]。三十歩(高度推定)の値を Position_setHeight() で受け取る。
//
// Why not altitude.h を include して Altitude_getHeight() を直接呼ぶ:
//   三十二歩(速度推定)の検証は机上でやる歩で、そこに高度推定を丸ごと
//   引き込むと、Flow の変換だけを確かめたいのに ToF と加速度積分まで
//   動かすことになる。高度は「誰かが入れる数値」として受け取り、
//   依存の向きを作らない。
//   代わりに**鮮度と有効性は自分で持つ**(下の2つ)。渡された値を
//   無条件に信じると、呼び出し側が入れ忘れた瞬間に気づけない。
static float _pos_height = 0.0f;

// 高度を最後に受け取った時刻[us]。0 は「まだ一度も来ていない」。
static int64_t _pos_last_height_us = 0;

// Flow_hasMoved() が最後に true だった時刻[us]。0 は「まだ一度も無い」。
static int64_t _pos_last_motion_us = 0;

// 現在時刻[us]を取る。SILSのホストテストでは esp_timer が無いので分ける。
// altitude.h / safety.h と同じ作法。
static int64_t _Position_nowUs() {
#ifdef SF_HOST_TEST
    extern int64_t PositionTest_nowUs();  // テスト側が時刻を進める
    return PositionTest_nowUs();
#else
    return esp_timer_get_time();
#endif
}

// 値を ±limit に収める
//
// limit が 0 以下でも常にクランプする。**pid.h の _Pid_clamp() とは
// 意味が違う。**
//
// Why not pid.h と同じ「0以下なら制限なし」にする: あちらは呼び出し側が
//   ゲインと一緒に制限値を組み立てるので「制限を決めていない」を表す値が
//   要る。こちらの limit は全てこのファイル内の定数(角度10度・速度0.3m/s)で、
//   どれも安全のために必ず効いてほしい上限。同じ名前の関数が隣のファイルと
//   逆の挙動をするのは危ないので、この違いをここに書いておく。
static float _Position_clamp(float value, float limit) {
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

// 推定値と制御器の内部状態を初期状態に戻す
//
// アーム時・Flow が無効になったとき・モード切替時に呼ぶ。
// 位置の原点は「呼んだ場所」になる。
//
// Why not 位置だけ残して速度をゼロにする: Flow が無効だった間に
//   機体は動いている。その間の位置は分からないので、分からない位置を
//   基準に戻ろうとするのが最も危険。原点を引き直す。
//
// 高度と鮮度のタイムスタンプは消さない。あれは外から来る観測の状態で、
// 制御器の内部状態ではない。ここで消すと、リセットのたびに
// 「高度がまだ来ていない」状態に戻って1周期ぶん無効になる。
void Position_reset() {
    _pos_vel_x = 0.0f;
    _pos_vel_y = 0.0f;
    _pos_x     = 0.0f;
    _pos_y     = 0.0f;

    _pos_vel_target_x = 0.0f;
    _pos_vel_target_y = 0.0f;

    _pos_roll_target  = 0.0f;
    _pos_pitch_target = 0.0f;

    _pos_is_valid = false;

    Pid_reset(&_pos_vel_pid_x);
    Pid_reset(&_pos_vel_pid_y);
    Pid_reset(&_pos_pos_pid_x);
    Pid_reset(&_pos_pos_pid_y);
}

// 位置制御の初期化
//
// Flow_init() が成功した後に呼ぶ。ゲインはここで入れる。
void Position_init() {
    Pid_init(&_pos_vel_pid_x, POSITION_VEL_KP, POSITION_VEL_KI, POSITION_VEL_KD,
             POSITION_VEL_I_LIMIT, POSITION_ANGLE_LIMIT_RAD);
    Pid_init(&_pos_vel_pid_y, POSITION_VEL_KP, POSITION_VEL_KI, POSITION_VEL_KD,
             POSITION_VEL_I_LIMIT, POSITION_ANGLE_LIMIT_RAD);

    Pid_init(&_pos_pos_pid_x, POSITION_POS_KP, POSITION_POS_KI, POSITION_POS_KD,
             POSITION_POS_I_LIMIT, POSITION_VEL_LIMIT);
    Pid_init(&_pos_pos_pid_y, POSITION_POS_KP, POSITION_POS_KI, POSITION_POS_KD,
             POSITION_POS_I_LIMIT, POSITION_VEL_LIMIT);

    // 外側のループは元の信号が既に一次遅れを通っていて滑らかなので、
    // D項フィルタは要らない(そもそも kd = 0)。
    Pid_setDCutoff(&_pos_vel_pid_x, 0.0f);
    Pid_setDCutoff(&_pos_vel_pid_y, 0.0f);

    _pos_height         = 0.0f;
    _pos_last_height_us = 0;
    _pos_last_motion_us = 0;

    Position_reset();
}

// アームした瞬間に呼ぶ
//
// 位置の原点を「今いる場所」に引き直す。
//
// **これを呼ばないと、アームの瞬間に機体が傾く。** 推定はアーム前から
// 走っていて、機体を手で持ち運んだぶんが _pos_x / _pos_y に積まれている。
// その状態でアームすると、位置ループが「持ち運ばれた距離ぶん戻ろう」として
// 上限いっぱいの目標角を出す。
//
// Why not アーム中かどうかをこのファイルが safety.h に聞く: 推定は
//   アームと無関係に走らせたい(離陸前に速度が出ているか CSV で見たい)。
//   状態を持ち込まず、切り替わった瞬間だけを通知として受ける。
//   safety.h も同じ形で Safety_notifyImuUpdate() を外から受けている。
void Position_notifyArmed() { Position_reset(); }

// 高度[m]を渡す
//
// 三十歩の高度推定(Altitude_getHeight())の値を毎周期入れる。
// ToF の生値ではなく推定値を使う。生値は 30Hz・量子化つきで、
// これを速度に掛けると速度そのものが 30Hz の階段になる。
//
// **高度推定が信用できないときは呼ばない。** altitude.h なら
// Altitude_getStatus() == ALTITUDE_STATUS_OK のときだけ呼ぶ。
// 呼ばれない状態が POSITION_HEIGHT_TIMEOUT_US 続けば、
// このファイルは自分で Flow を無効にする。
//
// Why not 「無効」を表す値(負数など)を渡せるようにする: 呼び出し側が
//   「無効なら呼ばない」だけで済む方が、渡す値を間違える余地が無い。
//   鮮度で判定するなら、呼び出し側がクラッシュして呼ばなくなった場合も
//   同じ経路で拾える。
void Position_setHeight(float height) {
    // 範囲外と NaN をここで落とす。
    //
    // **NaN が入ると Position_reset() まで消えない。** _pos_vel_x は
    // 一次遅れフィルタで前回値を持ち越すので、一度 NaN が入ると以降
    // ずっと NaN になる。さらに _Position_clamp() の比較は NaN に対して
    // 全て false になるため、クランプもすり抜けて目標角として出る。
    // pid.h が入口で isfinite を見ているのと同じ理由(pid.h の Why not)。
    const bool is_height_sane =
        isfinite(height) && (height >= 0.0f) && (height < 100.0f);
    if (!is_height_sane) {
        return;
    }

    _pos_height         = height;
    _pos_last_height_us = _Position_nowUs();
}

// 速度目標[m/s]を与える(機体固定座標。x:前 / y:右)
//
// スティックは速度指令として入る。中立(0,0)がポジションホールド。
// ±POSITION_VEL_LIMIT にクランプする。
//
// Why not 位置目標を与える形にする: 操縦者が持っているのはスティックの
//   変位で、それは「どれだけ動かしたいか」であって「どこに居たいか」では
//   ない。速度指令なら、スティックを倒している間だけ流れ、戻せばその場に
//   止まる、という直感どおりの動きになる。
void Position_setVelTarget(float vx, float vy) {
    // NaN はクランプをすり抜けるので入口で落とす(Position_setHeight と同じ)。
    const bool is_target_sane = isfinite(vx) && isfinite(vy);
    if (!is_target_sane) {
        return;
    }

    _pos_vel_target_x = _Position_clamp(vx, POSITION_VEL_LIMIT);
    _pos_vel_target_y = _Position_clamp(vy, POSITION_VEL_LIMIT);
}

// Flow を信用できるかを判定する
//
// dt・高度の鮮度・高度の大きさ・検出品質・モーションの鮮度の5つを見る。
static bool _Position_isFlowUsable(float dt) {
    const bool is_dt_usable = (dt > 0.0f) && (dt <= PID_DT_MAX);
    if (!is_dt_usable) {
        return false;
    }

    // 高度が一度も来ていない、または古い。
    //
    // Why not dt を積算して判定する: 制御ループが詰まって周期が崩れると
    //   dt の積算は「時間が進んでいない」ように見える。絶対時刻の差なら
    //   ループが止まっていても次に呼ばれた瞬間に気づく。altitude.h と同じ。
    const int64_t now            = _Position_nowUs();
    const bool    has_any_height = (_pos_last_height_us != 0);
    if (!has_any_height) {
        return false;
    }
    const bool is_height_fresh =
        (now - _pos_last_height_us) < POSITION_HEIGHT_TIMEOUT_US;
    if (!is_height_fresh) {
        return false;
    }

    // 高度が低すぎると、v = 見かけの角速度 × h の h が潰れて速度が出ない。
    // 床に置いた状態を「速度0」と誤認するのではなく、無効として扱う。
    const bool is_height_enough = _pos_height >= POSITION_FLOW_MIN_HEIGHT;
    if (!is_height_enough) {
        return false;
    }

    // 無地の床・暗所では、動いていないのに移動量が出る。
    const bool is_quality_enough = Flow_getSqual() >= POSITION_FLOW_SQUAL_MIN;
    if (!is_quality_enough) {
        return false;
    }

    // モーションの報告が途絶えていないか。
    // 詳しい理由は POSITION_FLOW_STALL_TIMEOUT_US のコメント。
    const bool has_any_motion = (_pos_last_motion_us != 0);
    if (!has_any_motion) {
        return false;
    }
    const bool is_motion_fresh =
        (now - _pos_last_motion_us) < POSITION_FLOW_STALL_TIMEOUT_US;
    if (!is_motion_fresh) {
        return false;
    }

    return true;
}

// 推定と制御を止めて、出力を安全側(目標角0)に落とす
//
// Flow が信用できないとき、外れ値を掴んだときの共通処理。
//
// Why not 呼び出し箇所ごとに同じ代入を並べる: 「速度を0にしたが目標角を
//   戻し忘れた」という抜けが、まさにこのファイルを1つにまとめた理由
//   (冒頭の Why not)。落とす手順は1箇所にしか書かない。
static void _Position_degrade() {
    // 速度を0にして位置積分を凍結する。
    //
    // Why not 直前の速度を保持して積分を続ける: 保持した速度で積分すると、
    //   実際には止まっている機体の推定位置が一定速度で流れ続け、
    //   位置制御が「戻ろう」として本当に動き出す。観測が無い区間は
    //   何も足さない方が、誤差が増えない。
    // Why not 加速度を積分して繋ぐ: バイアスとノイズの二重積分で
    //   数秒で発散する(knowledge/position-hold-design.md の却下案)。
    _pos_vel_x    = 0.0f;
    _pos_vel_y    = 0.0f;
    _pos_is_valid = false;

    // 目標角も0に落とす。速度が0だからと制御を続けると、
    // 凍結した位置誤差に対して傾き続けることになる。
    _pos_roll_target  = 0.0f;
    _pos_pitch_target = 0.0f;

    // I項が溜まったまま復帰すると、その分が一気に目標角として出る。
    Pid_reset(&_pos_vel_pid_x);
    Pid_reset(&_pos_vel_pid_y);
    Pid_reset(&_pos_pos_pid_x);
    Pid_reset(&_pos_pos_pid_y);

    // 位置も原点に戻す。
    //
    // Why not 凍結したまま残す: 観測が途切れている間に機体は流されるが、
    //   凍結した _pos_x は流される前の値のまま。復帰した瞬間に
    //   「元の位置まで戻れ」という大きな誤差が一気に効いて、機体が急に走る。
    //   原点を復帰したその場に引き直す方が、挙動が予測できる。
    //   位置ホールドの基準が変わるが、観測が無い間の位置は元々信用できない。
    _pos_x = 0.0f;
    _pos_y = 0.0f;
}

// 速度推定と位置制御の更新
//
// dt[s] は前回の Position_update() からの経過時間。Flow の読み出し間隔と
// 同じでなければならない(Δpx がその区間の累積だから)。
// 100Hz で回すなら 0.01。
//
// 呼ぶ順序: Flow_update() → Position_setHeight() → Position_update()。
// Δpx を読む前に Flow_update() が要り、h を掛ける前に高度が要る。
//
// 手順は参照実装の ESKF::updateFlowRaw() の 1〜4 段と同じ。
// (5段目の「機体座標→NED」は採らない。理由は下の位置積分のコメント)
// 出典: StampFly Ecosystem (MIT License, Copyright (c) 2026 Kouhei Ito)
//       firmware/vehicle_old/components/sf_algo_eskf/eskf.cpp updateFlowRaw()
//       https://github.com/M5Fly-kanazawa/stampfly_ecosystem
void Position_update(float dt) {
    // モーションの鮮度を先に更新する。
    // 判定より前に書かないと、今回のフレームの報告が判定に入らない。
    const bool has_measurement = Flow_hasMoved();
    if (has_measurement) {
        _pos_last_motion_us = _Position_nowUs();
    }

    const bool is_flow_usable = _Position_isFlowUsable(dt);
    if (!is_flow_usable) {
        _Position_degrade();
        return;
    }

    // 観測が無いフレームは、速度・位置を前回値のまま凍結して抜ける。
    //
    // Why これが要るか: ドライバはモーションビットが立たないとき Δ に 0 を
    //   代入する。この 0 を「観測された流れ」として扱うと、回転成分の除去
    //   (引き算)だけが残り
    //     flow_trans_x = 0 - gyro_y = -gyro_y
    //   という**存在しない速度が生成される**。静止していても角速度に比例した
    //   偽の速度が出て、それが目標角として姿勢ループへ返る。ピッチレートが
    //   正のとき機首下げ指令が返る、符号の決まった帰還が閉じるので、
    //   ゲイン次第で首振り発振か一方向へのドリフトになる。
    //   このファイルが「その場で傾けるだけなら速度≈0」を目指しているのと
    //   正反対の挙動なので、観測が無いフレームでは引き算自体を走らせない。
    //
    // Why not 速度を0にする: 0 も「静止している」という観測になってしまう。
    //   本当に静止しているのか、単に報告が無いだけなのかは区別できない。
    //   前回値のまま置く方が、短い空白では実態に近い。
    //   空白が POSITION_FLOW_STALL_TIMEOUT_US を超えれば
    //   _Position_isFlowUsable() が false になり、上で降格される。
    if (!has_measurement) {
        return;
    }

    const float delta_x = (float)Flow_getDeltaX();
    const float delta_y = (float)Flow_getDeltaY();

    // 外れ値のフレームは丸ごと捨てる。位置積分も進めない。
    // 床の切れ目や反射で、実際には動いていないのに大きな値が出る。
    const bool is_delta_sane = (fabsf(delta_x) <= POSITION_FLOW_DELTA_MAX) &&
                               (fabsf(delta_y) <= POSITION_FLOW_DELTA_MAX);
    if (!is_delta_sane) {
        _Position_degrade();
        return;
    }

    // --- 1. ピクセル → 見かけの角速度[rad/s] -----------------------
    // センサーが見ている地面の流れは「角度の流れ」として出る。
    // dt で割るのは、Δpx が区間の累積量だから。
    //
    // Why not 参照実装の flow_offset を引く: あちらは Δpx の定常オフセットを
    //   持つが、値は実測しないと決まらない。0 を引く行を先に置くと
    //   「補正済み」に見えてしまう。三十一歩で静止時の Δpx を測って
    //   オフセットが有意なら、そのときに足す。**要実測**。
    const float flow_cam_x = delta_x * POSITION_FLOW_RAD_PER_PIXEL / dt;
    const float flow_cam_y = delta_y * POSITION_FLOW_RAD_PER_PIXEL / dt;

    // --- 2. 回転成分の除去 ----------------------------------------
    //
    // **これを省くと必ず発振する。**
    // 機体が傾くと、その場に静止していても地面は流れて見える。その流れを
    // 「動いた」と解釈して補正すると、補正のために機体が傾き、傾いたことで
    // また流れが出る。傾く→流れを検出→補正で傾く、の正帰還になる。
    // 三十二歩の検証「その場で傾けるだけなら速度≈0」はこれを見るためのもの。
    // 出典:
    // knowledge/position-hold-design.md「★の回転成分除去を省くと発振する」
    //       参照実装 eskf.cpp updateFlowRaw() の "2. Remove rotation component"
    //
    // **符号が入れ替わることに注意。** ピッチ回転(y軸まわり)がカメラの
    // x方向の流れを作り、ロール回転(x軸まわり)がカメラの y方向の流れを
    // 作るので、軸の対応が x↔y で入れ替わり、片方だけ符号が反転する。
    //   flow_rot_x = +gyro_y   (ピッチレート)
    //   flow_rot_y = -gyro_x   (ロールレート)
    // 出典: knowledge/stampfly-ecosystem-reference.md「符号に注意」
    //       eskf.cpp:1075-1076 と一致
    //
    // 渡すのは**バイアス補正済みの角速度**。参照実装も同じ位置で
    // gyro_x - state_.gyro_bias.x を作って渡している(eskf.cpp:1072-1073)。
    // Attitude_getRollRate() / getPitchRate() は既にバイアス補正済みなので、
    // ここで引くものは無い。
    //
    // Why not 生のジャイロ(Imu_getGyro)を使う: バイアスが乗ったままだと、
    //   静止していても回転成分が引かれ続け、その分が偽の水平速度になる。
    //   バイアスは温度で動くので、起動時に較正した値を使う
    //   attitude.h 側の出口から取るのが正しい。
    //
    // Why not gyro_scale の係数を持つ: 参照実装は flow_gyro_scale を持つが
    //   実測値は 1.0 で、補正が要らないことが分かっている。1.0 の係数を
    //   掛ける行は、読む人に「ここに何か調整の余地がある」と誤解させる。
    const float gyro_x = Attitude_getRollRate();   // ロールレート[rad/s]
    const float gyro_y = Attitude_getPitchRate();  // ピッチレート[rad/s]

    const float flow_rot_x = +gyro_y;
    const float flow_rot_y = -gyro_x;

    const float flow_trans_x_cam = flow_cam_x - flow_rot_x;
    const float flow_trans_y_cam = flow_cam_y - flow_rot_y;

    // --- 3. カメラ座標 → 機体座標 ---------------------------------
    // 軸ごとのスケール補正だけ(非対角は0なので掛け算を書かない)。
    const float flow_trans_x = POSITION_FLOW_CAM_TO_BODY_XX * flow_trans_x_cam;
    const float flow_trans_y = POSITION_FLOW_CAM_TO_BODY_YY * flow_trans_y_cam;

    // --- 4. 並進速度[m/s] -----------------------------------------
    // 見かけの角速度 × 距離 = 接線速度。距離は高度そのもの。
    // 高い位置ほど同じ速度でも地面の流れが遅く見えるので、h を掛けて戻す。
    const float vel_x_raw = flow_trans_x * _pos_height;
    const float vel_y_raw = flow_trans_y * _pos_height;

    // --- 5. 一次遅れフィルタ --------------------------------------
    // Δpx の量子化ノイズを鈍らせる。a = dt/(RC+dt), RC = 1/(2π·fc)。
    // dt から毎回計算するので、読み出し間隔が揺れてもカットオフは保たれる。
    const float rc = 1.0f / (2.0f * (float)M_PI * POSITION_VEL_CUTOFF_HZ);
    const float a  = dt / (rc + dt);
    _pos_vel_x += a * (vel_x_raw - _pos_vel_x);
    _pos_vel_y += a * (vel_y_raw - _pos_vel_y);

    _pos_is_valid = true;

    // --- 6. 位置は速度の積分 --------------------------------------
    //
    // 機体固定座標のまま積む。ヨーで回して地面固定座標に直さない。
    //
    // Why not ヨーで回して地面固定座標にする(参照実装の5段目): ヨーは
    //   ジャイロ積分のみで絶対基準を持たず、時間とともにドリフトする
    //   (attitude.h)。ドリフトしたヨーで速度を回すと、位置の座標系そのものが
    //   ゆっくり回転し、「戻る方向」が少しずつずれていく。
    //   参照実装は ESKF がヨーも状態として推定しているので回せるが、
    //   こちらのヨーは積分値そのものなので前提が違う。
    //   この機体はヨーレート0を保つ前提(ヨー制御が働いている)なので、
    //   機体固定座標は実質的に固定されている。誤差を持ち込む変換を
    //   足す理由がない。
    //   知っておくべき制約: **ヨーを大きく回すとこの前提が崩れる。**
    //   回したら Position_reset() で原点を引き直す。
    _pos_x += _pos_vel_x * dt;
    _pos_y += _pos_vel_y * dt;

    // --- 7. 位置P → 速度目標 --------------------------------------
    //
    // 速度目標が0(=ホールド)のときだけ位置を保つ。スティックが倒れて
    // いる間は「動かしたい」のだから、位置に引き戻されては困る。
    //
    // Why not 常に位置Pを効かせる: スティックを倒すほど位置誤差が増え、
    //   位置Pがそれを打ち消す向きに働く。倒し続けているのに機体が
    //   進まない、という動きになる。
    const bool is_hold_mode =
        (fabsf(_pos_vel_target_x) < POSITION_VEL_HOLD_EPSILON) &&
        (fabsf(_pos_vel_target_y) < POSITION_VEL_HOLD_EPSILON);

    float vel_sp_x = _pos_vel_target_x;
    float vel_sp_y = _pos_vel_target_y;

    if (is_hold_mode) {
        // 目標位置は常に原点(0,0)。Position_reset() を呼んだ場所に留まる。
        vel_sp_x = Pid_update(&_pos_pos_pid_x, 0.0f, _pos_x, dt);
        vel_sp_y = Pid_update(&_pos_pos_pid_y, 0.0f, _pos_y, dt);
    } else {
        // ホールドしていない間は位置制御器を止め、原点も現在地へ引き直す。
        // そうしないと、スティックを戻した瞬間に「離れた分だけ戻る」動きが
        // 出る。戻したその場に留まるのが期待される動作。
        Pid_reset(&_pos_pos_pid_x);
        Pid_reset(&_pos_pos_pid_y);
        _pos_x = 0.0f;
        _pos_y = 0.0f;
    }

    vel_sp_x = _Position_clamp(vel_sp_x, POSITION_VEL_LIMIT);
    vel_sp_y = _Position_clamp(vel_sp_y, POSITION_VEL_LIMIT);

    // --- 8. 速度PI → 目標角 ---------------------------------------
    //
    // 符号の対応:
    //   前(+x)へ進むには機首を下げる → pitch は負(pitch正=機首上げ)
    //   右(+y)へ進むには右を下げる   → roll は正(roll正=右下がり)
    // つまり pitch だけ符号が反転する。
    //
    // Why not attitude.h / mixer.h 側の pitch の符号を反転して揃える:
    //   あちらは「pitch正=機首上げ」で推定・配分・制御の3者が一致して
    //   いる(attitude.h のコメント)。物理的にも「機首上げが正」の方が
    //   読んで分かる。反転が要るのは「進みたい向き」と「傾ける向き」が
    //   そもそも逆だからで、それはここでしか出てこない事情。
    //   一致している3者を崩すより、ここで1回反転する方が影響が小さい。
    const float pitch_from_vel =
        Pid_update(&_pos_vel_pid_x, vel_sp_x, _pos_vel_x, dt);
    const float roll_from_vel =
        Pid_update(&_pos_vel_pid_y, vel_sp_y, _pos_vel_y, dt);

    // --- 9. 目標角のクランプ --------------------------------------
    //
    // Pid_update() の out_limit でも同じ値で切っているが、ここでも切る。
    //
    // Why not PID の out_limit だけに任せる: この二重化は冗長ではなく、
    //   「ゲインを間違えても10度を超えない」という保証を、ゲインの設定と
    //   同じ場所に置かないためのもの。Pid_init() の引数を1つ間違えれば
    //   out_limit は簡単に消える(0以下を渡すと無制限になる)。
    //   最後に出す値を出す直前に切るのが、最も外れにくい。
    _pos_pitch_target =
        _Position_clamp(-pitch_from_vel, POSITION_ANGLE_LIMIT_RAD);
    _pos_roll_target = _Position_clamp(roll_from_vel, POSITION_ANGLE_LIMIT_RAD);
}

// 前方向速度[m/s]を取得する(機体固定座標。前が正)
float Position_getVelX() { return _pos_vel_x; }

// 右方向速度[m/s]を取得する(機体固定座標。右が正)
float Position_getVelY() { return _pos_vel_y; }

// 前方向の移動量[m]を取得する
//
// Position_reset() を呼んだ地点からの相対位置。速度の積分なので
// ドリフトする。絶対位置ではない。
float Position_getPosX() { return _pos_x; }

// 右方向の移動量[m]を取得する
float Position_getPosY() { return _pos_y; }

// 目標ロール角[rad]を取得する。角度PIDの目標値に入れる。
//
// ±POSITION_ANGLE_LIMIT_RAD(10度)にクランプ済み。
// Position_isValid() が false のときは 0 が返る。
float Position_getRollTarget() { return _pos_roll_target; }

// 目標ピッチ角[rad]を取得する。角度PIDの目標値に入れる。
float Position_getPitchTarget() { return _pos_pitch_target; }

// 直近の更新で Flow を信用できたか
//
// false の間は速度0・位置凍結・目標角0になっている。
// 200ms 続いたら位置ホールドから高度ホールドへ降格する
// (knowledge/position-hold-design.md の失陥表)。判定は呼び出し側で行う。
bool Position_isValid() { return _pos_is_valid; }
