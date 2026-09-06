#pragma once

// 安全装置(アーム状態機械とフェイルセーフ)
//
// この機体でモーターに何かを書く経路は、必ずここを通す。
// Safety_isArmed() が false の間、ミキサーは全モーターに 0 を書く。
//
// Why not 各所で個別に安全判定を入れる: 判定が散らばると「どの条件が
//   モーターを止めるのか」を読むのにファイルを横断することになり、
//   条件を1つ足したときに漏れる箇所が出る。落とす理由(Safety_getReason())
//   まで含めて1箇所に集約する。
//
// 状態は DISARMED <-> ARMED の2つだけ。離陸・着陸シーケンスのような
// 中間状態は持たない。
//
// Why not 参照実装のような多状態(INIT/IDLE_GROUND/TAKEOFF/FLYING/...):
//   StampFly Ecosystem (MIT License, Copyright (c) 2026 Kouhei Ito)
//   firmware/vehicle/components/sf_state/include/flight_state.hpp は
//   7状態 + 4モードを持つが、あれは自動離着陸を含む機能があってのもの。
//   ここでの関心は「モーターを回してよいか」の一点で、状態を増やすほど
//   「今どの状態か」を追う手間が増える。必要になった歩で足す。
//
// 設計の出典: knowledge/position-hold-design.md「安全性」

#include <math.h>
#include <stdint.h>

#include "attitude.h"  // Attitude_getRoll / Attitude_getPitch [rad]

// 失陥の理由 ----------------------------------
// LEDの色分けやシリアル出力で「なぜ落ちたか」を出せるように列挙型で持つ。
//
// Why not bool 1つで「落ちた」だけを持つ: 実機では原因が分からないと
//   同じ落ち方を繰り返すだけになる。特にリンク断と傾き過大は現場での
//   対処が正反対(送信機側を疑う / 機体を置き直す)なので区別が要る。
enum SafetyReason {
    SAFETY_REASON_NONE,           // 失陥していない
    SAFETY_REASON_LINK_LOST,      // リンク断(規定時間 新しい受信が無い)
    SAFETY_REASON_TILT_EXCEEDED,  // 傾き過大(反転・衝突後)
    SAFETY_REASON_IMU_STALE,      // IMU異常(新しいデータが来ない)
    SAFETY_REASON_USER_DISARM,    // 操縦者からのディスアーム要求
};

// アーム条件のしきい値 --------------------------
// スロットルは 0.0-1.0 に正規化された値で渡す前提。
// 完全な 0 を要求するとスティックの中点ずれで永久にアームできないので、
// 少しだけ余裕を持たせる。
//
// Why not 0.0 ちょうど: JoyStick の ADC 生値は個体差で数十カウントずれる
//   (Joy_calibrate() が吸収するが完全ではない)。4095 に対する 0.05 は
//   約 200 カウントで、実測の中点ずれ(<300 カウントを閾値にしている)より
//   厳しくない範囲。**実機で要調整。**
#define SAFETY_ARM_THROTTLE_MAX 0.05f

// アームを許す傾き。水平に置いた状態からの許容ずれ。
// 机や床が多少傾いていても通る程度に取ってある。
#define SAFETY_ARM_TILT_MAX_RAD (10.0f * 0.017453293f)  // 10度

// 失陥のしきい値 --------------------------------
// リンク断の判定時間。
//
// Why not 参照実装の 500ms: StampFly Ecosystem
//   (firmware/vehicle/components/sf_comm/include/comm.hpp の
//    kLinkTimeoutMs = 500) は 500ms を使うが、あちらはリンク断で
//   ホバリングしてから自律着陸する。こちらは即カットなので、
//   カットまでの惰性飛行を短くする方を選ぶ。
//   36g・室内1m 以内では、落ちることより飛び去ることの方が危険。
#define SAFETY_LINK_TIMEOUT_US 300000  // 300ms

// 傾き過大の判定。ここを超えたら姿勢制御では戻せないとみなす。
#define SAFETY_TILT_LIMIT_RAD (60.0f * 0.017453293f)  // 60度

// IMU の鮮度。制御周期 2.5ms に対して 8 周期分の猶予。
// 姿勢が分からない機体は制御できないので、他より厳しく取る。
#define SAFETY_IMU_TIMEOUT_US 20000  // 20ms

// 内部状態 ------------------------------------
static bool         _safety_is_armed = false;
static SafetyReason _safety_reason   = SAFETY_REASON_NONE;

// 最後に「新しいデータが来た」時刻[us]。Safety_update() で経過を見る。
//
// Why not 経過時間をfloatで積算する: 400Hz で dt を足し続けると
//   丸め誤差が乗る。esp_timer_get_time() の絶対時刻の差なら乗らない。
static int64_t _safety_last_comm_us = 0;
static int64_t _safety_last_imu_us  = 0;

// アーム条件の判定に使う、外から与えられる状態
static float _safety_throttle     = 0.0f;
static bool  _safety_is_connected = false;

// 現在時刻[us]を取る。SILS のホストテストでは esp_timer が無いので分ける。
static int64_t _Safety_nowUs() {
#ifdef SF_HOST_TEST
    extern int64_t SafetyTest_nowUs();  // テスト側が時刻を進める
    return SafetyTest_nowUs();
#else
    return esp_timer_get_time();
#endif
}

// 安全装置の初期化
//
// 起動直後は必ず DISARMED。リンクも IMU も「まだ来ていない」状態から
// 始めるため、受信時刻を現在時刻で埋めない。
//
// Why not 受信時刻を現在時刻で初期化する: 起動直後の 300ms だけ
//   リンクが生きているように見える。アーム条件に BLE 接続中が入って
//   いるので実害は出にくいが、「見えている状態が実際と違う」ことを
//   作らない。
void Safety_init() {
    _safety_is_armed     = false;
    _safety_reason       = SAFETY_REASON_NONE;
    _safety_last_comm_us = 0;
    _safety_last_imu_us  = 0;
    _safety_throttle     = 0.0f;
    _safety_is_connected = false;
}

// 通信を受信したことを通知する
//
// BLE の受信コールバック(BLE_EVENT_RECEIVED)から毎回呼ぶ。
// 呼ばれた時刻だけを覚え、中身は見ない。
void Safety_notifyCommRx() { _safety_last_comm_us = _Safety_nowUs(); }

// IMU の新しいデータを取り込んだことを通知する
//
// Imu_update() の直後に呼ぶ。SPI が死んで同じ値を読み続けるケースまでは
// ここでは見ない(全軸0固定の検出は別の歩で足す)。
void Safety_notifyImuUpdate() { _safety_last_imu_us = _Safety_nowUs(); }

// 操縦者のスロットル入力を渡す(0.0-1.0)
//
// アーム条件の「スロットル最小」の判定にだけ使う。
void Safety_setThrottle(float throttle) { _safety_throttle = throttle; }

// BLE の接続状態を渡す
//
// Why not driver_ble.h を直接参照する: StampFly 側は BLE_PERIPHERAL で
//   BLE_isConnected() が定義されていない(CENTRAL 側にしかない)。
//   接続/切断イベントを受け取る main が状態を持ち、ここへ渡す。
void Safety_setLinkConnected(bool is_connected) {
    _safety_is_connected = is_connected;
}

// アーム済みか。モーター出力はこれが true のときだけ許される。
bool Safety_isArmed() { return _safety_is_armed; }

// 直近の失陥理由。LED やシリアルへの表示に使う。
// アームに成功すると SAFETY_REASON_NONE に戻る。
SafetyReason Safety_getReason() { return _safety_reason; }

// 失陥理由を人が読める文字列にする(シリアル出力用)
const char* Safety_getReasonName(SafetyReason reason) {
    switch (reason) {
        case SAFETY_REASON_NONE:
            return "NONE";
        case SAFETY_REASON_LINK_LOST:
            return "LINK_LOST";
        case SAFETY_REASON_TILT_EXCEEDED:
            return "TILT_EXCEEDED";
        case SAFETY_REASON_IMU_STALE:
            return "IMU_STALE";
        case SAFETY_REASON_USER_DISARM:
            return "USER_DISARM";
        default:
            return "UNKNOWN";
    }
}

// ディスアーム要求。いつでも最優先で通る。
//
// 条件を一切見ない。飛行中でも即座にモーターが止まる。
// 「止める」側に条件を付けると、止めたい状況ほど条件が揃わなくなる。
void Safety_requestDisarm() {
    _safety_is_armed = false;
    _safety_reason   = SAFETY_REASON_USER_DISARM;
}

// アーム要求。全条件が揃ったときだけ通る。
//
// 戻り値: アームできたら true
//
// Why not 揃うまで待って自動でアームする: 操縦者が意図していない
//   タイミングでプロペラが回り始めることになる。アームは常に人の
//   明示的な要求から始める。
bool Safety_requestArm() {
    // 既にアームしているなら何もしない(理由も消さない)
    if (_safety_is_armed) {
        return true;
    }

    const float   roll  = Attitude_getRoll();
    const float   pitch = Attitude_getPitch();
    const int64_t now   = _Safety_nowUs();

    // スロットルが上がったままアームすると、その瞬間に全開で回りだす
    const bool is_throttle_low = _safety_throttle <= SAFETY_ARM_THROTTLE_MAX;
    if (!is_throttle_low) {
        return false;
    }

    // 傾いた状態でアームすると、姿勢制御が水平へ戻そうとして跳ねる
    const bool is_level = (fabsf(roll) < SAFETY_ARM_TILT_MAX_RAD) &&
                          (fabsf(pitch) < SAFETY_ARM_TILT_MAX_RAD);
    if (!is_level) {
        return false;
    }

    // IMU が動いていること。一度も通知が来ていない(=0)場合も弾く。
    const bool is_imu_alive =
        (_safety_last_imu_us != 0) &&
        ((now - _safety_last_imu_us) < SAFETY_IMU_TIMEOUT_US);
    if (!is_imu_alive) {
        return false;
    }

    // リンクが繋がっていること。繋がっていない機体はアーム後すぐ
    // リンク断で落ちるだけなので、そもそもアームさせない。
    const bool is_link_alive =
        _safety_is_connected && (_safety_last_comm_us != 0) &&
        ((now - _safety_last_comm_us) < SAFETY_LINK_TIMEOUT_US);
    if (!is_link_alive) {
        return false;
    }

    _safety_is_armed = true;
    _safety_reason   = SAFETY_REASON_NONE;
    return true;
}

// 安全装置の更新。制御ループから毎周期(400Hz)呼ぶ。
//
// dt はインタフェースを他の Xxx_update(dt) と揃えるために受け取るが、
// 判定には使わない。
//
// Why not dt を積算してタイムアウトを測る: 制御ループが詰まって
//   周期が崩れたとき、dt の積算は「時間が進んでいない」ように見える。
//   失陥検出は、まさにループが崩れている状況で正しく働く必要がある。
//   絶対時刻の差なら、ループが止まっていても次に呼ばれた瞬間に気づく。
void Safety_update(float dt) {
    (void)dt;

    // ディスアーム中は何も検出しない。
    // 地上に置いたまま傾けても失陥理由が上書きされない方が、
    // 直前になぜ落ちたのかを読める。
    if (!_safety_is_armed) {
        return;
    }

    const int64_t now = _Safety_nowUs();

    // --- リンク断 --------------------------------------------------
    // **即カット。自律降下はしない。**
    //
    // Why not リンク断で自律降下(ホバリングしながら降ろす):
    //   自律降下には高度推定(ToF + 相補フィルタ)と高度制御が正しく
    //   動いている前提が要る。つまりフェイルセーフの中で最も複雑な
    //   経路が、最も信頼できない状況(通信が切れた = 何かが壊れている)で
    //   走ることになる。降下中に高度推定が外れれば、止まっていたはずの
    //   機体が加速する。
    //   36g・室内1m 以内という条件では、落ちるダメージより飛び去る
    //   危険の方が大きい。即座に止める方が結果を予測できる。
    //   参照実装(StampFly Ecosystem)は自律着陸を選んでいるが、
    //   あちらは高度制御が実証済みで、こちらはまだ無い。
    const bool is_link_lost =
        !_safety_is_connected ||
        ((now - _safety_last_comm_us) >= SAFETY_LINK_TIMEOUT_US);
    if (is_link_lost) {
        _safety_is_armed = false;
        _safety_reason   = SAFETY_REASON_LINK_LOST;
        return;
    }

    // --- IMU 異常 --------------------------------------------------
    // 姿勢が分からない機体は制御できない。傾き判定より先に見るのは、
    // 古い姿勢値で傾き判定をしても意味が無いため。
    const bool is_imu_stale =
        (now - _safety_last_imu_us) >= SAFETY_IMU_TIMEOUT_US;
    if (is_imu_stale) {
        _safety_is_armed = false;
        _safety_reason   = SAFETY_REASON_IMU_STALE;
        return;
    }

    // --- 傾き過大 --------------------------------------------------
    // 反転・衝突した後にモーターを回し続けない。
    // 60度は姿勢制御で戻せる範囲を明らかに超えている。
    const float roll             = Attitude_getRoll();
    const float pitch            = Attitude_getPitch();
    const bool  is_tilt_exceeded = (fabsf(roll) > SAFETY_TILT_LIMIT_RAD) ||
                                  (fabsf(pitch) > SAFETY_TILT_LIMIT_RAD);
    if (is_tilt_exceeded) {
        _safety_is_armed = false;
        _safety_reason   = SAFETY_REASON_TILT_EXCEEDED;
        return;
    }
}
