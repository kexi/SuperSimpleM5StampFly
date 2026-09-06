#pragma once

// ToF距離センサ(VL53L3CX)のドライバー — 下向きのみ
//
// 高度ホールド(三十歩)に必要な「下向きの距離[m]」だけを取る。
//
// ■ この歩は最小実装にとどめ、精度は割り切る
//
// VL53L3CXはヒストグラム方式で、STが配布する BareDriver(1.2.14) は
// 約3万行ある。参照実装もそれをまるごと抱えている。
// 出典: StampFly Ecosystem (MIT License, Copyright (c) 2026 Kouhei Ito)
//       firmware/vehicle/components/sf_hal_vl53l3cx/
//       （src/vl53lx/ 配下だけで 29,954 行。VL53LX_GetMultiRangingData() で
//         マルチターゲットの距離列を得る作りになっている）
//       https://github.com/M5Fly-kanazawa/stampfly_ecosystem
//
// 3万行の移植は「なるべくシンプルに」という本リポジトリの方針に合わない。
// 一方でレジスタマップを読むと、VL53L3CX は VL53L1X と同じ番地割り当てで、
// 単一ターゲットの距離は 0x0096 に mm で出てくることが分かる。
// 出典: 参照実装 include/vl53lx/vl53lx_register_map.h の以下が
//       VL53L1X の公開データシートと一致することを確認した:
//         VL53LX_GPIO_HV_MUX__CTRL                            0x0030
//         VL53LX_GPIO__TIO_HV_STATUS                          0x0031
//         VL53LX_SYSTEM__INTERRUPT_CLEAR                      0x0086
//         VL53LX_SYSTEM__MODE_START                           0x0087
//         VL53LX_RESULT__RANGE_STATUS                         0x0089
//         VL53LX_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 0x0096
//         VL53LX_FIRMWARE__SYSTEM_STATUS                      0x00E5
//         VL53LX_IDENTIFICATION__MODEL_ID                     0x010F
// そこで VL53L1X 流の「設定ブロックを一括で書いて 0x0096 を読む」だけの
// 実装にする。
//
// この割り切りで捨てるもの:
//   - ヒストグラム処理による複数ターゲットの分離(窓ガラス越しなど)
//   - クロストーク補正・オフセット補正のキャリブレーション
//   - VL53L3CX本来の測距レンジ(最大値は VL53L1X 相当まで落ちる)
// 室内の高度ホールドで見るのは「真下の床までの 0.05〜2m」だけなので、
// これらが無くても成立すると判断した。**要実測**。
//
// Why not STのBareDriverをそのまま入れる: 動くものは早く手に入るが、
//   何が起きているか読めないブラックボックスが3万行増える。
//   本リポジトリは「ゼロから書いて理解する」ことが目的で、目的と手段が逆転する。
// Why not Pololu/Adafruit等のArduinoライブラリを使う: 同じ理由。
//   加えて、それらはVL53L1X用でVL53L3CXでの動作は保証されていない。
//   自分で最小のレジスタ操作を書けば、どこまでが保証範囲かが自分で分かる。

#include <Arduino.h>

#include "driver_i2c.h"  // I2C_read16 / I2C_write16

// I2Cアドレス --------------------------------
// VL53L3CXの出荷時アドレス(7bit)。前向き・下向きとも同じ値で出てくる。
// 出典: 参照実装 include/stampfly_tof_config.h
//       VL53L3CX_DEFAULT_I2C_ADDR 0x29
#define TOF_I2C_ADDR 0x29

// レジスタ ------------------------------------
// 番地は参照実装の vl53lx_register_map.h に一致する(上の出典を参照)
#define TOF_REG_SOFT_RESET              0x0000  // ソフトリセット
#define TOF_REG_I2C_SLAVE_DEVICE_ADDR   0x0001  // I2Cアドレス変更用
#define TOF_REG_CONFIG_START            0x002D  // 設定ブロックの先頭
#define TOF_REG_GPIO_HV_MUX_CTRL        0x0030  // 割り込みの極性設定
#define TOF_REG_GPIO_TIO_HV_STATUS      0x0031  // 割り込みピンの状態
#define TOF_REG_SYSTEM_INTERRUPT_CLEAR  0x0086  // 割り込みクリア
#define TOF_REG_SYSTEM_MODE_START       0x0087  // 測定の開始/停止
#define TOF_REG_RESULT_RANGE_STATUS     0x0089  // 測定結果のステータス
#define TOF_REG_RESULT_RANGE_MM         0x0096  // 距離[mm](クロストーク補正後)
#define TOF_REG_FIRMWARE_SYSTEM_STATUS  0x00E5  // ブート完了フラグ
#define TOF_REG_IDENTIFICATION_MODEL_ID 0x010F  // モデルID

// 期待値 --------------------------------------
// VL53L1/L3ファミリ共通のモデルID(0x010F,0x0110の2バイト)
#define TOF_MODEL_ID 0xEACC

// 測定モード ----------------------------------
#define TOF_MODE_START_BACK_TO_BACK 0x40  // 連続測定
#define TOF_MODE_STOP               0x00  // 停止

// 割り込み極性 ----------------------------------
// 0x0030 の bit4 が極性ビット。0 = アクティブHIGH / 1 = アクティブLOW。
// 出典: 参照実装 include/vl53lx/vl53lx_ll_device.h
//         VL53LX_DEVICEINTERRUPTLEVEL_ACTIVE_HIGH 0x00
//         VL53LX_DEVICEINTERRUPTLEVEL_ACTIVE_LOW  0x10
//         VL53LX_DEVICEINTERRUPTLEVEL_ACTIVE_MASK 0x10
#define TOF_INTERRUPT_POLARITY_MASK 0x10

// 有効とみなす距離の範囲[m] ---------------------
// 下限: センサー自身のカバーガラスの反射を拾う領域を避ける。**要実測**
// 上限: 最小実装ではヒストグラム処理を捨てているので、遠距離は信用しない。
//       高度ホールドが対象にするのは 2m までなので実用上困らない。**要実測**
#define TOF_DISTANCE_MIN 0.03f
#define TOF_DISTANCE_MAX 2.00f

// ポーリング間隔[us] ----------------------------
// センサー自身は約30Hz(タイミングバジェット33ms)でしか値を更新しないが、
// ここは「新しい値ができたか」を見に行く間隔なので 5ms(200Hz)にする。
// 33ms 間隔で見に行くと、センサーの更新位相とずれた分がまるごと遅延に
// なり、最悪で 1周期(33ms)ぶん古い高度で制御することになる。
// 細かく覗いておけば、出来上がった直後に拾える。
//
// Why not 制御ループ(400Hz)の毎周期で覗く: I2Cは400kHzでも1バイト読むのに
//   数十us掛かる。データレディ判定だけで400Hzぶん回すと、値が変わらない
//   周期にも同じコストを払う。200Hzなら遅延は5ms以内に収まり、
//   30Hz更新のセンサーに対しては十分細かい。
#define TOF_POLL_INTERVAL_US 5000

// レンジステータス ------------------------------
// 0 = 有効。それ以外はシグマ不足・信号不足・範囲外などのエラー。
#define TOF_RANGE_STATUS_VALID 0

// 状態 ----------------------------------------
static bool     _tof_initialized  = false;  // 初期化に成功したか
static float    _tof_distance_m   = 0.0f;   // 最後に読めた距離[m]
static bool     _tof_valid        = false;  // その距離が信用できるか
static bool     _tof_has_new_data = false;  // 直近のupdateで新データを読んだか
static uint32_t _tof_last_poll_us = 0;      // 最後にポーリングした時刻

// 割り込みが立っているとみなす bit0 の値。
// Tof_init() が 0x0030 を読んで決める。
//
// Why not 「アクティブLOW」で決め打ちする: 極性は 0x0030 の設定次第で、
//   設定ブロックの内容やセンサー個体で変わりうる。決め打ちして極性が
//   逆だと「常にデータレディ」に見え、同じ古い値を読み続けたまま
//   エラーも出ない。参照実装(src/vl53lx/vl53lx_wait.c)も
//   0x0030 を読んで interrupt_ready を決めており、それに倣う。
static uint8_t _tof_interrupt_ready = 0x01;

// VL53L1X系の初期設定ブロック(0x2D〜0x87の91バイト)
//
// 電源投入直後のレジスタは測定できる状態になっておらず、この列を
// 先頭から順に書いて初めて動く。個々のビットの意味はSTが公開していない。
// VL53L1Xの公開ドライバ(ST UltraLite Driver)が書いている既定値と同じ並びで、
// タイミングバジェットは既定のまま(約33ms相当)にしてある。
//
// **要実測**: この91バイトは参照実装から取ったものではない
// (参照実装はこういう平坦なブロックを持たず、BareDriverが個別に書く)。
// VL53L1X ULD の既定値として書いてあるので、実機で測距が始まらない
// 場合はまずここを疑う。
//
// Why not 意味のあるレジスタだけ書く: どのビットが必須かの資料が無く、
//   部分的に書くと「動くこともある」状態になる。再現しないバグの温床なので、
//   ブロックごと決め打ちで書いて、変数は後から個別に上書きする方針にする。
//
// clang-format off
static const uint8_t _TOF_CONFIG_BLOCK[] = {
    0x00,  // 0x2D : 予約(既定値)
    0x00,  // 0x2E
    0x00,  // 0x2F
    0x01,  // 0x30 : GPIO_HV_MUX__CTRL。bit4=0 なのでアクティブHIGH
    0x02,  // 0x31
    0x00,  // 0x32
    0x02,  // 0x33
    0x08,  // 0x34
    0x00,  // 0x35
    0x08,  // 0x36
    0x10,  // 0x37
    0x01,  // 0x38
    0x01,  // 0x39
    0x00,  // 0x3A
    0x00,  // 0x3B
    0x00,  // 0x3C
    0x00,  // 0x3D
    0xFF,  // 0x3E
    0x00,  // 0x3F
    0x0F,  // 0x40
    0x00,  // 0x41
    0x00,  // 0x42
    0x00,  // 0x43
    0x00,  // 0x44
    0x00,  // 0x45
    0x20,  // 0x46 : 割り込みは新しいデータが出るたびに立てる
    0x0B,  // 0x47
    0x00,  // 0x48
    0x00,  // 0x49
    0x02,  // 0x4A
    0x0A,  // 0x4B
    0x21,  // 0x4C
    0x00,  // 0x4D
    0x00,  // 0x4E
    0x05,  // 0x4F
    0x00,  // 0x50
    0x00,  // 0x51
    0x00,  // 0x52
    0x00,  // 0x53
    0xC8,  // 0x54
    0x00,  // 0x55
    0x00,  // 0x56
    0x38,  // 0x57
    0xFF,  // 0x58
    0x01,  // 0x59
    0x00,  // 0x5A
    0x08,  // 0x5B
    0x00,  // 0x5C
    0x00,  // 0x5D
    0x01,  // 0x5E
    0xDB,  // 0x5F
    0x0F,  // 0x60
    0x01,  // 0x61
    0xF1,  // 0x62
    0x0D,  // 0x63
    0x01,  // 0x64 : シグマ閾値の上位
    0x68,  // 0x65 : シグマ閾値の下位
    0x00,  // 0x66
    0x80,  // 0x67
    0x08,  // 0x68
    0xB8,  // 0x69
    0x00,  // 0x6A
    0x00,  // 0x6B
    0x00,  // 0x6C : 測定間隔(インターメジャメント)
    0x00,  // 0x6D
    0x0F,  // 0x6E
    0x89,  // 0x6F
    0x00,  // 0x70
    0x00,  // 0x71
    0x00,  // 0x72
    0x00,  // 0x73
    0x00,  // 0x74
    0x00,  // 0x75
    0x00,  // 0x76
    0x01,  // 0x77
    0x0F,  // 0x78
    0x0D,  // 0x79
    0x0E,  // 0x7A
    0x0E,  // 0x7B
    0x00,  // 0x7C
    0x00,  // 0x7D
    0x02,  // 0x7E
    0xC7,  // 0x7F
    0xFF,  // 0x80
    0x9B,  // 0x81
    0x00,  // 0x82
    0x00,  // 0x83
    0x00,  // 0x84
    0x01,  // 0x85
    0x00,  // 0x86 : 割り込みクリア
    0x00,  // 0x87 : 測定開始(ここでは止めたまま。Tof_init()の最後で開始する)
};
// clang-format on

// 1バイト書く
static bool _Tof_write8(uint16_t reg, uint8_t value) {
    return I2C_write16(TOF_I2C_ADDR, reg, &value, 1);
}

// 1バイト読む
//
// 戻り値: 通信に成功したら true。値は out に入る。
//
// Why not 失敗時に 0 を返す形にする: 0 は「割り込みが立っている(極性が
//   アクティブLOWのとき)」「ブート未完了」「レンジステータス正常」の
//   いずれとしても正しい値で、通信失敗と区別できない。
//   センサーを抜いた状態で「常にデータレディ・ステータス正常」に
//   見えてしまうので、成否は戻り値で分ける。
static bool _Tof_read8(uint16_t reg, uint8_t* out) {
    return I2C_read16(TOF_I2C_ADDR, reg, out, 1);
}

// 2バイト読む(ビッグエンディアン)
// 戻り値: 通信に成功したら true。値は out に入る。
static bool _Tof_read16(uint16_t reg, uint16_t* out) {
    uint8_t buf[2] = {0, 0};
    if (!I2C_read16(TOF_I2C_ADDR, reg, buf, 2)) {
        return false;
    }
    *out = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    return true;
}

// 2個のToFのうち、下向きだけを起こす
//
// 前向き(G9)と下向き(G7)は同じI2Cアドレス0x29で工場出荷される。
// 両方を同時に起こすと、同じアドレスの2台が同時に応答してバスが壊れ、
// どちらの値も読めない。
// 出典: knowledge/stampfly-axes-and-mixer.md「ToF の XSHUT ピン」
//       (参照実装 README.md のピン表: 前方=GPIO9 / 底面=GPIO7)
//
// この歩で必要なのは下向きの距離だけなので、前向きはXSHUTをLOWに落として
// シャットダウンしたまま置く。LOWの間そのセンサーはI2Cに一切応答しない。
//
// Why not 片方のアドレスを0x30に変え替えて2台とも使う: 参照実装はそうしている
//   （底面ToFを0x30へ変更）が、そのためには「片方だけ起こす→アドレスを書き換える
//   →もう片方を起こす」という順序依存の手順が要る。
//   前向きToFは今のところ使い道が無く(障害物検知は設計に入っていない)、
//   手順を増やすだけで得るものが無い。前向きを使う歩が来たときに足す。
// Why not 前向きのXSHUTを未接続のまま放置する: XSHUTは内部プルアップで
//   HIGHに寄るため、放っておくと勝手に起きてアドレスが衝突する。
//   明示的にLOWで押さえないと「たまに動かない」になる。
static void _Tof_shutdownFrontSensor() {
    pinMode(PIN_TOF_FRONT_XSHUT, OUTPUT);
    digitalWrite(PIN_TOF_FRONT_XSHUT, LOW);
}

// 下向きToFの電源を入れ直す。
// 起動直後は状態が不定なので、一度落としてから上げて揃える。
static void _Tof_powerCycleBottomSensor() {
    pinMode(PIN_TOF_BOTTOM_XSHUT, OUTPUT);
    digitalWrite(PIN_TOF_BOTTOM_XSHUT, LOW);
    delay(10);  // 内部の電源が抜けきるまでの待ち
    digitalWrite(PIN_TOF_BOTTOM_XSHUT, HIGH);
    delay(10);  // ブート開始待ち(データシートのブート時間は最大1.2ms)
}

// ファームウェアのブート完了を待つ
// 戻り値: 時間内にブートしたら true
static bool _Tof_waitBooted() {
    // 100msあればブートは終わる。終わらなければ繋がっていないと判断する。
    for (int i = 0; i < 100; i++) {
        uint8_t status = 0;
        // 読めない間はまだ起きていないだけなので、失敗も待ちとして扱う
        if (_Tof_read8(TOF_REG_FIRMWARE_SYSTEM_STATUS, &status)) {
            const bool is_booted = (status & 0x01) != 0;
            if (is_booted) {
                return true;
            }
        }
        delay(1);
    }
    return false;
}

// 割り込みの極性を読み取り、_tof_interrupt_ready を決める
//
// 0x0030 の bit4 が 0 ならアクティブHIGH、1 ならアクティブLOW。
// データレディ判定で 0x0031 の bit0 と突き合わせる値をここで作る。
// 出典: 参照実装 src/vl53lx/vl53lx_wait.c
//       VL53LX_is_new_data_ready_poll() と同じ判定
static bool _Tof_readInterruptPolarity() {
    uint8_t mux_ctrl = 0;
    if (!_Tof_read8(TOF_REG_GPIO_HV_MUX_CTRL, &mux_ctrl)) {
        return false;
    }
    const bool is_active_high = (mux_ctrl & TOF_INTERRUPT_POLARITY_MASK) == 0;
    _tof_interrupt_ready      = is_active_high ? 0x01 : 0x00;
    return true;
}

// ToFセンサの初期化
//
// 戻り値: センサーと通信でき、測定を開始できたら true
//
// I2Cバスの初期化(I2C_init())は呼び出し側の責任。
// 気圧センサ・電流センサと同じバスを共有していて、ここで初期化すると
// 他のドライバの初期化順に依存してしまうため。
bool Tof_init() {
    _tof_initialized  = false;
    _tof_valid        = false;
    _tof_has_new_data = false;
    _tof_distance_m   = 0.0f;
    _tof_last_poll_us = 0;

    // 先に前向きを黙らせる。順序が逆だと、下向きを起こした瞬間に
    // アドレスが衝突して以降の通信が全部失敗する。
    _Tof_shutdownFrontSensor();
    _Tof_powerCycleBottomSensor();

    if (!_Tof_waitBooted()) {
        return false;
    }

    // モデルIDを確認する。繋がっていないのに設定を書いても意味がない。
    uint16_t model_id = 0;
    if (!_Tof_read16(TOF_REG_IDENTIFICATION_MODEL_ID, &model_id)) {
        return false;
    }
    const bool is_connected = (model_id == TOF_MODEL_ID);
    if (!is_connected) {
        return false;
    }

    // 設定ブロックを先頭から一括で書く。
    // Wireの送信バッファは既定で128バイトあり、91バイト+アドレス2バイトは
    // 1トランザクションに収まる。
    const bool is_config_written =
        I2C_write16(TOF_I2C_ADDR, TOF_REG_CONFIG_START, _TOF_CONFIG_BLOCK,
                    (uint8_t)sizeof(_TOF_CONFIG_BLOCK));
    if (!is_config_written) {
        return false;
    }

    // 極性は設定ブロックを書いた後の値で決める。
    // ブロックが 0x0030 を上書きするので、書く前に読むと違う値になる。
    if (!_Tof_readInterruptPolarity()) {
        return false;
    }

    // 連続測定を開始する。以降センサーは自分のペースで測り続け、
    // 新しい値ができるたびに割り込みフラグを立てる。
    if (!_Tof_write8(TOF_REG_SYSTEM_MODE_START, TOF_MODE_START_BACK_TO_BACK)) {
        return false;
    }

    // 1回目の測定を待って捨てる。
    // 電源投入直後の1発目はキャリブレーションを兼ねていて値が信用できない。
    delay(50);
    _Tof_write8(TOF_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    _tof_initialized = true;
    return true;
}

// 新しい測定結果が用意できているか
//
// 0x0031 の bit0 を、Tof_init() が 0x0030 から決めた期待値と突き合わせる。
// 一致していれば割り込みが立っている = 新しい値がある。
static bool _Tof_isDataReady() {
    uint8_t status = 0;
    if (!_Tof_read8(TOF_REG_GPIO_TIO_HV_STATUS, &status)) {
        return false;  // 読めないなら新データは無いものとして扱う
    }
    return (status & 0x01) == _tof_interrupt_ready;
}

// ToFセンサの更新
//
// 制御ループ(400Hz)から毎周期呼んでよい。内部で間引くので、
// I2Cを叩くのは最大でも200Hz。
// 新しいデータが無ければ前回値をそのまま保つ。
void Tof_update() {
    // 新データフラグは毎周期落とす。
    // Tof_hasNewData() は「直近のこの呼び出しで新しい値を読んだか」を
    // 表すので、前の周期の true を持ち越してはいけない。
    _tof_has_new_data = false;

    if (!_tof_initialized) {
        return;
    }

    // 間引き。前回のポーリングから十分時間が経つまで何もしない。
    const uint32_t now_us = micros();
    const uint32_t elapsed_us =
        now_us - _tof_last_poll_us;  // オーバーフロー時も差は正しい
    const bool is_too_early = elapsed_us < TOF_POLL_INTERVAL_US;
    if (is_too_early) {
        return;
    }
    _tof_last_poll_us = now_us;

    // データが無ければ前回値を保つ。
    // 30Hzのセンサーを200Hzで見に行くので、ほとんどの回はここで戻る。
    if (!_Tof_isDataReady()) {
        return;
    }

    uint8_t    range_status = 0;
    uint16_t   distance_mm  = 0;
    const bool is_read_ok =
        _Tof_read8(TOF_REG_RESULT_RANGE_STATUS, &range_status) &&
        _Tof_read16(TOF_REG_RESULT_RANGE_MM, &distance_mm);

    // 次の測定へ進ませる。ここを忘れると割り込みが立ちっぱなしになり、
    // 同じ値を延々読み続けることになる。読み出しに失敗していても
    // クリアする(しないと次も同じ失敗した測定を見に行くだけ)。
    _Tof_write8(TOF_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    if (!is_read_ok) {
        _tof_valid = false;
        return;
    }

    // ステータスの下位5bitが実際のエラーコード。上位は内部フラグ。
    const bool is_status_ok = (range_status & 0x1F) == TOF_RANGE_STATUS_VALID;
    if (!is_status_ok) {
        // 測れなかった。前回値は残すが「信用するな」と伝える。
        // Why not 0を返す: 高度制御が0を高度0mと解釈すると全開で上昇する。
        //   値を残して無効フラグを立て、使うかどうかを制御側に決めさせる。
        _tof_valid = false;
        return;
    }

    const float distance_m = distance_mm * 0.001f;

    const bool is_in_range =
        (distance_m >= TOF_DISTANCE_MIN) && (distance_m <= TOF_DISTANCE_MAX);
    if (!is_in_range) {
        _tof_valid = false;
        return;
    }

    _tof_distance_m   = distance_m;
    _tof_valid        = true;
    _tof_has_new_data = true;
}

// 最後に読めた距離[m]を取得する
//
// 傾いていると斜め距離になる。真下までの高度に直すのは高度推定(三十歩)の
// 仕事で、ここでは生の距離をそのまま返す。
//
// Why not ここで傾き補正まで済ませる: ドライバが姿勢推定に依存すると、
//   センサー単体で動作確認できなくなる。ドライバは「センサーが返した値」だけを
//   返し、機体の都合はその上の層で足す。
float Tof_getDistance() { return _tof_distance_m; }

// 直近の距離が信用できるかを返す
// 未初期化・通信失敗・測定エラー・範囲外なら false
bool Tof_isValid() { return _tof_initialized && _tof_valid; }

// 直近の Tof_update() で**新しい**測距値を読んだかを返す
//
// 高度推定(三十歩)の Altitude_notifyTof() は「ToFの新データが来た周期だけ
// 補正する」ことを前提にしている。Tof_update() は200Hzで呼ばれても
// 実際に値が変わるのは約30Hzなので、この関数が true の周期だけ通知する。
//
// Why not Altitude 側で「前回と違う値か」を見て判断する: 静止していれば
//   同じ距離が正しく2回来る。値の比較では「新しいが同じ値」と
//   「古い値の読み直し」が区別できず、静止時ほど補正が入らなくなる。
//   新データの到着を知っているのは割り込みフラグを見たドライバだけ。
bool Tof_hasNewData() { return _tof_has_new_data; }
