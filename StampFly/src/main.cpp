// はじめの二十八歩
// 姿勢推定・PID・ミキサー・安全装置を繋いで、手動ホバリングまで持っていく
//
// ここまでの部品(attitude.h / pid.h / mixer.h /
// safety.h)を制御ループに結線する。 カスケードは
// knowledge/position-hold-design.md の
//   位置P → 速度PI → 角度P → 角速度PID → ミキサー
// のうち、**内側2段(角度P → 角速度PID)だけ**を有効にする。
//
// Why not 位置・速度ループまで一度に書く: 外側2段は高度推定(altitude.h)と
//   速度推定(position.h)を前提にしていて、そのどちらもまだ実測で裏が取れて
//   いない(三十歩・三十二歩)。無い推定値の代わりに定数を入れて形だけ作ると、
//   「動いていないのに繋がっている」コードが残る。外側の枠はコメントで
//   置き場所だけ示し、中身は歩が来たときに埋める。
//
// **プロペラを付ける最初の歩。** 必ずテザー(糸を機体中央に結んで手で保持)を
// 付け、クッションの上で、人は2m以上離れて始めること。
// 出典: knowledge/position-hold-design.md「運用ルール」
//
// ---- 操縦方法 --------------------------------------------------------
//
//   左スティック 上下(y1) : スロットル(上に倒すほど強い)
//   右スティック 左右(x2) : ロール(右に倒すと右へ傾く)
//   右スティック 上下(y2) : ピッチ(上に倒すと機首下げ = 前進)
//   左スティック 左右(x1) : ヨーレート(右に倒すと機首が右へ回る)
//
//   アーム    : 左右のトリガを両方、1秒間押し続ける
//               (スロットル最小・ほぼ水平・IMU正常・BLE接続中 が揃っていること)
//   ディスアーム: 右スティックのボタンを押す。いつでも最優先で即停止
//
// Why not 1つのボタンでアームとディスアームをトグルする: 止めたいときに
//   間違って押すとアームしてしまう。**止める操作は常に単押しで、
//   始める操作だけを難しくする**という非対称にする。
// Why not アームも単押し: 転がしたときにボタンが当たってプロペラが回る。
//   両トリガ同時1秒は、意図せず成立することがまず無い組み合わせ。
//   出典: knowledge/position-hold-design.md「アーム条件: … 両トリガ1秒」

#include <Arduino.h>

// BLEライブラリ
#include <NimBLEDevice.h>

#define BLE_PERIPHERAL 1

// ピン定義。
//
// Why not 他のドライバ経由で入ることに頼る: driver_motor.h / driver_led.h /
//   driver_sound.h は PIN_* を使うのに hardware_config.h を include して
//   いない。今は driver_flow.h → driver_spi.h が先に引き込むので通るが、
//   この並びは include の順序という「読んでも気づけない依存」でしかない。
//   Motor_init() は setup の先頭で確実に動く必要があるので、明示して切る。
#include "hardware_config.h"

// ../Common/driver/include/...
#include "driver_ble.h"    // BLEドライバ
#include "driver_flow.h"   // オプティカルフローセンサドライバ
#include "driver_i2c.h"    // I2Cドライバ
#include "driver_imu.h"    // IMUドライバ
#include "driver_led.h"    // LEDドライバ
#include "driver_motor.h"  // モータードライバ
#include "driver_sound.h"  // サウンドドライバ
#include "driver_timer.h"  // Timerドライバ
#include "driver_tof.h"    // ToF距離センサドライバ

// StampFly/include/...
#include "altitude.h"  // 高度推定と高度制御
#include "attitude.h"  // 姿勢推定
#include "mixer.h"     // モーター配分
#include "pid.h"       // PID
#include "position.h"  // 速度推定と位置制御
#include "safety.h"    // 安全装置

// 制御周期 --------------------------------------
// 400Hz(2500us)。attitude.h の ATTITUDE_UPDATE_HZ と揃っていること。
// PIDのゲインは周期に依存するので、ゲインを詰める前に確定させてある。
#define CONTROL_PERIOD_US 2500
#define CONTROL_DT        (CONTROL_PERIOD_US / 1000000.0f)  // 0.0025[s]

// 外側ループの間引き ------------------------------
// フロー・ToF・高度・位置は 100Hz(4フレームに1回)。
// 姿勢ほど速く要らないうえ、センサ自体が 30〜100Hz でしか更新されない。
#define CONTROL_SLOW_DIVIDER 4
#define CONTROL_SLOW_DT      (CONTROL_DT * CONTROL_SLOW_DIVIDER)  // 0.01[s]

// ---- スティックのスケーリング ---------------------------------------
//
// JoyStick 側は 12bit ADC の生値(0-4095)をそのまま送ってくる。
// 中点は 2048 付近だが個体差でずれる(Joy_calibrate() が吸収しきれない分)。
//
// Why not 短い名前(STICK_CENTER 等)のままにする: このファイルは main.cpp
//   なので他所へ漏れないが、driver_joy.h が同名の enum STICK_L/STICK_R を
//   持っている。将来 driver_joy.h をここへ入れたときに意味の違う STICK_*
//   が並ぶので、機体側の操縦入力であることを接頭辞で示す。
#define CTRL_STICK_CENTER 2048.0f
#define CTRL_STICK_HALF   2048.0f

// 中立とみなす幅(正規化後の絶対値)。
// 中点ずれと ADC ノイズで、指を離していても目標角が入り続けるのを防ぐ。
// **要実測**(スティックを離したときの正規化値の振れ幅をCSVで見て決める)。
//
// 参照実装 StampFly Ecosystem にも同種のデッドゾーンがあるが、**具体的な
// 定数名と値までは本セッションで確認していない**ので、出典として数字を
// 借りずに自前の初期値として置く。Joy_calibrate() の合格しきい値が
// 生値 300 カウント(= 正規化 0.073)なので、それより少し狭い所から始める。
#define CTRL_STICK_DEADZONE 0.06f

// スティック最大倒しに対する目標角[rad]。
// 参照実装は 30度(attitude_control::MAX_ROLL_ANGLE = 0.5236f)だが、
// こちらは室内で 1m 以内のホバリングしかしない。30度も傾けると一瞬で
// 壁まで飛ぶので、傾きを浅くして扱いやすくする。
// **要実測**(浅すぎて姿勢を戻せないなら上げる)。
// 出典: StampFly Ecosystem (MIT License, Copyright (c) 2026 Kouhei Ito)
//       https://github.com/M5Fly-kanazawa/stampfly_ecosystem
#define CTRL_STICK_MAX_ANGLE_RAD (12.0f * 0.017453293f)  // 12度

// スティック最大倒しに対する目標ヨーレート[rad/s]。
// 参照実装の 5.0 rad/s (~286deg/s) はアクロ用で、室内でこの速さで回すと
// 方向が分からなくなる。1/5 程度に落とす。**要実測**。
#define CTRL_STICK_MAX_YAW_RATE 1.0f  // rad/s (~57deg/s)

// 位置ホールド中のスティック最大速度指令[m/s]
//
// 室内なので速くしない。0.3m/s は歩くより遅く、部屋の端まで数秒かかる。
// position.h 側でも同じ値でクランプしているが、ここで先に絞っておけば
// スティックの可動域いっぱいが実際に使える範囲に対応する。
// **要実測**(遅すぎて操縦感が悪ければ上げる)。
#define CTRL_STICK_MAX_VELOCITY 0.3f

// 高度ホールド中のスロットルスティックの扱い
//
// スティックは「昇降速度」の指示として使う。中立で今の高度を保つ。
// 0.5m/s は室内で扱いやすい速さ。**要実測**。
#define CTRL_CLIMB_RATE 0.5f  // m/s

// スティック中立付近の不感帯。ノイズで目標高度が流れるのを防ぐ。
#define CTRL_THROTTLE_DEADBAND 0.10f

// ToF が使えないときに、高度を固定値とみなして Flow だけ試すためのモード
//
// 0 にすると通常動作(ToFの推定высоを使う)。0以外にすると、その値[m]を
// 高度として Position に渡す。
//
// Why これが要るか: Optical Flow の移動量を速度に直すには高度が要るので、
//   ToF が動かないと位置制御を一切検証できない。テザーで一定の高さに
//   保てば、高度を手で与えて Flow の軸・符号・回転成分除去だけを
//   先に確かめられる。ToF が直ったら 0 に戻す。
//   詳細は knowledge/preflight-checklist.md「ToF が動かなかった場合」。
#define CTRL_FIXED_HEIGHT 0.0f  // m (0 = ToFの推定を使う)

// スロットルの上限[duty]。
// ミキサーの MOTOR_DUTY_MAX (0.8) まで素で出せると、姿勢トルクを足す余地が
// スティック側で先に食い潰される。操縦入力の段でも上限をかける。
// ホバリングは概ね 0.4〜0.6 の見込み(**要実測**)なので、0.7 あれば足りる。
#define CTRL_STICK_MAX_THROTTLE 0.7f

// ---- PIDゲイン ------------------------------------------------------
//
// **ここの数字は全て「要実測」。飛ばしながら詰めるための出発点でしかない。**
//
// 出発点の作り方(根拠のない数字を置かないため):
//   参照実装 StampFly Ecosystem (MIT License, Copyright (c) 2026 Kouhei Ito)
//   firmware/vehicle_old/main/config.hpp の「電圧スケールゲイン」を、
//   こちらの正規化 duty スケールへ換算した。
//   あちらの操作量の単位は電圧[V]で、出力制限が 3.7V(電池電圧)。
//   こちらの操作量は無次元の duty で、実質の上限は 1.0。
//   したがって **ゲインを 1/3.7 する**と、同じ物理的な効き方になる。
//
//   あちらは C(s) = Kp(1 + 1/(Ti·s) + Td·s) の時定数形なので、
//   並列形へは  ki = Kp/Ti,  kd = Kp·Td  で移す。
//
//   角速度ロール: Kp=0.65, Ti=0.7, Td=0.01
//     → kp = 0.65/3.7      = 0.176
//       ki = 0.65/0.7/3.7  = 0.251
//       kd = 0.65*0.01/3.7 = 0.00176
//   角速度ピッチ: Kp=0.95, Ti=0.7, Td=0.025 (ピッチは慣性が大きく強め)
//   角速度ヨー  : Kp=3.0,  Ti=0.8, Td=0.01
//   角度(共通)  : Kp=5.0  ※角度ループの出力は[rad/s]で単位換算が要らない
//
// **この換算は机上の理屈で、実機で合う保証はない。**
// あちらとこちらでは推定器も(ESKF/相補フィルタ)、ミキサーの正規化も、
// モーターの動作点も違う。**最初は kp を半分から始め、振動しない範囲で
// 上げていくこと。** 詰めた最終値と測定条件は必ず knowledge/ に残す。
//
// 詰める順序(内側から。外側を先に触ると何が効いたか分からなくなる):
//   1. 角速度PIDの kp を、手で機体を回して振動が出る直前まで上げる
//   2. 角速度PIDの kd を、1で残った高周波の暴れが収まるまで
//   3. 角速度PIDの ki を、ゆっくりしたドリフトが消えるまで
//   4. 角度Pの kp を、姿勢の戻りが遅すぎない範囲で
//
// 出典: https://github.com/M5Fly-kanazawa/stampfly_ecosystem
//       firmware/vehicle_old/main/config.hpp (rate_control / attitude_control)

// 角速度ループ(内側)。入力[rad/s] → 出力[無次元 duty差]
#define PID_RATE_ROLL_KP 0.176f   // 要実測
#define PID_RATE_ROLL_KI 0.251f   // 要実測
#define PID_RATE_ROLL_KD 0.0018f  // 要実測

#define PID_RATE_PITCH_KP 0.257f   // 要実測 (0.95/3.7)
#define PID_RATE_PITCH_KI 0.367f   // 要実測 (0.95/0.7/3.7)
#define PID_RATE_PITCH_KD 0.0064f  // 要実測 (0.95*0.025/3.7)

#define PID_RATE_YAW_KP 0.811f   // 要実測 (3.0/3.7)
#define PID_RATE_YAW_KI 1.014f   // 要実測 (3.0/0.8/3.7)
#define PID_RATE_YAW_KD 0.0081f  // 要実測 (3.0*0.01/3.7)

// 角速度PIDのI項上限と出力上限[duty]。
// ミキサーは throttle ± (roll+pitch+yaw) を配る。3軸が同時に上限まで出ると
// スロットルより姿勢トルクの方が大きくなり、必ず飽和する。
// 1軸あたり 0.25 なら、3軸合計でも 0.75 で MOTOR_DUTY_MAX(0.8) に収まる。
#define PID_RATE_I_LIMIT   0.10f  // 要実測
#define PID_RATE_OUT_LIMIT 0.25f  // 要実測

// 角度ループ(外側)。入力[rad] → 出力[rad/s](= 角速度ループの目標値)
//
// Why not D項を入れる: 角度の微分は角速度そのもので、それは内側ループが
//   既に見ている。外側にもDを入れると同じ信号に二重にゲインが掛かる。
// Why not I項を入れる: 定常的な傾き(重心ずれ)は内側のI項が吸収する。
//   外側にもIを置くと、両方が同じ誤差を積んで押し合いになる。
#define PID_ANGLE_ROLL_KP  5.0f  // 要実測 (参照実装 ROLL_ANGLE_KP と同値)
#define PID_ANGLE_PITCH_KP 5.0f  // 要実測 (参照実装 PITCH_ANGLE_KP と同値)

// 角度PIDの出力上限[rad/s]。ここが角速度ループの目標値の上限になる。
// 参照実装の MAX_RATE_SETPOINT = 3.0 rad/s に合わせる。**要実測**。
#define PID_ANGLE_OUT_LIMIT 3.0f

// ---- CSVログ --------------------------------------------------------
// 50Hz(8フレームに1回)で出す。
//
// Why not 400Hz で全周期出す: 115200bps では1行(約120バイト)に 10ms 掛かる。
//   2.5ms 周期に間に合わず、シリアル送信でループが詰まって制御が壊れる。
//   50Hz なら 1行/20ms で、姿勢の振動(数Hz〜十数Hz)を見るには足りる。
// Why not 10Hz: ゲインを詰めるときに見たいのは振動の波形で、
//   10Hz では 5Hz の振動すら形が見えない(ナイキスト)。
#define CTRL_LOG_DIVIDER 8

// ログを出すフレームの位相をずらす。
//
// Why not どちらも frame % N == 0 にする: CONTROL_SLOW_DIVIDER(4) と
//   CTRL_LOG_DIVIDER(8) はどちらも 8 の約数なので、8フレームに1回
//   「フロー読み出し + シリアル送信」が同じ周期に重なる。最も重い処理が
//   一箇所に固まると、その周期だけ 400Hz を割る。位相を1フレームずらして
//   谷を分散させる。
#define CTRL_LOG_PHASE 1

// ジョイスティックのボタンビット(JoyStick 側 main.cpp の詰め方に合わせる)
//
// Why not 共有ヘッダの enum(BUTTON_TRIG_L 等)を使う: あれは driver_joy.h の
//   もので、JoyStick 側の I2C ドライバを StampFly に持ち込むことになる。
//   ここで欲しいのは「BLE で飛んでくる 1バイトのどのビットか」だけ。
#define CTRL_BUTTON_TRIG_L  (1 << 0)
#define CTRL_BUTTON_TRIG_R  (1 << 1)
#define CTRL_BUTTON_STICK_L (1 << 2)
#define CTRL_BUTTON_STICK_R (1 << 3)

// アームに必要な両トリガの保持時間[s]
#define CTRL_ARM_HOLD_SECONDS 1.0f

// ---- 状態 -----------------------------------------------------------

// センサーと通信できたか(setup で確定し、以後変わらない)
static bool is_imu_ready  = false;
static bool is_flow_ready = false;
static bool is_tof_ready  = false;

// BLE の接続状態。コールバック(別タスク)が書き、ループが読む。
static volatile bool is_link_connected = false;

// BLE 受信は別タスクのコールバックで来るので、ループ側と共有する変数は
// volatile にする。
//
// Why not ミューテックスで守る: CommData_t は 10バイトで、memcpy 中に
//   読まれると新旧が混ざった値になりうる。ただしスティック値が1周期だけ
//   混ざっても、次の 2.5ms 後には正しい値で上書きされる。400Hz の制御
//   ループでロックを取る方が、ジッタという確実な害を持ち込む。
//   **ただしボタンだけは 1周期の取り違えがアーム/ディスアームの誤動作に
//   なる**ので、ループ側で1周期に1回だけスナップショットを取り、以後は
//   そのコピーだけを見る(下記 _Control_snapshotComm)。
static volatile CommData_t commdata;
static volatile bool       is_commdata_received = false;

// この周期ぶんのスナップショット。ループ内ではこちらだけを読む。
static CommData_t ctrl_comm      = {0, 0, 0, 0, 0, 0};
static bool       is_comm_usable = false;

// ボタン(前周期との差分を取るために保持する)
static uint8_t last_button = 0;

// アーム要求が連続して成立している時間[s]
static float arm_hold_seconds = 0.0f;

// ループ処理時間の最大値[us]。400Hz に間に合っているかの確認用。
static int64_t max_elapsed_us = 0;

// PID インスタンス ------------------------------
// 角速度ループ(内側)
static Pid pid_rate_roll;
static Pid pid_rate_pitch;
static Pid pid_rate_yaw;

// 角度ループ(外側)
static Pid pid_angle_roll;
static Pid pid_angle_pitch;

// 操縦入力(正規化済み。loop の先頭で毎周期作り直す)
static float stick_throttle = 0.0f;  // 0.0〜CTRL_STICK_MAX_THROTTLE
static float stick_roll     = 0.0f;  // -1.0〜1.0
static float stick_pitch    = 0.0f;  // -1.0〜1.0
static float stick_yaw      = 0.0f;  // -1.0〜1.0

// 目標値(ログに出すために保持する)
static float target_roll_rad   = 0.0f;
static float target_pitch_rad  = 0.0f;
static float target_yaw_rate   = 0.0f;
static float control_out_roll  = 0.0f;
static float control_out_pitch = 0.0f;
static float control_out_yaw   = 0.0f;

// ---- BLE ------------------------------------------------------------

// BLEイベントコールバック
//
// **別タスクから呼ばれる。** ここで重い処理をすると BLE スタックが詰まる。
// 受信データのコピーと安全装置への通知だけに留める。
//
// Why not ここで Sound_play を呼ぶ: 音の再生はタスク生成(xTaskCreate)を
//   伴う。BLE のコールバック文脈からタスクを作ると、接続と切断が連続した
//   ときに BLE スタックのスタックを食う。**接続音・切断音はループ側で
//   状態の変化を見て鳴らす**(下記 _Control_updateLink)。
static void _ble_event_callback(BLEEventParam_t* param) {
    switch (param->event) {
        case BLE_EVENT_CONNECTED:
            is_link_connected = true;
            break;

        case BLE_EVENT_DISCONNECTED:
            is_link_connected = false;
            // 切断そのものは Safety_update() がリンク断として拾う。
            // ここで Safety_requestDisarm() を呼ぶと失陥理由が
            // USER_DISARM になり、なぜ落ちたのか分からなくなる。
            break;

        case BLE_EVENT_RECEIVED:
            memcpy((void*)&commdata, &param->data, sizeof(CommData_t));
            is_commdata_received = true;
            // 通し番号が進んだときだけ「生きている」とみなす。
            //
            // Why not 受信しただけで生存とみなす: BLEのリンクが生きていても、
            //   送信側のループが固まれば同じ値が届き続ける。受信の有無では
            //   送信側の停止を検出できない。通し番号が進むことを条件にする。
            {
                static uint8_t last_seq        = 0;
                static bool    has_seen_packet = false;
                const bool     is_fresh =
                    !has_seen_packet || (commdata.seq != last_seq);
                if (is_fresh) {
                    last_seq        = commdata.seq;
                    has_seen_packet = true;
                    Safety_notifyCommRx();
                }
            }
            break;
    }
}

// リンク状態の変化を見て、音を1回だけ鳴らす
//
// Why not コールバックの中で鳴らす: 上記のとおりタスク生成を BLE の
//   文脈に持ち込まないため。ここなら 2.5ms 以内に必ず拾える。
static void _Control_updateLink() {
    static bool was_link_connected = false;

    const bool is_link_changed = (is_link_connected != was_link_connected);
    if (!is_link_changed) {
        return;
    }

    was_link_connected = is_link_connected;

    if (is_link_connected) {
        USBSerial.println("# BLE CONNECTED");
        Sound_play(SOUND_PRESET_CONNECTED);
        return;
    }

    USBSerial.println("# BLE DISCONNECTED");
    Sound_play(SOUND_PRESET_DISCONNECTED);
}

// この周期で使う受信データを1回だけ写し取る
//
// 以後 _Control_updateSticks / _Control_updateArming は ctrl_comm しか
// 読まない。同じ周期の中でスティックとボタンが別の受信パケット由来に
// なる(= 中立のスティックと押されたトリガが混ざる)ことを防ぐ。
//
// リンクが切れている間は「受信データ無し」として扱う。
//
// Why not 最後に受け取った値を使い続ける: 切断直前のパケットでスロットルが
//   上がっていると、その値が Safety_setThrottle() に残り続ける。すると
//   再接続してもアーム条件のスロットル最小が永久に成立せず、
//   「なぜアームできないのか分からない」状態になる。
//   リンクが無い = 操縦入力は無い、と倒す。
static void _Control_snapshotComm() {
    const bool is_comm_available = is_commdata_received && is_link_connected;
    if (!is_comm_available) {
        is_comm_usable = false;
        memset(&ctrl_comm, 0, sizeof(ctrl_comm));
        return;
    }

    memcpy(&ctrl_comm, (const void*)&commdata, sizeof(CommData_t));
    is_comm_usable = true;
}

// ---- 操縦入力 --------------------------------------------------------

// 生の ADC 値(0-4095)を -1.0〜1.0 に正規化し、デッドゾーンを抜く
//
// デッドゾーンの外では 0 から連続に立ち上がるよう、残りの幅で伸ばし直す。
// Why not デッドゾーン内を 0 にするだけ: 境界で出力が 0 から
//   CTRL_STICK_DEADZONE へ飛び、指を少し動かしただけで機体がガクッと傾く。
static float _Control_normalizeStick(uint16_t raw) {
    const float value = ((float)raw - CTRL_STICK_CENTER) / CTRL_STICK_HALF;

    const bool is_in_deadzone = fabsf(value) < CTRL_STICK_DEADZONE;
    if (is_in_deadzone) {
        return 0.0f;
    }

    const float sign    = (value > 0.0f) ? 1.0f : -1.0f;
    const float shifted = fabsf(value) - CTRL_STICK_DEADZONE;
    return sign * shifted / (1.0f - CTRL_STICK_DEADZONE);
}

// 受信データから操縦入力を作る
//
// スロットルだけ扱いが違う。JoyStick の Y 軸は上に倒すと値が小さくなるので、
// (2048 - y1) / 2048 で「上に倒すほど大きい 0.0〜1.0」にする。
// 二十一歩と同じ向き。**要実測**(y1 の向きは実機のログで確認する)。
static void _Control_updateSticks() {
    // BLE が来ていないうちは全て中立にしておく。
    //
    // Why not ctrl_comm の初期値(全0)をそのまま正規化する: 0 は ADC の
    //   下限であって中立ではない。ロールが -1.0(左いっぱい)として
    //   解釈され、アーム直後に横へ飛ぶ。
    if (!is_comm_usable) {
        stick_throttle = 0.0f;
        stick_roll     = 0.0f;
        stick_pitch    = 0.0f;
        stick_yaw      = 0.0f;
        return;
    }

    float throttle =
        (CTRL_STICK_CENTER - (float)ctrl_comm.y1) / CTRL_STICK_HALF;
    if (throttle < 0.0f) {
        throttle = 0.0f;
    }
    if (throttle > 1.0f) {
        throttle = 1.0f;
    }
    stick_throttle = throttle * CTRL_STICK_MAX_THROTTLE;

    stick_roll = _Control_normalizeStick(ctrl_comm.x2);

    // ピッチはスティックを上に倒す(= y2 が小さくなる)と機首下げ = 前進。
    // attitude.h / mixer.h の pitch 正は機首上げなので、符号を反転する。
    //
    // Why not attitude.h 側で機首下げを正にする: そちらは推定と配分の
    //   符号を揃えるために機首上げ正で固定してある(反転してはいけない)。
    //   **操縦者の感覚に合わせる反転は、操縦入力を作るこの1箇所だけで行う。**
    stick_pitch = -_Control_normalizeStick(ctrl_comm.y2);

    stick_yaw = _Control_normalizeStick(ctrl_comm.x1);
}

// ---- アーム操作 ------------------------------------------------------

// PID の内部状態をまとめてリセットする
//
// アームの瞬間と、ディスアーム中の毎周期に呼ぶ。
static void _Control_resetPids() {
    Pid_reset(&pid_rate_roll);
    Pid_reset(&pid_rate_pitch);
    Pid_reset(&pid_rate_yaw);
    Pid_reset(&pid_angle_roll);
    Pid_reset(&pid_angle_pitch);
}

// ボタン入力からアーム/ディスアームを処理する
//
// dt[s] は前回呼び出しからの経過時間。トリガの保持時間を測るのに使う。
static void _Control_updateArming(float dt) {
    const uint8_t button      = ctrl_comm.button;
    const uint8_t prev_button = last_button;

    // 前回値の更新はここで済ませておく。以後どの経路で return しても
    // エッジ検出が壊れない。
    last_button = button;

    // --- ディスアーム(最優先) -------------------------------------
    // 押した瞬間だけで判定する。押しっぱなしを条件にすると、
    // 止めたいのに1秒待たされる。
    const bool is_disarm_pressed =
        (button & CTRL_BUTTON_STICK_R) && !(prev_button & CTRL_BUTTON_STICK_R);
    if (is_disarm_pressed) {
        Safety_requestDisarm();
        arm_hold_seconds = 0.0f;
        USBSerial.println("# DISARMED (user)");
        return;
    }

    // --- 失陥ラッチの解除 -------------------------------------------
    // 失陥でカットされた後は、明示的に解除するまで再アームできない。
    //
    // Why 人の操作を挟むか: 失陥の原因(衝突・IMU異常)が去れば、アーム条件
    //   は自然に揃ってしまう。ラッチが無いと、落ちた機体を拾おうとした
    //   瞬間にプロペラが回り出す。**なぜ落ちたかを人が確認してから**
    //   再開させるための一手間。
    //
    // ディスアーム(右スティックボタン)とは別のボタンにする。
    // 同じにすると、止めようとした操作で解除まで済んでしまう。
    const bool is_clear_pressed =
        (button & CTRL_BUTTON_STICK_L) && !(prev_button & CTRL_BUTTON_STICK_L);
    if (is_clear_pressed && Safety_isFailsafeLatched()) {
        Safety_clearFailsafe();
        USBSerial.println("# FAILSAFE CLEARED");
        return;
    }

    // --- 既にアーム済み ---------------------------------------------
    // カウントを進める意味がないので 0 に戻して抜ける。
    //
    // Why not そのまま return する: 溜まったままディスアームすると、
    //   トリガを握りっぱなしのまま次の周期で保持時間が満了扱いになり、
    //   **止めた直後に勝手に再アームする。** 止める操作の直後に回り出す
    //   のは、この機体で最も起きてはいけない挙動。
    if (Safety_isArmed()) {
        arm_hold_seconds = 0.0f;
        return;
    }

    // --- アーム ----------------------------------------------------
    // 両トリガを CTRL_ARM_HOLD_SECONDS 押し続けている間だけカウントを進める。
    // 離した瞬間に 0 に戻すので、断続的に押しても溜まらない。
    const bool is_arm_requested =
        (button & CTRL_BUTTON_TRIG_L) && (button & CTRL_BUTTON_TRIG_R);

    // トリガを一度完全に離すまで、次のアーム試行を受け付けない。
    //
    // Why これが要るか: 握ったままにしていると、条件が揃った瞬間に
    //   アームが成立してしまう。特に危ないのが失陥の直後で、機体が
    //   落ちて水平に落ち着いた瞬間にプロペラが回り出す。拾おうと手を
    //   伸ばしたところに回転が始まることになる。
    //   アームは常に「離す→握る」という人の明示的な操作から始める。
    static bool is_arm_latched = false;
    if (!is_arm_requested) {
        arm_hold_seconds = 0.0f;
        is_arm_latched   = false;  // 離したので次の握りを受け付ける
        return;
    }
    if (is_arm_latched) {
        return;  // 握りっぱなし。一度離すまで何もしない
    }

    arm_hold_seconds += dt;

    const bool is_hold_satisfied = arm_hold_seconds >= CTRL_ARM_HOLD_SECONDS;
    if (!is_hold_satisfied) {
        return;
    }

    // 較正が終わるまではアームさせない。
    //
    // Why not safety.h 側の条件に入れる: safety.h は attitude.h の角度は
    //   見るが、較正の進捗までは見ていない。較正中の角度は 0 のままなので
    //   「水平」判定は通ってしまう。**姿勢が信用できない機体をアームさせ
    //   ない**判定は、較正の有無を知っているここで足す。
    const bool is_ready_to_arm = Attitude_isCalibrated();
    if (!is_ready_to_arm) {
        return;
    }

    // ここまで来たら安全装置の全条件を確認してもらう。
    // スロットル最小・ほぼ水平・IMU正常・BLE接続中 は safety.h が見る。
    //
    // 通らなかったらラッチを立てて、一度離すまで再試行させない。
    // 握ったまま待たせると、条件が揃った瞬間に人の操作なしでアームする。
    const bool is_armed = Safety_requestArm();
    if (!is_armed) {
        is_arm_latched   = true;
        arm_hold_seconds = 0.0f;
        return;
    }

    // アームが成立したので、次は一度離してから握り直させる
    is_arm_latched = true;

    // 目標高度を今いる高さにする。
    // 固定値にすると、アームした瞬間にそこまで一気に上がろうとする。
    // 以降はスロットルスティックで上下させる(_Control_update 参照)。
    Altitude_setTarget(Altitude_getHeight());

    // 位置の原点を今いる場所に引き直す。
    // 地上で運んだぶんの積分が残っていると、離陸後に元の場所へ戻ろうとする。
    Position_notifyArmed();

    // アームの瞬間にI項をゼロに戻す。
    // 地上に置いている間、姿勢誤差(床の傾き・重心ずれ)は消えないまま
    // 積み続ける。溜めたまま回し始めると、その分が一気に出て機体が跳ねる。
    _Control_resetPids();

    Sound_play(SOUND_PRESET_BOOT);
    USBSerial.println("# ARMED");
    arm_hold_seconds = 0.0f;
}

// ---- 制御 ------------------------------------------------------------

// カスケード制御(角度P → 角速度PID)を1周期進め、ミキサーへ渡す
//
// dt[s] は制御周期。
//
// 目標角はスティックから直接作る。**将来ここに位置・速度ループが入る。**
// 三十三歩(速度制御)以降、スティックは速度指令になり、
//   位置P → 速度PI → ここの target_roll_rad
// という繋がりになる。今はその手前なので、スティックが直接角度を決める。
static void _Control_update(float dt) {
    // --- 角度ループ(外側) -----------------------------------------
    // スティック → 目標角[rad]
    //
    // 符号: stick_roll 正 = 右に倒す → target_roll_rad 正 = 右下がり。
    // attitude.h の roll 正(右下がり)と一致するので反転は要らない。
    // pitch は _Control_updateSticks で既に反転済み。ここで二度目の
    // 反転をしてはいけない。
    //
    // 位置ホールドが効いている間は、スティックではなく位置ループが
    // 目標角を決める。スティックは Position_setVelTarget() を通じて
    // 「どっちへどれだけの速さで動きたいか」を指示する形に変わる。
    //
    //   位置P → 速度PI → ここの target_roll_rad → 角度P → 角速度PID
    //
    // Why not 常に位置ループを通す: Flowが無効(無地の床・暗所・低すぎる高度)
    //   のときに目標角が0に張り付くと、操縦者が動かそうとしても動かない
    //   機体になる。位置が信用できないときはスティックで直接角度を決める
    //   方が、操縦者の意図どおりに動いて安全。
    const bool is_position_hold = Position_isValid();
    if (is_position_hold) {
        // スティックは速度指令。中立(0)ならその場に留まる。
        Position_setVelTarget(stick_pitch * CTRL_STICK_MAX_VELOCITY,
                              stick_roll * CTRL_STICK_MAX_VELOCITY);

        target_roll_rad  = Position_getRollTarget();
        target_pitch_rad = Position_getPitchTarget();
    } else {
        target_roll_rad  = stick_roll * CTRL_STICK_MAX_ANGLE_RAD;
        target_pitch_rad = stick_pitch * CTRL_STICK_MAX_ANGLE_RAD;
    }

    // 角度誤差 → 目標角速度[rad/s]
    const float rate_sp_roll =
        Pid_update(&pid_angle_roll, target_roll_rad, Attitude_getRoll(), dt);
    const float rate_sp_pitch =
        Pid_update(&pid_angle_pitch, target_pitch_rad, Attitude_getPitch(), dt);

    // ヨーは角度ループを通さない。
    //
    // Why not ヨー角も角度制御する: ヨー角はジャイロ積分のみでドリフトする
    //   (attitude.h に磁気センサを入れていない)。ドリフトした角度を
    //   目標にすると、機体は勝手に回り続ける方が「正しい」ことになる。
    //   ヨーは角速度だけを制御し、絶対方位は操縦者が見て直す。
    //
    // ここは PID_ANGLE_OUT_LIMIT を通らないので、スティックの
    // CTRL_STICK_MAX_YAW_RATE がそのまま角速度目標の上限になる。
    target_yaw_rate = stick_yaw * CTRL_STICK_MAX_YAW_RATE;

    // --- 角速度ループ(内側) ---------------------------------------
    // 測定値は Attitude_getXxxRate()。これはジャイロの生値(バイアス補正済み)
    // を機体軸(FRD)へ変換したもので、角度の微分ではない。
    control_out_roll =
        Pid_update(&pid_rate_roll, rate_sp_roll, Attitude_getRollRate(), dt);
    control_out_pitch =
        Pid_update(&pid_rate_pitch, rate_sp_pitch, Attitude_getPitchRate(), dt);
    control_out_yaw =
        Pid_update(&pid_rate_yaw, target_yaw_rate, Attitude_getYawRate(), dt);

    // --- 配分 -------------------------------------------------------
    // Mixer_update() が Motor_setSpeed() まで行う。PWM への書き出しは
    // 呼び出し側の Motor_update()。
    // スロットルスティックで目標高度を上下させる。
    //
    // Why not スティックの位置をそのまま目標高度にする: 中立で0.5m、
    //   上端で1.0m のように割り当てると、スティックを離した瞬間に
    //   その高度へ飛んでいく。上下の「速度」を指示する形にすれば、
    //   離したところで止まり、操縦感が普通のドローンと揃う。
    //
    // 中立付近には不感帯を置く。スティックのノイズで目標がじわじわ
    // ずれていくのを防ぐため。
    const bool is_altitude_live = (Altitude_getStatus() == ALTITUDE_STATUS_OK);
    if (is_altitude_live) {
        // stick_throttle は 0.0-1.0。0.5 を中立とみなす。
        const float climb_input = (stick_throttle - 0.5f) * 2.0f;  // -1.0..+1.0
        const bool is_in_deadband = fabsf(climb_input) < CTRL_THROTTLE_DEADBAND;
        if (!is_in_deadband) {
            const float target =
                Altitude_getTarget() + climb_input * CTRL_CLIMB_RATE * dt;
            Altitude_setTarget(target);
        }
    }

    // 高度ホールドが効いていればそのスロットルを使う。
    //
    // Why not 常に高度ホールドを通す: ToFが無効(レンジ外・センサー異常)の
    //   ときに高度ホールドのスロットルは0になる。それをそのまま渡すと
    //   墜落する。推定が生きているときだけ任せ、それ以外は操縦者の手に戻す。
    //   降格の設計は knowledge/position-hold-design.md の失陥表を参照。
    const bool  is_altitude_hold = (Altitude_getStatus() == ALTITUDE_STATUS_OK);
    const float throttle =
        is_altitude_hold ? Altitude_getThrottle() : stick_throttle;

    Mixer_update(throttle, control_out_roll, control_out_pitch,
                 control_out_yaw);
}

// 制御量をゼロに戻す(ディスアーム中に呼ぶ)
//
// I項を毎周期リセットし続けるので、ディスアーム中はどれだけ長く置いても
// 積み上がらない。ログに出る値も 0 になり、「止まっている」ことが読める。
static void _Control_reset() {
    _Control_resetPids();

    // 高度・位置の推定と積分も戻す。
    //
    // Why これが要るか: ディスアーム中も推定は動き続ける。位置の積分は
    //   機体を手で運んだぶんだけ溜まり、高度のI項も地面に置いた状態で
    //   「目標高度に届かない」誤差を積む。戻さずにアームすると、
    //   溜まったぶんを取り戻そうとして機体が走り出す。
    Altitude_reset();
    Position_reset();

    target_roll_rad   = 0.0f;
    target_pitch_rad  = 0.0f;
    target_yaw_rate   = 0.0f;
    control_out_roll  = 0.0f;
    control_out_pitch = 0.0f;
    control_out_yaw   = 0.0f;
}

// ---- LED -------------------------------------------------------------

// 機体の状態を LED の色で示す
//
//   黄の点滅   : センサー初期化に失敗(飛ばせない)
//   白の点滅   : ジャイロ較正中(機体を動かさない)
//   緑         : アーム済み(プロペラが回る)
//   赤の速い点滅: 失陥でカットした(直前の理由はシリアルに出ている)
//   青の点滅   : ディスアーム(リンクなし。JoyStick を待っている)
//   青         : ディスアーム(較正済み・リンクあり。アームできる)
static void _Control_updateLed() {
    const uint32_t frame       = Timer_getFrameCount();
    const bool     is_blink_on = (frame % 200) < 100;  // 2Hz
    const bool     is_fast_on  = (frame % 80) < 40;    // 5Hz
    const uint8_t  blink       = is_blink_on ? 100 : 0;
    const uint8_t  fast        = is_fast_on ? 150 : 0;

    if (!is_imu_ready) {
        // IMU が無ければ姿勢が分からない。飛ばせないことを最優先で示す。
        LED_setColor(0, blink, blink, 0);  // 黄の点滅
        LED_setColor(1, blink, blink, 0);
        return;
    }

    if (!Attitude_isCalibrated()) {
        LED_setColor(0, blink, blink, blink);  // 白の点滅
        LED_setColor(1, blink, blink, blink);
        return;
    }

    if (Safety_isArmed()) {
        LED_setColor(0, 0, 150, 0);  // 緑
        LED_setColor(1, 0, 150, 0);
        return;
    }

    // ディスアーム中。直前に失陥で落ちたなら、それを赤で出し続ける。
    //
    // Why not 一定時間で消す: 落ちた直後に機体を拾いに行く数秒の間に
    //   消えてしまうと、何が起きたのか分からないまま次の飛行に入る。
    //   次にアームするまで出したままにする(Safety_requestArm() の成功で
    //   SAFETY_REASON_NONE に戻る)。
    const SafetyReason reason = Safety_getReason();
    const bool         is_failsafe =
        (reason != SAFETY_REASON_NONE) && (reason != SAFETY_REASON_USER_DISARM);
    if (is_failsafe) {
        LED_setColor(0, fast, 0, 0);  // 赤の速い点滅
        LED_setColor(1, fast, 0, 0);
        return;
    }

    if (!is_link_connected) {
        LED_setColor(0, 0, 0, blink);  // 青の点滅
        LED_setColor(1, 0, 0, blink);
        return;
    }

    LED_setColor(0, 0, 0, 100);  // 青
    LED_setColor(1, 0, 0, 100);
}

// ---- CSVログ ---------------------------------------------------------

// --- 机上検証モード（プロペラを外して使う） --------------------
//
// knowledge/preflight-checklist.md の段階1を、シリアルを見ながら
// 判定できるようにする。飛行用のCSVは波形を取るためのもので、
// 「符号が合っているか」を目で見るには向かない。
//
// Why not CSVだけで済ませる: 段階1で確かめたいのは「機首を上げたとき
//   pitchが正に増えるか」といった向きの一致で、数字の羅列から読むと
//   間違えやすい。ここを間違えたまま飛ばすと正帰還で裏返るので、
//   人が誤読しにくい形で出す。
//
// アームしていない間だけ出す。飛行中はCSVの邪魔になる。
static void _Control_printBench() {
    float accel[3];
    float gyro[3];
    Imu_getAccel(accel);
    Imu_getGyro(gyro);

    const float rad_to_deg = 57.29578f;

    // 傾けた向きを言葉で出す。符号の読み違いを防ぐため。
    const float roll_deg  = Attitude_getRoll() * rad_to_deg;
    const float pitch_deg = Attitude_getPitch() * rad_to_deg;

    const char* roll_dir  = (roll_deg > 5.0f)    ? "右下がり"
                            : (roll_deg < -5.0f) ? "左下がり"
                                                 : "水平";
    const char* pitch_dir = (pitch_deg > 5.0f)    ? "機首上げ"
                            : (pitch_deg < -5.0f) ? "機首下げ"
                                                  : "水平";

    USBSerial.printf(
        "[姿勢] roll %+6.1f(%s) pitch %+6.1f(%s) yaw %+6.1f  "
        "[加速度] %+5.2f %+5.2f %+5.2f  [角速度] %+5.2f %+5.2f %+5.2f\n",
        roll_deg, roll_dir, pitch_deg, pitch_dir,
        Attitude_getYaw() * rad_to_deg, accel[0], accel[1], accel[2], gyro[0],
        gyro[1], gyro[2]);

    USBSerial.printf(
        "[Flow] dx %+5d dy %+5d squal %3d %s  "
        "[ToF] %.3fm %s  [負荷] %lldus/%dus\n",
        Flow_getDeltaX(), Flow_getDeltaY(), Flow_getSqual(),
        Flow_getSqual() >= 0x19 ? "OK" : "低品質(模様のある床へ)",
        Tof_getDistance(), Tof_isValid() ? "OK" : "無効", max_elapsed_us,
        CONTROL_PERIOD_US);
}

// CSV のヘッダを出す。'#' で始まる行はコメントなので、
// 取り込む側は '#' 行を捨てれば列名だけ拾える。
static void _Control_printLogHeader() {
    USBSerial.println(
        "t_ms,armed,reason,thr,"
        "roll,pitch,yaw,roll_sp,pitch_sp,"
        "roll_rate,pitch_rate,yaw_rate,yaw_rate_sp,"
        "out_roll,out_pitch,out_yaw,"
        "m_fl,m_fr,m_rl,m_rr,cut,max_us");
}

// 1行ぶんの CSV を出す
//
// 角度は degree で出す。rad のままだと 0.05 のような小さい数字が並び、
// 波形として眺めたときに振れているのかどうかが読めない。
static void _Control_printLog() {
    const float rad_to_deg = 57.29578f;

    USBSerial.printf(
        "%lu,%d,%s,%.3f,"
        "%.2f,%.2f,%.2f,%.2f,%.2f,"
        "%.2f,%.2f,%.2f,%.2f,"
        "%.4f,%.4f,%.4f,"
        "%.3f,%.3f,%.3f,%.3f,%.3f,%lld\n",
        (unsigned long)millis(), Safety_isArmed() ? 1 : 0,
        Safety_getReasonName(Safety_getReason()), stick_throttle,
        Attitude_getRoll() * rad_to_deg, Attitude_getPitch() * rad_to_deg,
        Attitude_getYaw() * rad_to_deg, target_roll_rad * rad_to_deg,
        target_pitch_rad * rad_to_deg, Attitude_getRollRate() * rad_to_deg,
        Attitude_getPitchRate() * rad_to_deg,
        Attitude_getYawRate() * rad_to_deg, target_yaw_rate * rad_to_deg,
        control_out_roll, control_out_pitch, control_out_yaw,
        Mixer_getDuty(MOTOR_FL), Mixer_getDuty(MOTOR_FR),
        Mixer_getDuty(MOTOR_RL), Mixer_getDuty(MOTOR_RR),
        Mixer_getThrottleCut(), (long long)max_elapsed_us);

    max_elapsed_us = 0;  // 次の区間の最大値を取り直す
}

// ---- setup / loop ----------------------------------------------------

void setup() {
    // モーターを最優先で初期化して止める。
    //
    // **起動直後にプロペラを回さないための最重要処理。** ESP32 の GPIO は
    // リセット直後にフローティングで、PWM を設定するまでモータードライバが
    // 何を読むか決まらない。他の初期化(シリアル待ちの delay、BLE、IMU の
    // 設定ブロブ書き込み)は合わせて 1秒以上掛かるので、その前に確実に
    // 0 を書き込んでおく。
    //
    // Why not USBSerial.begin() を先に置く: 直後の delay(1000) の間、
    //   モーターの PWM が未設定のまま放置される。**シリアルが出ることより
    //   プロペラが回らないことが先。**
    // Why not 初期化の並び順どおり(LED → …)に書く: 順番を「読みやすさ」で
    //   決めると、後から初期化を1行足したときにモーターが後ろへずれる。
    //   止める処理は常に先頭に置く。
    Motor_init();
    Mixer_init();    // 全モーターに 0 を書く
    Motor_update();  // ここで PWM に 0 が出る

    USBSerial.begin(115200);
    delay(1000);  // シリアルモニタの接続待ち

    LED_init();
    Sound_init();
    Timer_init(CONTROL_PERIOD_US);

    Safety_init();  // 起動直後は必ず DISARMED

    is_imu_ready  = Imu_init();
    is_flow_ready = Flow_init();

    // I2Cは複数のセンサーが共有する。ToFより先に立ち上げる。
    I2C_init();
    is_tof_ready = Tof_init();

    // 高度・位置の推定を初期化する。センサーが無くても呼んでよい
    // (それぞれ内部で無効状態から始まり、値が来るまで制御を出さない)。
    Altitude_init();
    Position_init();

    // ID 表示は残す。飛ばない・姿勢が出ないときに、
    // 「センサーと喋れていないのか、制御が悪いのか」の切り分けに要る。
    USBSerial.printf("# VL53L3CX: %s\n", is_tof_ready ? "OK" : "NG");
    USBSerial.printf("# BMI270  : %s (ID 0x%02X)\n", is_imu_ready ? "OK" : "NG",
                     Imu_getChipID());
    USBSerial.printf("# PMW3901 : %s (ID 0x%02X)\n",
                     is_flow_ready ? "OK" : "NG", Flow_getProductID());

    Attitude_init();

    // PID の生成。ゲインは全て要実測(上の定義のコメントを参照)。
    Pid_init(&pid_rate_roll, PID_RATE_ROLL_KP, PID_RATE_ROLL_KI,
             PID_RATE_ROLL_KD, PID_RATE_I_LIMIT, PID_RATE_OUT_LIMIT);
    Pid_init(&pid_rate_pitch, PID_RATE_PITCH_KP, PID_RATE_PITCH_KI,
             PID_RATE_PITCH_KD, PID_RATE_I_LIMIT, PID_RATE_OUT_LIMIT);
    Pid_init(&pid_rate_yaw, PID_RATE_YAW_KP, PID_RATE_YAW_KI, PID_RATE_YAW_KD,
             PID_RATE_I_LIMIT, PID_RATE_OUT_LIMIT);

    // 角度ループは P のみ(ki=kd=0)。i_limit は 0 で「制限なし」だが、
    // ki=0 なので I項は積まれない。
    Pid_init(&pid_angle_roll, PID_ANGLE_ROLL_KP, 0.0f, 0.0f, 0.0f,
             PID_ANGLE_OUT_LIMIT);
    Pid_init(&pid_angle_pitch, PID_ANGLE_PITCH_KP, 0.0f, 0.0f, 0.0f,
             PID_ANGLE_OUT_LIMIT);

    BLE_init(_ble_event_callback);
    BLE_connect();

    USBSerial.println("# 静置してジャイロ較正を待つ(白点滅の間は動かさない)");
    USBSerial.println(
        "# ARM: 両トリガを1秒 / DISARM: 右スティックのボタン / "
        "失陥解除: 左スティックのボタン");
    _Control_printLogHeader();

    if (is_imu_ready) {
        Sound_play(SOUND_PRESET_BOOT);
    }
}

void loop() {
    Timer_sync();  // 400Hz のフレーム同期

    // --- 姿勢推定(400Hz) ------------------------------------------
    if (is_imu_ready) {
        Imu_update();
        Attitude_update(CONTROL_DT);
        // 「新しい姿勢が入った」ことを安全装置に伝える。
        // これが途切れると SAFETY_REASON_IMU_STALE でカットされる。
        Safety_notifyImuUpdate();
    }

    // --- 通信 --------------------------------------------------------
    BLE_update();  // 未接続ならアドバタイズを出し直す
    _Control_updateLink();
    Safety_setLinkConnected(is_link_connected);

    // この周期で使う受信データを固定してから、スティックとボタンを作る。
    // 両者が同じパケット由来であることを保証する。
    _Control_snapshotComm();
    _Control_updateSticks();
    Safety_setThrottle(stick_throttle);  // アーム条件のスロットル最小判定用
    _Control_updateArming(CONTROL_DT);

    // --- 外側ループ(100Hz) ----------------------------------------
    //
    // 姿勢(400Hz)より遅くてよい。センサー自体が ToF 30Hz / Flow 100Hz 程度
    // でしか更新されないので、これより速く回しても同じ値を読むだけになる。
    //
    // 順序が重要: Flow と ToF を読む → 高度を推定 → その高度を使って
    // 速度を推定する。Optical Flow の移動量[px]を速度[m/s]に直すには
    // 高度が要る(同じ流れでも高いほど実距離が大きい)ので、
    // 高度が先に確定していないと速度が出ない。
    const bool is_slow_frame =
        (Timer_getFrameCount() % CONTROL_SLOW_DIVIDER) == 0;
    if (is_slow_frame) {
        if (is_flow_ready) {
            Flow_update();
        }
        if (is_tof_ready) {
            Tof_update();
        }

        // 高度推定と高度制御。ToFが無効な間はスロットルを出さない。
        Altitude_update(CONTROL_SLOW_DT);

        // 速度推定に高度を渡す。
        //
        // Flow の移動量[px]を速度[m/s]に直すには高度が要る(同じ流れでも
        // 高いほど実距離が大きい)。推定が信用できないときは渡さない。
        // 渡されない状態が続けば position.h 側が自分で無効化する。
        const bool is_height_usable =
            (CTRL_FIXED_HEIGHT > 0.0f) ||
            (Altitude_getStatus() == ALTITUDE_STATUS_OK);
        if (is_height_usable) {
            const float height = (CTRL_FIXED_HEIGHT > 0.0f)
                                     ? CTRL_FIXED_HEIGHT
                                     : Altitude_getHeight();
            Position_setHeight(height);
        }

        // 速度推定と位置制御。出力は目標ロール・ピッチ角。
        Position_update(CONTROL_SLOW_DT);
    }

    // --- 安全装置(400Hz) ------------------------------------------
    // 制御を計算する前に見る。ここでアームが落ちれば、この周期から
    // モーターは 0 になる。
    Safety_update(CONTROL_DT);

    // --- 制御と配分(400Hz) ----------------------------------------
    //
    // **モーターへ書く経路はこの if だけ。**
    // Safety_isArmed() が false の間は Mixer_stop() しか通らないので、
    // 較正中も、リンク断後も、起動直後も、プロペラは回らない。
    if (Safety_isArmed()) {
        _Control_update(CONTROL_DT);
    } else {
        Mixer_stop();
        _Control_reset();
    }

    Motor_update();  // ここで初めて PWM に書き出される

    // --- 表示とログ --------------------------------------------------
    // 処理時間は Motor_update() の後で測る。制御に必要な処理が全部
    // 終わった時点の値でないと、間に合っているかの判断にならない。
    const int64_t elapsed = Timer_getElapsedTime();
    if (elapsed > max_elapsed_us) {
        max_elapsed_us = elapsed;
    }

    _Control_updateLed();

    // フロー読み出しと重ならないよう位相をずらす(CTRL_LOG_PHASE 参照)
    const bool is_log_frame =
        (Timer_getFrameCount() % CTRL_LOG_DIVIDER) == CTRL_LOG_PHASE;
    if (is_log_frame) {
        // アーム中は飛行ログ(CSV)、ディスアーム中は机上検証の表示。
        //
        // Why 分けるか: CSVは波形を取るための形式で、机上で「符号が
        //   合っているか」を目で確かめるには読みにくい。飛ぶ前の確認と
        //   飛んでからの記録では、必要な形が違う。
        if (Safety_isArmed()) {
            _Control_printLog();
        } else {
            _Control_printBench();
            max_elapsed_us = 0;  // 次の区間の最大値を取り直す
        }
    }

    LED_update();
    Timer_update();
    Sound_update();
}
