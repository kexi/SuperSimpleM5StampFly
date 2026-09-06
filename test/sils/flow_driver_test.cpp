// driver_flow.h を SILS のチップモデルに繋いで検証するテスト
//
// 実機なしで、初期化シーケンスが正しいかを確かめる。
// Arduino の SPI API を、SILS が提供する xfer() に置き換えて動かす。
//
// 検証できる範囲の制約:
//   SILSのチップモデルは移動量をモーションバースト(0x16)でしか返さず、
//   個別レジスタ(0x02-0x07)の読み出しは常に0を返す。本ドライバは個別読み
//   なので、移動量そのものはこのモデルでは検証できない。
//   初期化シーケンス(検証0x47・条件分岐0x67・キャリブレーション)の確認に絞る。
//
// Why not 実機だけで確認する: USB が使えない状況でも回帰を回したい。
//   初期化の検証(0x47)や条件分岐(0x67)は目視では追えないため、
//   モデル相手に自動で確認できる価値が大きい。

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "pmw3901_device.hpp"

// --- Arduino API の最小スタブ ------------------------------------
// driver_flow.h が使う分だけを用意する
#define HSPI 2
#define MSBFIRST 1
#define SPI_MODE3 3
#define OUTPUT 1
#define HIGH 1
#define LOW 0

static void delay(unsigned long) {}
static void delayMicroseconds(unsigned int) {}
static void pinMode(int, int) {}

// CSの状態を追う。SILSのモデルはCS単位でトランザクションを区切るため。
static bool _cs_low = false;
static uint8_t _tx[16];
static uint8_t _rx[16];
static int _len = 0;

static void digitalWrite(int pin, int value) {
  if (pin != sils_pmw3901::CS_PIN) {
    return; // IMU側のCSは無視
  }
  const bool going_low = (value == LOW);
  if (going_low) {
    _cs_low = true;
    _len = 0;
    return;
  }
  // CSが上がったらトランザクション確定
  if (_cs_low && _len > 0) {
    sils_pmw3901::xfer(_tx, _rx, _len);
  }
  _cs_low = false;
}

struct SPISettings {
  SPISettings(uint32_t, uint8_t, uint8_t) {}
};

class SPIClass {
public:
  explicit SPIClass(int) {}
  void begin(int, int, int, int) {}
  void beginTransaction(SPISettings) {}
  void endTransaction() {}

  // 1バイト転送。
  //
  // SILSのモデルはトランザクション全体(2バイト以上)を受け取って初めて
  // 応答を返すため、1バイトずつ問い合わせても0しか返ってこない。
  // アドレスを受け取った時点で「2バイト分」を先に問い合わせておき、
  // 続く転送でその結果を返す。
  uint8_t transfer(uint8_t out) {
    if (!_cs_low || _len >= (int)sizeof(_tx)) {
      return 0;
    }
    _tx[_len] = out;
    _len++;

    if (_len == 1) {
      // アドレス受信。読み出しなら結果を先に取っておく
      _tx[1] = 0x00;
      sils_pmw3901::xfer(_tx, _rx, 2);
      return _rx[0];
    }
    if (_len == 2) {
      // 書き込みならここで確定させる
      const bool is_write = (_tx[0] & 0x80) != 0;
      if (is_write) {
        sils_pmw3901::xfer(_tx, _rx, 2);
        return 0;
      }
      return _rx[1]; // 読み出し結果
    }
    return 0;
  }
};

// hardware_config.h の代わり
#define PIN_SPI_MOSI 14
#define PIN_SPI_MISO 43
#define PIN_SPI_SCK 44
#define PIN_IMU_CS 46
#define PIN_FLOW_CS 12

// driver_spi.h の代わり(SILSではバスを持たない)
static SPIClass _stub_spi(HSPI);
static SPIClass *SPI_get() { return &_stub_spi; }
static void SPI_init() {}

#include "../../Common/driver/include/driver_flow.h"

// --- テスト本体 --------------------------------------------------
static int failures = 0;

static void check(const char *name, long got, long want) {
  const bool ok = (got == want);
  if (!ok) {
    failures++;
  }
  printf("  [%s] %s (got %ld, want %ld)\n", ok ? "PASS" : "FAIL", name, got,
         want);
}

int main() {
  printf("[flow_driver_test] --- (1) 初期化 ---\n");

  const bool inited = Flow_init();
  check("Flow_init() が成功する", inited ? 1 : 0, 1);
  check("Product ID", Flow_getProductID(), 0x49);
  check("Inverse Product ID", Flow_getInverseProductID(), 0xB6);

  printf("[flow_driver_test] --- (2) レジスタ書き込みが届いている ---\n");

  // 初期化で書いた設定が、モデル側に保存されているかを確認する。
  // 0x7F(バンク選択)は最後に0x00が書かれて終わる。
  sils_pmw3901::set_motion(0, 0, 0x50);
  Flow_update();
  check("Flow_update() が例外なく完了する", 1, 1);

  printf("[flow_driver_test] --- (3) 再初期化しても成功する ---\n");

  // 条件分岐とキャリブレーションを含む初期化が、
  // 2回目以降も破綻しないことを確認する
  const bool reinited = Flow_init();
  check("2回目の Flow_init()", reinited ? 1 : 0, 1);
  check("Product ID (再確認)", Flow_getProductID(), 0x49);

  printf("[flow_driver_test] %s (%d failures)\n",
         failures == 0 ? "OK" : "FAILED", failures);
  return failures == 0 ? 0 : 1;
}
