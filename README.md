# SuperSimpleM5StampFly

M5Stack社が発売した StampFly と AtomJoyStick のファームウェアをゼロから書いてみよう！！という無謀な試み。飛んだら成功！飽きたら終了！

## 方針

- なるべくシンプルにする
- まずはIMU姿勢制御して飛ばすところまでやる
- ToF・OpticalFlow・気圧は余裕があれば手を出す
- デバイスごとにドライバを分ける
- StampFly と AtomJoyStick で共通化できる部分は共通化する

## ToDo

- [ ] アホほどある

## 進捗

「はじめの N 歩」として一歩ずつ足していく。全体の設計は
[knowledge/position-hold-design.md](knowledge/position-hold-design.md) を参照。

|歩|内容|状態|
|----|----|----|
|〜二十一|BLEでジョイスティックを受けてモーターを回す|実機で動作確認済み|
|二十二|PMW3901の製品ID読み出し|実機未確認|
|二十三|SPIバスの共有化、BMI270のチップID読み出し|実機未確認|
|二十四|BMI270の設定書き込みと加速度・角速度、400Hz化|実機未確認|
|二十五|姿勢推定(相補フィルタ)|実機未確認|
|二十六|安全装置(アーム・フェイルセーフ)|実機未確認|
|二十七|PIDとミキサー|実機未確認|
|二十八|姿勢制御(手動ホバリング)|実機未確認|
|二十九|ToF(VL53L3CX)の距離読み出し|実機未確認|
|三十|高度推定と高度ホールド|実機未確認|
|三十二〜三十四|速度推定と**ポジションホールド**|実機未確認|

**コードとしては一通り繋がったが、実機では一度も飛ばしていない。**
PIDゲインは全て「要実測」で、飛ばしながら詰める前提の出発点でしかない。

## 制御の構成

カスケード制御。外側ほど遅く、外側の出力は内側の目標値になる。

```
位置P → 速度PI → 角度P → 角速度PID → ミキサー → モーター
 (100Hz)         (400Hz)              ↑
                                  Safety_isArmed()
```

- 姿勢推定・高度推定は相補フィルタ（Kalmanは使わない。ホバリングの狭い動作域では
  精度差が出ない一方、コード量と説明コストが10倍違うため）
- Optical Flowは**回転成分を除去**してから速度に変換する。省くと
  「傾く→流れを検出→補正で傾く」の正帰還で発振する
- モーター出力は `Safety_isArmed()` を唯一の経路とし、較正が終わるまでアームできない

## 飛ばすときの注意

- **プロペラを付ける前に、机の上で各軸の符号を確認する**
- 最初はテザー（糸を機体中央に結んで手で保持）から始める
- クッションの上、人は2m以上離れる、保護メガネ
- ゲインはCSVログを見ながら少しずつ詰める

## 開発環境

Nix + direnv で環境を用意している。`direnv allow` すると必要なツールが揃う。

```sh
direnv allow          # 初回のみ
just install-hooks    # 初回のみ(git hooksの導入)
just --list           # 使えるコマンド一覧
just build            # StampFlyのビルド
just flash-monitor    # 書き込んでシリアルモニタを開く
just test-sils        # 実機なしでドライバを検証する
```

### 書き込み前の注意

**StampFly も Stack-chan(CoreS3) も同じ ESP32-S3 で、`pio device list` の
VID:PID は両方 `303A:1001`。この出力からは区別できない。**
書き込み先の MAC アドレス(`SER=`)を必ず確認すること。
過去に取り違えて別デバイスを上書きした事故がある
（[詳細](knowledge/flashing-device-identification.md)）。

### 実機なしでの検証

[StampFly Ecosystem](https://github.com/M5Fly-kanazawa/stampfly_ecosystem) の
SILS にセンサーのチップモデルがあり、`just test-sils` でドライバの初期化
シーケンスをホスト上で検証できる（[詳細](knowledge/sils-emulator.md)）。

```sh
ghq get https://github.com/M5Fly-kanazawa/stampfly_ecosystem
just test-sils
```

## ナレッジ

調査・計測で得た知見は [knowledge/](knowledge/index.md) に
[OKF v0.2](https://github.com/GoogleCloudPlatform/open-knowledge-format) で残している。
**効かなかったことや、誤りと判明した結論も消さずに残す。**

## 参考にした実装

いずれも MIT License。出典を明記した上で利用している。

- [StampFly Ecosystem](https://github.com/M5Fly-kanazawa/stampfly_ecosystem) (Copyright (c) 2026 Kouhei Ito)
  - PMW3901 の初期化手順（PixArt 公式実装ガイド）
  - BMI270 の設定ファイル(8192バイト、Bosch 配布)
  - SILS のセンサーチップモデル（ドライバ検証用）

## StampFly のSPI結線

IMU(BMI270)とオプティカルフローセンサ(PMW3901)は同じSPIバスを共有し、CSピンで区別する。
そのため片方と通信する前に、もう片方のCSをHIGH(非選択)にしておく必要がある。

|信号|GPIO|
|----|----|
|MOSI|14|
|MISO|43|
|SCK|44|
|CS (BMI270)|46|
|CS2 (PMW3901)|12|

CS2 = G12 は [公式回路図](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1069/Stamp_Fly_v1.0.pdf)・
公式ファームウェア・StampFly Ecosystem の3つで裏付けを取った
（Web上の記載はG46とG12で食い違っていた）。詳細は
[knowledge/stampfly-spi-pinout.md](knowledge/stampfly-spi-pinout.md)。


## 参考資料

- StampFly & Atom ジョイスティック ファームウェア書き込みガイド https://docs.m5stack.com/ja/guide/hobby_kit/stampfly/stamply_firmware

### M5 Stamp Fly関連

- オリジナルファームウェア https://github.com/m5stack/M5StampFly
  （**PMW3901 は未実装**。SPIで読んでいるのはBMI270のみ）
- 教育・研究プラットフォーム https://github.com/M5Fly-kanazawa/stampfly_ecosystem
  （オリジナルファームウェア作者による。PMW3901の実装・ESKF・シミュレータ・SILS）

下表は v1.0 の仕様。手元の実機は **v1.1**（メインモジュールが Stamp-S3A に変更）で、
GPIO配置は同一だがバッテリが320mAh、重量が27.6gになっている。
**v1.1の製品特徴リストからは PMW3901 の記載が消えているが、ピンマップと回路図リンクには
残っている**（公式ドキュメント内で矛盾。実機で確認が必要）。

|仕様|概要|
|----|----|
|M5StampS3|ESP32-S3@Xtensa LX7、8 MB-FLASH、Wi-Fi、OTG/CDC support|
|距離センサ|VL53L3CXV0DH/1 (0x52) @ 最大3m|
|オプティカルフローセンサ|PMW3901MB-TXQT|
|気圧センサ|BMP280（0x76）@ 300-1100 hPa|
|3軸磁力センサ|BMM150（0x10)|
|6軸IMUセンサ|BMI270|
|バッテリー|300 mAh 高電圧リチウムポリマーバッテリ（LiHV）|
|電流電圧計|INA3221AIRGVR（0x40）|
|ブザー|Built-in Buzzer @ 5020|
|製品サイズ|107 x 107 x 30 mm|
|製品重量|36.2 g|

### M5 Atom JoyStick関連

- オリジナルファームウェア https://github.com/m5stack/Atom-JoyStick

|仕様|概要|
|----|----|
|マイコン|STM32F030F4P6|
|RGB LED|WS2812C|
|充電IC|TP4067 4.35 V|
|バッテリ|300 mAh 高電圧リチウムポリマーバッテリ（LiHV）|
|充電電流|	5V / 500 mA|
|ブザー|	Built-in Buzzer @ 5020|
|製品サイズ|84 x 60 x 30 mm|
|製品重量|63.5g|
