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

## 開発環境

Nix + direnv で環境を用意している。`direnv allow` すると必要なツールが揃う。

```sh
direnv allow          # 初回のみ
just install-hooks    # 初回のみ(git hooksの導入)
just --list           # 使えるコマンド一覧
just build            # StampFlyのビルド
just flash-monitor    # 書き込んでシリアルモニタを開く
```

## 参考にした実装

- PMW3901 の性能設定レジスタ列: [Bitcraze_PMW3901](https://github.com/bitcraze/Bitcraze_PMW3901) (MIT License, Copyright (c) 2017 Bitcraze AB)

## StampFly のSPI結線

IMU(BMI270)とオプティカルフローセンサ(PMW3901)は同じSPIバスを共有し、CSピンで区別する。
値は[公式回路図](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1069/Stamp_Fly_v1.0.pdf)で確認したもの。

|信号|GPIO|
|----|----|
|MOSI|14|
|MISO|43|
|SCK|44|
|CS (BMI270)|46|
|CS2 (PMW3901)|12|


## 参考資料

- StampFly & Atom ジョイスティック ファームウェア書き込みガイド https://docs.m5stack.com/ja/guide/hobby_kit/stampfly/stamply_firmware

### M5 Stamp Fly関連

- オリジナルファームウェア https://github.com/m5stack/M5StampFly

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
