---
okf_version: "0.2"
---

# SuperSimpleM5StampFly ナレッジ索引

## ハードウェア

* [StampFly の SPI 結線と PMW3901 の CS ピン](stampfly-spi-pinout.md) - IMU(BMI270) と オプティカルフローセンサ(PMW3901) が共有する SPI バスの GPIO 割り当て。CS2 は G12。ただし v1.1 では PMW3901 が実装されているか公式ドキュメント内で情報が矛盾しており、実機確認が必要。
* [BMI270 の SPI 通信仕様](bmi270-spi.md) - IMU(BMI270) のレジスタアクセス手順。読み出しにダミーバイトが1つ挟まる点と、電源投入直後の I2C→SPI モード切替が要点。
* [PMW3901 の SPI 通信仕様と疎通確認](pmw3901-driver.md) - オプティカルフローセンサ PMW3901 のレジスタアクセス手順、SPI モード・クロック、製品 ID による疎通確認の方法。

* [Atom JoyStick のハードウェア構成とピン配置](atom-joystick-hardware.md) - AtomS3 と STM32F030F4P6 の 2 チップ構成。ジョイスティックとボタンは STM32 側に繋がり、AtomS3 からは I2C(0x59) 越しに読む。

## 参照実装

* [StampFly Ecosystem — Optical Flow の実測値が得られる参照実装](stampfly-ecosystem-reference.md) - Kouhei Ito 氏(オリジナルファームウェア作者)による教育・研究プラットフォーム。PMW3901 の完全な実装と、速度変換に必要な実測定数が揃っている。

## 設計

* [室内ポジションホールドまでの設計（はじめの二十三〜三十四歩）](position-hold-design.md) - 姿勢制御からポジションホールドまでを12段階に分ける設計。推定は全て相補フィルタ、単一ループ400Hz、外側は間引き。二十八歩の手動ホバリングまではOptical Flowに依存しない。

## 実機作業

* [書き込み前に必ず対象デバイスを同定する](flashing-device-identification.md) - ESP32-S3 は StampFly も CoreS3(Stack-chan) も同じ VID:PID を返すため、pio device list だけでは区別できない。同定せずに書き込んで別デバイスを上書きした事故の記録。

## ビルド・開発環境

* [PlatformIO の platform を固定しないとビルドが壊れる](platformio-version-pinning.md) - platform = espressif32 をバージョン無しで書くと解決先が src-<hash> 版にぶれて TypeError でビルドが失敗する。@6.13.0 に固定して解消した。
