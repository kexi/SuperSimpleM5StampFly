---
okf_version: "0.2"
---

# SuperSimpleM5StampFly ナレッジ索引

## ハードウェア

* [StampFly の SPI 結線と PMW3901 の CS ピン](stampfly-spi-pinout.md) - IMU(BMI270) と オプティカルフローセンサ(PMW3901) が共有する SPI バスの GPIO 割り当て。CS2 は G12（Web 上の記載は G46 と食い違うが回路図で確定）。
* [PMW3901 の SPI 通信仕様と疎通確認](pmw3901-driver.md) - オプティカルフローセンサ PMW3901 のレジスタアクセス手順、SPI モード・クロック、製品 ID による疎通確認の方法。

## ビルド・開発環境

* [PlatformIO の platform を固定しないとビルドが壊れる](platformio-version-pinning.md) - platform = espressif32 をバージョン無しで書くと解決先が src-<hash> 版にぶれて TypeError でビルドが失敗する。@6.13.0 に固定して解消した。
