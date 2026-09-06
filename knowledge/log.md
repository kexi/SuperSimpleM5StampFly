# Directory Update Log

## 2026-09-06

* **Creation**: [StampFly Ecosystem](stampfly-ecosystem-reference.md) を追加。CS=G12 の三度目の裏付けと、速度変換の実測定数。
* **Update**: [室内ポジションホールドまでの設計](position-hold-design.md) の K_px を訂正。概算値 0.02 rad/px は実測 0.00222 に対し 1 桁違っていた。
* **Creation**: [BMI270 の SPI 通信仕様](bmi270-spi.md) を追加。
* **Update**: [StampFly の SPI 結線](stampfly-spi-pinout.md) に、公式ファームウェアが G12 を CS として扱っている事実を追記。PMW3901 が存在することの傍証。
* **Creation**: [室内ポジションホールドまでの設計](position-hold-design.md) を追加。実装前の設計のため draft。
* **Update**: [StampFly の SPI 結線](stampfly-spi-pinout.md) を訂正。v1.1 では PMW3901 の実装有無が公式ドキュメント内で矛盾しており未確定。status を draft に戻した。
* **Creation**: [Atom JoyStick のハードウェア構成とピン配置](atom-joystick-hardware.md) を追加。
* **Creation**: [書き込み前に必ず対象デバイスを同定する](flashing-device-identification.md) を追加。Stack-chan を誤って上書きした事故の記録。
* **Creation**: [StampFly の SPI 結線と PMW3901 の CS ピン](stampfly-spi-pinout.md) を追加。回路図から CS2 = G12 を確定。
* **Creation**: [PMW3901 の SPI 通信仕様と疎通確認](pmw3901-driver.md) を追加。実機確認前のため draft。
* **Creation**: [PlatformIO の platform を固定しないとビルドが壊れる](platformio-version-pinning.md) を追加。
