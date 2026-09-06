---
type: Playbook
title: 書き込み前に必ず対象デバイスを同定する
description: ESP32-S3 は StampFly も CoreS3(Stack-chan) も同じ VID:PID を返すため、pio device list だけでは区別できない。同定せずに書き込んで別デバイスを上書きした事故の記録。
tags: [stampfly, pitfall, hardware]
status: stable
generated: { by: claude-opus-5/1m, at: 2026-09-06T06:50:00Z }
verified:
  - { by: claude-opus-5/1m, at: 2026-09-06T02:45:00Z }
  - { by: human:kexi, at: 2026-09-06T06:50:00Z }
sources:
  - id: incident-20260906
    resource: 本リポジトリでの作業（2026-09-06、macOS 26 / aarch64-darwin）
    title: Stack-chan を誤って上書きした事故の実測
---

# 何が起きたか

StampFly のファームウェアを、接続されていた **Stack-chan (M5Stack CoreS3) に
書き込んでしまった。**[^incident-20260906]

`pio device list` の出力はこうだった:

```
/dev/cu.usbmodem2101
Hardware ID: USB VID:PID=303A:1001 SER=44:1B:F6:DF:59:68 LOCATION=2-1
Description: USB JTAG/serial debug unit
```

ESP32-S3 が 1 台だけ見えたので StampFly だと判断した。**間違いだった。**

# なぜ区別できなかったか

**VID:PID=303A:1001 は「Espressif の USB JTAG/serial」であって、製品の識別子ではない。**

| デバイス | MCU | VID:PID |
|---|---|---|
| StampFly | ESP32-S3 (StampS3) | 303A:1001 |
| Stack-chan | ESP32-S3 (CoreS3) | 303A:1001 |
| その他 S3 系 | ESP32-S3 | 303A:1001 |

`Description` も `USB JTAG/serial debug unit` で共通。**この出力からは製品を
一意に決められない。** 唯一デバイス固有なのは `SER=` の MAC アドレスだけ。

# 対処

**書き込み前に、対象が意図したデバイスかを必ず確認する。**

1. **MAC アドレスを控えておく。** `SER=` が各機体固有。一度同定したら
   このリポジトリの表に追記する
2. **1 台だけ挿す。** 複数挿すと取り違える
3. **迷ったら人に聞く。** フラッシュ書き込みは元に戻しにくい。
   「たぶんこれだろう」で実行しない

## 既知のデバイス

| MAC | デバイス |
|---|---|
| `44:1B:F6:DF:59:68` | Stack-chan (CoreS3) |
| `48:CA:43:B6:59:C0` | **StampFly (v1.1)** |

**2 台とも `VID:PID=303A:1001` / `Description: USB JTAG/serial debug unit` で、
MAC 以外に区別する手がかりは無い。** 同時に挿すと `pio device list` の
並び順も固定ではないので、ポート名（`/dev/cu.usbmodem*`）を覚えても当てにならない。

# なぜ「1 台しか見えないから、それだ」が誤りか

**「他に候補が見えない」は「これが目的のデバイスだ」を意味しない。**
目的のデバイスが繋がっていないだけかもしれない。実際この事故では
StampFly は繋がっておらず、無関係な Stack-chan だけが見えていた。

存在しないものを探すのではなく、**見えているものが何かを確認する**。

# 被害と復旧

- Stack-chan のフラッシュが StampFly 用ファームウェアで上書きされた
- **書き込み前のバックアップを取っていなかった**ため、元の内容は失われた
- ハードウェアの損傷は無い（書き込んだコードはモーター駆動を行わない）

**書き込み前に `esptool.py read_flash` でバックアップを取れば、
この種の事故は復旧可能になる。** 今回は取っていなかった。

[^incident-20260906]: 本リポジトリでの作業での事故の実測
