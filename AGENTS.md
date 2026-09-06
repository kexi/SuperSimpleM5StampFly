# SuperSimpleM5StampFly

M5Stack の StampFly と AtomJoyStick のファームウェアをゼロから書く実験リポジトリ。
チュートリアル形式で「はじめの N 歩」として少しずつ機能を足していく。

## 作業を始める前に

**`knowledge/` の OKF ドキュメントを確認する。** まず [索引](knowledge/index.md)
を読み、これから触る領域の既知の知見と過去の誤りを踏まえてから着手する。

同じ調査を二度やらないため、そして一度踏んだ落とし穴を二度踏まないため。

## 実機を扱うときの鉄則

**フラッシュ書き込みの前に、対象デバイスを必ず同定する。**

StampFly も Stack-chan (CoreS3) も同じ ESP32-S3 で、`pio device list` の
VID:PID は両方 `303A:1001`。この出力からは製品を区別できない。
過去に取り違えて別デバイスを上書きした事故がある
（[詳細](knowledge/flashing-device-identification.md)）。

- 書き込み先の MAC アドレス (`SER=`) を確認する
- 「1 台しか見えないからこれだろう」で実行しない
- 確信が持てなければ人に聞く。フラッシュ書き込みは元に戻しにくい

## 開発環境

Nix + direnv。`direnv allow` で必要なツールが揃う。

```sh
direnv allow          # 初回のみ
just install-hooks    # 初回のみ(git hooks の導入)
just --list           # 使えるコマンド一覧
```

コマンドは justfile に集約する。各レシピには説明コメントを必ず付ける。

## 構成

```
Common/driver/include/   # StampFly と JoyStick で共有するドライバ(header-only)
StampFly/                # StampFly 側の PlatformIO プロジェクト
JoyStick/                # AtomJoyStick 側の PlatformIO プロジェクト
knowledge/               # OKF v0.2 ナレッジ
bin/                     # lint スクリプト
```

## コードの書き方

- ドライバはデバイスごとに分け、`Xxx_init` / `Xxx_update` で揃える
- 実装はヘッダに直接書く(header-only)。既存の `driver_*.h` に倣う
- コメントは日本語。**コードには How、コメントには Why not** を書く
- ピン定義は `StampFly/include/hardware_config.h` に集約する
- `if` の条件式には名前を付ける(`const bool is_xxx = ...`)
- 関数内では早期リターンを心がける

## コミット

- Semantic Commit Messages に従う
- **コミットログには Why(なぜ変更したか)** を書く
- main へ直接コミットせず、ブランチを切る

## 知見を残す

調査・計測で得た知見は `knowledge/` に OKF v0.2 で記録する。

- **効かなかったことこそ残す。** 期待・実際・なぜ外れたかを書く
- 数字には測定条件を添える。条件の無い数字は条件が変わった瞬間に嘘になる
- 誤りと判明した結論は、消さずに訂正を並べて残す
- tag は `knowledge/tags.yml` の統制語彙のみ。新しい tag は説明付きで先に追加する

書き方の詳細は `okf` skill を参照。
