# 既定のターゲット。使えるコマンド一覧を表示する
default:
    @just --list

# StampFly のファームウェアをビルドする
build:
    cd StampFly && pio run

# StampFly のファームウェアをビルドして書き込む
flash:
    cd StampFly && pio run --target upload

# StampFly のシリアルモニタを開く
monitor:
    cd StampFly && pio device monitor

# StampFly を書き込んでそのままシリアルモニタを開く
flash-monitor:
    cd StampFly && pio run --target upload && pio device monitor

# AtomJoyStick のファームウェアをビルドする
build-joystick:
    cd JoyStick && pio run

# AtomJoyStick のファームウェアをビルドして書き込む
flash-joystick:
    cd JoyStick && pio run --target upload

# ビルド成果物を削除する
clean:
    cd StampFly && pio run --target clean

# リポジトリ全体を整形する
fmt:
    nix fmt

# 整形済みかを検証する（CI / hook 用）
fmt-check:
    nix flake check

# 秘密情報が混入していないか検査する
secret-scan:
    gitleaks detect --no-banner

# GitHub Actions の定義を検証する
lint-actions:
    actionlint

# GitHub Actions が SHA で pin されているか検証する
pin-check:
    pinact run --check

# ナレッジ(OKF)の形式を検証する
okf-lint:
    ./bin/okf-lint
    ./bin/okf-lint-tags

# シェルスクリプトを静的解析する
lint-shell:
    shellcheck bin/*

# git hooks をインストールする
install-hooks:
    lefthook install
