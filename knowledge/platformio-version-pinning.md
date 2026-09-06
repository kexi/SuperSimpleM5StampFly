---
type: Playbook
title: PlatformIO の platform を固定しないとビルドが壊れる
description: platform = espressif32 をバージョン無しで書くと解決先が src-<hash> 版にぶれて TypeError でビルドが失敗する。@6.13.0 に固定して解消した。
tags: [build, toolchain, pitfall]
status: stable
generated: { by: claude-opus-5/1m, at: 2026-09-06T02:30:00Z }
verified:
  - { by: claude-opus-5/1m, at: 2026-09-06T02:30:00Z }
sources:
  - id: repro-build
    resource: 本リポジトリ StampFly/ での pio run 実行（2026-09-06、macOS 26 / aarch64-darwin、PlatformIO 6.1.19）
    title: ビルド失敗と修正の実測
---

# 症状

`pio run` が次で落ちる。コンパイル前、フレームワーク解決の段階。

```
TypeError: argument should be a str or an os.PathLike object
where __fspath__ returns a str, not 'NoneType':
  ...
  File ".../builder/frameworks/arduino.py", line 524:
    build_script_path = str(Path(FRAMEWORK_DIR) / "tools" / "pioarduino-build.py")
```

`FRAMEWORK_DIR` が `None` になっている。

# 原因

`platformio.ini` に `platform = espressif32` とだけ書くと、
インストール済みの複数バージョンのうちどれに解決されるかが安定しない。

`~/.platformio/platforms/` に次が同居していた:[^repro-build]

```
espressif32
espressif32@6.13.0
espressif32@7.0.1
espressif32@src-4380a9ee0fc3d1e47feece1185fd2391
espressif32@src-596c9938872833461cb0fe93e12b1b9f
```

`src-<hash>` 版に解決され、それが `framework-arduinoespressif32` を
見つけられずに `FRAMEWORK_DIR = None` となっていた。
**パッケージ自体は存在していた**ので、「入っていないから」ではない。

# 対処

バージョンを固定する。

```ini
[env]
; ツールチェーンの解決先がぶれてビルドが壊れないよう、バージョンを固定する
platform = espressif32@6.13.0
```

これで解決。副作用として、環境が変わってもビルドが再現するようになる。

# 見分け方

**コンパイルエラーではなく Python の TypeError / AttributeError で落ちたら、
コードではなくツールチェーン解決を疑う。** ソースを直しても意味がない。

`ls ~/.platformio/platforms/` に `@src-<hash>` が混ざっていたら、この症状の候補。

# Why not — 毎回クリーンインストールする

`~/.platformio` を消せばその場は直るが、次に別プロジェクトが
別バージョンを入れた瞬間に再発する。固定する方が安い。

なお Nix devShell では `PLATFORMIO_CORE_DIR` をリポジトリ内に向けて、
マシン全体の `~/.platformio` と混ざらないようにもしている。

[^repro-build]: 本リポジトリ StampFly/ での pio run 実行
