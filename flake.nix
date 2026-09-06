{
  description = "SuperSimpleM5StampFly - StampFly / AtomJoyStick firmware from scratch";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    treefmt-nix.url = "github:numtide/treefmt-nix";
    treefmt-nix.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      treefmt-nix,
      ...
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = nixpkgs.legacyPackages.${system};

        treefmtEval = treefmt-nix.lib.evalModule pkgs ./treefmt.nix;

        # nixpkgs の platformio は Linux では bubblewrap(bwrap) による FHS
        # ラッパー。GitHub Actions のランナーでは uid map を張る権限が無く
        # "bwrap: setting up uid map: Permission denied" で落ちる。
        # ラッパーを経由しない platformio-core を使う。
        # macOS では両者が同じ derivation なので違いは出ない。
        platformio = if pkgs.stdenv.isLinux then pkgs.platformio-core else pkgs.platformio;
      in
      {
        formatter = treefmtEval.config.build.wrapper;

        checks.formatting = treefmtEval.config.build.check self;

        devShells.default = pkgs.mkShell {
          packages = with pkgs; [
            # ビルド・書き込み(esptoolはPlatformIOが内蔵版を使うので入れない)
            platformio
            # platformio-core が呼ぶ外部ツール
            python3
            git

            # タスクランナー / git hooks
            just
            lefthook

            # 整形・静的解析
            clang-tools
            nixfmt

            # ナレッジ(OKF)の検証
            yq-go

            # セキュリティ / CI
            gitleaks
            shellcheck
            actionlint
            pinact
          ];

          shellHook = ''
            # PlatformIO のツールチェーンはリポジトリ内に閉じ込めて、
            # マシン全体の ~/.platformio と混ざらないようにする
            export PLATFORMIO_CORE_DIR="$PWD/.platformio"

            echo "SuperSimpleM5StampFly dev shell"
            echo "  just --list で使えるコマンドを表示します"
          '';
        };
      }
    );
}
