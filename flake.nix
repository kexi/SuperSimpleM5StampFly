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
      in
      {
        formatter = treefmtEval.config.build.wrapper;

        checks.formatting = treefmtEval.config.build.check self;

        devShells.default = pkgs.mkShell {
          packages = with pkgs; [
            # ビルド・書き込み(esptoolはPlatformIOが内蔵版を使うので入れない)
            platformio

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
