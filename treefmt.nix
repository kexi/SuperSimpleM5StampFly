# treefmt-nix の設定。`nix fmt` でリポジトリ全体を一括整形する。
{ pkgs, ... }:
{
  projectRootFile = "flake.nix";

  programs = {
    # Nix
    nixfmt.enable = true;

    # C / C++ (StampFly/.clang-format と Common/driver/include/.clang-format を使う)
    clang-format.enable = true;

    # justfile
    just.enable = true;
  };

  settings.formatter.clang-format.includes = [
    "*.c"
    "*.h"
    "*.cpp"
    "*.hpp"
  ];
}
