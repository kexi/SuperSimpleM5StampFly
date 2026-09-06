# SILS を使ったホストテスト

実機なしで `Common/driver/include/` のドライバを検証する。

## 何を検証できるか

`flow_driver_test.cpp` は PMW3901 ドライバの**初期化シーケンス**を検証する。
公式手順は検証(`0x47`)・条件分岐(`0x67`)・個体ごとのキャリブレーション計算を
含み、目視では追えないため、モデル相手に自動で確認する価値が大きい。

**検証できないこと**: 移動量そのもの。SILS のチップモデルは移動量を
モーションバースト(`0x16`)でしか返さず、本ドライバが使う個別レジスタ
読み出し(`0x02`-`0x07`)は常に 0 を返すため。実機で確認する。

## 前提

[StampFly Ecosystem](https://github.com/M5Fly-kanazawa/stampfly_ecosystem)
(MIT License) を clone しておく。

```sh
ghq get https://github.com/M5Fly-kanazawa/stampfly_ecosystem
```

## 実行

```sh
just test-sils
```

環境変数 `STAMPFLY_ECOSYSTEM` で場所を指定できる（未指定なら ghq の既定位置）。
