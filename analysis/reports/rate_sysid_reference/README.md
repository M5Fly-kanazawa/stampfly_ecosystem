# rate_sysid_reference — SILモデル一致ゲートの実機基準値

`docs/architecture/simulation-policy.md` §4「モデル一致ゲート」が使う、実機同定の基準値。

## 出所

`reference.json` は以下2本の実飛行システム同定 run の `metrics.json`（`sysid.<axis>.fit`）
から、`make_reference.py` が機械的に生成した（手打ちなし）。

- `analysis/reports/altlog_20260614T201629/metrics.json`
- `analysis/reports/altlog_20260614T214537/metrics.json`

各軸について、2 run の `b`（1/有効慣性）と `L_total = T + L`（むだ時間＋モータ遅れの合計）
を単純平均している。

## なぜ `L_total = T + L` であって `T`・`L` 個別ではないか

`rate_sysid.py` の `fit_plant()` は `G(s) = b·e^{-Ls}/(s(Ts+1))` の3パラメータを
Nelder-Mead でフィットするが、`T` と `L` の分離は退化しやすい（例: roll/yaw では
`T` が `~1e-10 s` に潰れ、遅れが全て `L` 側に寄る）。一方で **合計遅れ `L_total`
は2 run 間で安定**（roll ≈14.1〜14.7ms、pitch ≈16.2〜18.3ms、yaw ≈10.6〜11.0ms）。
そのため、ゲートの合否判定には `L_total` を使い、`T`・`L` 個別の値は参考情報として
`runs` 配下に残すのみとする。

## 注意: ヨー基準値の信頼度

ヨーの基準値は3パラメータフィット（反トルク零点を含まないモデル）由来で、コヒーレンスも
0.44〜0.54 と低い。ヨー軸は4パラメータ（LHP零点つき）モデルの方が有意に良いことが
分かっている（`firmware/vehicle/docs/yaw_axis_model.md`）。ヨー行は3軸の中で最も弱い
基準値として扱い、4パラモデルでの基準再導出を将来の更新候補とする。

## 更新方法

新しい実機同定 run（`sf sysid rate-fit` 等で生成した `metrics.json` を持つ
`analysis/reports/altlog_*`）が増えたら、`make_reference.py` の `RUNS` リストに
追記して再実行する:

```bash
python3 analysis/reports/rate_sysid_reference/make_reference.py
```

`reference.json` がその場で再生成される。これにより **ゲート基準は実機飛行の蓄積と
ともに更新される**（本書は起動時の一点物ではない）。

## 使用箇所

`sf sil sysid-gate`（`lib/sfcli/commands/sil.py`）が本 `reference.json` を読み、
`simulator/sil/scenarios/sysid_gate.scn` の SIL 実行結果（`tools/log_analyzer/rate_sysid.py`
— 実機同定と同一コード — でフィット）と比較して合否を判定する。

許容差（`docs/architecture/simulation-policy.md` §4）:

| 指標 | 許容差 |
|---|---|
| `b`（1/有効慣性） | ±50% |
| `L_total`（むだ時間＋モータ遅れの合計） | ±20% |

`T`・`L` 個別の内訳とコヒーレンスは参考表示のみで、合否判定には使わない。
