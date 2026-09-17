# sf_snn_core — shared CUBA-LIF neuron math (CubaLifLayer, LinearReadout,
# snn_weight_init) used by BOTH sf_estimator_snn and sf_controller_snn.
# Pure math, no Pub-Sub/data_types dependency, so no REQUIRES beyond the
# standard toolchain (matches sf_math's header-only pattern; this one adds
# .cpp sources because the layer carries per-instance state).
#
# sf_snn_core — sf_estimator_snn と sf_controller_snn の両方が使う共有
# CUBA-LIF ニューロン数学（CubaLifLayer, LinearReadout, snn_weight_init）。
# 純粋な数学で Pub-Sub/data_types に依存しないため標準ツールチェーン以外の
# REQUIRES は無い（sf_math のヘッダーのみパターンと同じ方針。層がインスタンス
# ごとの状態を持つため .cpp を追加している点のみ異なる）。
idf_component_register(
    SRCS
        "cuba_lif_layer.cpp"
        "linear_readout.cpp"
    INCLUDE_DIRS "include"
)
