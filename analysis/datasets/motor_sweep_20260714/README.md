# Motor sweep current dataset (2026-07-14)

`sf motor sweep --duty 40 --sec 3` outputs from three airframes
(shiro2 = 白2, mujirushi = 無印, ki3 = 黄3), props off / props on,
battery near full (4.26-4.34 V). Collected to investigate the
product-inherent CW/CCW duty asymmetry seen in flight logs.

3個体（白2・無印・黄3）の `sf motor sweep` 出力。プロペラ無し/有りの
2条件、満充電付近。飛行ログで見られる製品固有の CW/CCW duty 非対称の
原因調査用。

Key result / 主要結果: props-on group current difference (CCW-CW) is
only -0.5% / +2.0% / +5.6% across the three units — far below the ~40%
torque asymmetry the flight trim would require. Motor-direction and
prop-torque(Cq) group asymmetry hypotheses are refuted; the leading
remaining hypothesis is aerodynamic interaction of the assembled craft
(4-prop wake on the frame), to be tested by a string-suspension
`motor all` test and per-unit hover logs.
