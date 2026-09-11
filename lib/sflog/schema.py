"""
GENERATED FILE - do not edit; run protocol/tools/gen_flight_log.py
生成ファイル - 手で編集しないこと。protocol/tools/gen_flight_log.py を実行して再生成する。

StampFly flight-log v1 schema constants.
StampFly フライトログ v1 形式のスキーマ定数。

Source of truth / 正本: protocol/spec/flight_log.yaml
"""

FORMAT = 'stampfly-flight-log'
VERSION = 1

# Stream name -> {file, source, nominal_rate_hz, required, columns}.
# columns is a tuple of (name, type, unit); the fuller per-column
# metadata (description in both languages) lives in _COLUMN_META
# below and is exposed through schema_for().
# ストリーム名 -> {file, source, nominal_rate_hz, required, columns}。
# columns は (name, type, unit) のタプル。説明文（日英）を含む
# より詳細な列メタデータは下の _COLUMN_META にあり、schema_for()
# 経由で取得する。
STREAMS = {   'imu': {   'file': 'imu.csv',
               'source': 'IMU+ESKF (0x40)',
               'nominal_rate_hz': 400,
               'required': True,
               'columns': (   ('timestamp_us', 'int', 'us'),
                              ('seq', 'int', 'n/a'),
                              ('gyro_x', 'float', 'rad/s'),
                              ('gyro_y', 'float', 'rad/s'),
                              ('gyro_z', 'float', 'rad/s'),
                              ('accel_x', 'float', 'm/s^2'),
                              ('accel_y', 'float', 'm/s^2'),
                              ('accel_z', 'float', 'm/s^2'),
                              ('gyro_raw_x', 'float', 'rad/s'),
                              ('gyro_raw_y', 'float', 'rad/s'),
                              ('gyro_raw_z', 'float', 'rad/s'),
                              ('accel_raw_x', 'float', 'm/s^2'),
                              ('accel_raw_y', 'float', 'm/s^2'),
                              ('accel_raw_z', 'float', 'm/s^2'))},
    'attitude': {   'file': 'attitude.csv',
                    'source': 'IMU+ESKF (0x40)',
                    'nominal_rate_hz': 400,
                    'required': False,
                    'columns': (   ('timestamp_us', 'int', 'us'),
                                   ('seq', 'int', 'n/a'),
                                   ('quat_w', 'float', 'unitless'),
                                   ('quat_x', 'float', 'unitless'),
                                   ('quat_y', 'float', 'unitless'),
                                   ('quat_z', 'float', 'unitless'),
                                   ('gyro_bias_x', 'float', 'rad/s'),
                                   ('gyro_bias_y', 'float', 'rad/s'),
                                   ('gyro_bias_z', 'float', 'rad/s'),
                                   ('accel_bias_x', 'float', 'm/s^2'),
                                   ('accel_bias_y', 'float', 'm/s^2'),
                                   ('accel_bias_z', 'float', 'm/s^2'))},
    'posvel': {   'file': 'posvel.csv',
                  'source': 'PosVel (0x41)',
                  'nominal_rate_hz': 400,
                  'required': False,
                  'columns': (   ('timestamp_us', 'int', 'us'),
                                 ('seq', 'int', 'n/a'),
                                 ('pos_x', 'float', 'm'),
                                 ('pos_y', 'float', 'm'),
                                 ('pos_z', 'float', 'm'),
                                 ('vel_x', 'float', 'm/s'),
                                 ('vel_y', 'float', 'm/s'),
                                 ('vel_z', 'float', 'm/s'))},
    'rate_ref': {   'file': 'rate_ref.csv',
                    'source': 'RateRef (unified packet 0x50 fixed part)',
                    'nominal_rate_hz': 400,
                    'required': False,
                    'columns': (   ('timestamp_us', 'int', 'us'),
                                   ('seq', 'int', 'n/a'),
                                   ('rate_ref_roll', 'float', 'rad/s'),
                                   ('rate_ref_pitch', 'float', 'rad/s'),
                                   ('rate_ref_yaw', 'float', 'rad/s'))},
    'motor': {   'file': 'motor.csv',
                 'source': 'Duty400 (0x4A)',
                 'nominal_rate_hz': 400,
                 'required': False,
                 'columns': (   ('timestamp_us', 'int', 'us'),
                                ('seq', 'int', 'n/a'),
                                ('duty_FR', 'float', '0..1'),
                                ('duty_RR', 'float', '0..1'),
                                ('duty_RL', 'float', '0..1'),
                                ('duty_FL', 'float', '0..1'))},
    'ctrl_output': {   'file': 'ctrl_output.csv',
                       'source': 'ControlOutput400 (0x4B)',
                       'nominal_rate_hz': 400,
                       'required': False,
                       'columns': (   ('timestamp_us', 'int', 'us'),
                                      ('seq', 'int', 'n/a'),
                                      ('thrust', 'float', 'N'),
                                      ('torque_roll', 'float', 'N*m'),
                                      ('torque_pitch', 'float', 'N*m'),
                                      ('torque_yaw', 'float', 'N*m'))},
    'pilot': {   'file': 'pilot.csv',
                 'source': 'Control (0x42)',
                 'nominal_rate_hz': 50,
                 'required': False,
                 'columns': (   ('timestamp_us', 'int', 'us'),
                                ('throttle', 'float', '0..1'),
                                ('roll', 'float', '-1..1'),
                                ('pitch', 'float', '-1..1'),
                                ('yaw', 'float', '-1..1'))},
    'ctrl_ref': {   'file': 'ctrl_ref.csv',
                    'source': 'CtrlRef (0x48)',
                    'nominal_rate_hz': 50,
                    'required': False,
                    'columns': (   ('timestamp_us', 'int', 'us'),
                                   ('flight_mode', 'int', 'enum'),
                                   ('angle_ref_roll', 'float', 'rad'),
                                   ('angle_ref_pitch', 'float', 'rad'),
                                   ('total_thrust', 'float', 'N'),
                                   ('duty_FR', 'float', '0..1'),
                                   ('duty_RR', 'float', '0..1'),
                                   ('duty_RL', 'float', '0..1'),
                                   ('duty_FL', 'float', '0..1'),
                                   ('alt_setpoint', 'float', 'm'),
                                   ('alt_vel_target', 'float', 'm/s'),
                                   ('climb_rate_cmd', 'float', 'm/s'),
                                   ('pos_setpoint_x', 'float', 'm'),
                                   ('pos_setpoint_y', 'float', 'm'))},
    'baro': {   'file': 'baro.csv',
                'source': 'Baro (0x45)',
                'nominal_rate_hz': 50,
                'required': False,
                'columns': (   ('timestamp_us', 'int', 'us'),
                               ('altitude', 'float', 'm'),
                               ('pressure', 'float', 'Pa'))},
    'tof_bottom': {   'file': 'tof_bottom.csv',
                      'source': 'ToF bottom (0x44)',
                      'nominal_rate_hz': 30,
                      'required': False,
                      'columns': (   ('timestamp_us', 'int', 'us'),
                                     ('distance', 'float', 'm'),
                                     ('status', 'int', 'enum'))},
    'tof_front': {   'file': 'tof_front.csv',
                     'source': 'ToF front (0x47)',
                     'nominal_rate_hz': 30,
                     'required': False,
                     'columns': (   ('timestamp_us', 'int', 'us'),
                                    ('distance', 'float', 'm'),
                                    ('status', 'int', 'enum'))},
    'flow': {   'file': 'flow.csv',
                'source': 'Flow (0x43)',
                'nominal_rate_hz': 100,
                'required': False,
                'columns': (   ('timestamp_us', 'int', 'us'),
                               ('dx', 'int', 'counts'),
                               ('dy', 'int', 'counts'),
                               ('quality', 'int', '0..255'))},
    'mag': {   'file': 'mag.csv',
               'source': 'Mag (0x46)',
               'nominal_rate_hz': 25,
               'required': False,
               'columns': (   ('timestamp_us', 'int', 'us'),
                              ('x', 'float', 'uT'),
                              ('y', 'float', 'uT'),
                              ('z', 'float', 'uT'))},
    'status': {   'file': 'status.csv',
                  'source': 'Status (0x4F)',
                  'nominal_rate_hz': 1,
                  'required': False,
                  'columns': (   ('timestamp_us', 'int', 'us'),
                                 ('uptime_ms', 'int', 'ms'),
                                 ('voltage', 'float', 'V'),
                                 ('current_ma', 'float', 'mA'),
                                 ('flight_state', 'int', 'enum'),
                                 ('sensor_health', 'int', 'bitmask'),
                                 ('eskf_status', 'int', 'bitmask'),
                                 ('reset_reason', 'int', 'enum'),
                                 ('pid_roll_kp', 'float', 'unitless (gain)'),
                                 ('pid_roll_ti', 'float', 's'),
                                 ('pid_roll_td', 'float', 's'),
                                 ('pid_pitch_kp', 'float', 'unitless (gain)'),
                                 ('pid_pitch_ti', 'float', 's'),
                                 ('pid_pitch_td', 'float', 's'),
                                 ('pid_yaw_kp', 'float', 'unitless (gain)'),
                                 ('pid_yaw_ti', 'float', 's'),
                                 ('pid_yaw_td', 'float', 's'))},
    'eskf_cov': {   'file': 'eskf_cov.csv',
                    'source': 'ESKF P-diag (0x49)',
                    'nominal_rate_hz': None,
                    'required': False,
                    'columns': (   ('timestamp_us', 'int', 'us'),
                                   ('p_pos_x', 'float', 'm^2'),
                                   ('p_pos_y', 'float', 'm^2'),
                                   ('p_pos_z', 'float', 'm^2'),
                                   ('p_vel_x', 'float', '(m/s)^2'),
                                   ('p_vel_y', 'float', '(m/s)^2'),
                                   ('p_vel_z', 'float', '(m/s)^2'),
                                   ('p_att_x', 'float', 'rad^2'),
                                   ('p_att_y', 'float', 'rad^2'),
                                   ('p_att_z', 'float', 'rad^2'),
                                   ('p_bg_x', 'float', '(rad/s)^2'),
                                   ('p_bg_y', 'float', '(rad/s)^2'),
                                   ('p_bg_z', 'float', '(rad/s)^2'),
                                   ('p_ba_x', 'float', '(m/s^2)^2'),
                                   ('p_ba_y', 'float', '(m/s^2)^2'),
                                   ('p_ba_z', 'float', '(m/s^2)^2'))},
    'truth': {   'file': 'truth.csv',
                 'source': 'sils/sim',
                 'nominal_rate_hz': None,
                 'required': False,
                 'columns': (   ('timestamp_us', 'int', 'us'),
                                ('pos_x', 'float', 'm'),
                                ('pos_y', 'float', 'm'),
                                ('pos_z', 'float', 'm'),
                                ('quat_w', 'float', 'unitless'),
                                ('quat_x', 'float', 'unitless'),
                                ('quat_y', 'float', 'unitless'),
                                ('quat_z', 'float', 'unitless'),
                                ('vel_x', 'float', 'm/s'),
                                ('vel_y', 'float', 'm/s'),
                                ('vel_z', 'float', 'm/s'),
                                ('rate_x', 'float', 'rad/s'),
                                ('rate_y', 'float', 'rad/s'),
                                ('rate_z', 'float', 'rad/s'))},
    'events': {   'file': 'events.csv',
                  'source': 'sils',
                  'nominal_rate_hz': None,
                  'required': False,
                  'columns': (   ('timestamp_us', 'int', 'us'),
                                 ('event', 'str', 'n/a'),
                                 ('value', 'str', 'n/a'))}}

# Stream name -> ordered list of column names (including timestamp_us).
# ストリーム名 -> 列名の順序付きリスト（timestamp_us を含む）。
COLUMN_NAMES = {
    name: [c[0] for c in info['columns']] for name, info in STREAMS.items()
}

# Streams that MUST be present in every v1 bundle (only imu.csv today --
# every other stream is optional because not every firmware/scenario
# sends every packet type).
# 全ての v1 バンドルに必須のストリーム（現状 imu.csv のみ -- 他は全て
# 任意。ファーム/シナリオによって送らないパケット種別があるため）。
REQUIRED_STREAMS = ['imu']

# Streams that publish one row per CONTROL CYCLE, all sharing the
# 'seq' column as their true per-observation key -- their timestamp_us
# can legitimately repeat (a control cycle that did not get a new IMU
# sample reuses its timestamp; see each stream's timestamp_us/seq
# description above). Derived here as "declares a 'seq' column",
# so this list never drifts out of sync with the YAML.
# 制御周期ごとに1行発行するストリーム。真の観測識別子は 'seq' 列で
# あり、timestamp_us は正当に重複し得る（新しい IMU 標本を得られ
# なかった周期は時刻を再利用する。各ストリームの timestamp_us/seq
# の説明を参照）。"'seq' 列を持つ" として導出するため、YAML との
# 乖離が起こらない。
LOCKSTEP_STREAMS = ['imu', 'attitude', 'posvel', 'rate_ref', 'motor', 'ctrl_output']

# Full per-column metadata (name/type/unit/description in both
# languages). Keyed by stream name; used by schema_for() to build the
# schema.json that travels inside a bundle.
# 列ごとの完全なメタデータ（名前・型・単位・日英の説明）。
# ストリーム名で引く。schema_for() がバンドル同梱の schema.json を
# 作るのに使う。
_COLUMN_META = {   'imu': [   {   'name': 'timestamp_us',
                   'type': 'int',
                   'unit': 'us',
                   'description_ja': '機体起動基準のマイクロ秒タイムスタンプ（SILS は仮想時計）。 制御周期が新しい IMU '
                                     '標本を得られなかった周期では、前の IMU '
                                     '標本と同じ値が連続する（実機で観測済み。行の一意な識別には `seq` を使うこと）。',
                   'description_en': 'Vehicle boot-relative microsecond timestamp '
                                     '(SILS: virtual clock). Repeats the previous '
                                     "cycle's value when a control cycle did not "
                                     'receive a new IMU sample (observed on real '
                                     'hardware -- use `seq` to uniquely identify a '
                                     'row).'},
               {   'name': 'seq',
                   'type': 'int',
                   'unit': 'n/a',
                   'description_ja': '制御周期の通し番号。実機取得では統合パケット 0x50 のヘッダ '
                                     'sequence（16bit、巻き戻りを取得側で展開）× 8 + パケット内 '
                                     'インデックス。SILS では制御周期カウンタ。旧 JSONL からの変換 では、imu '
                                     'ストリームの (timestamp_us, 同一時刻内の出現順) に 対応付けた imu '
                                     '側の行番号を使う。対応が取れない行は空欄。',
                   'description_en': 'Control-cycle sequential number. For a real '
                                     'vehicle capture: the unified packet (0x50) '
                                     'header sequence (16-bit, unwrapped by the '
                                     'capture tool) x 8 + the in-packet sub-index. For '
                                     'SILS: the control-cycle counter. When converted '
                                     "from legacy JSONL: the imu stream's row number, "
                                     'matched by (timestamp_us, occurrence order '
                                     'within that timestamp). Empty when no match is '
                                     'found.'},
               {   'name': 'gyro_x',
                   'type': 'float',
                   'unit': 'rad/s',
                   'description_ja': '推定器入力の角速度 X（機体座標系、フィルタ後）。',
                   'description_en': 'Estimator-input angular rate X (body frame, '
                                     'post-filter).'},
               {   'name': 'gyro_y',
                   'type': 'float',
                   'unit': 'rad/s',
                   'description_ja': '推定器入力の角速度 Y（機体座標系、フィルタ後）。',
                   'description_en': 'Estimator-input angular rate Y (body frame, '
                                     'post-filter).'},
               {   'name': 'gyro_z',
                   'type': 'float',
                   'unit': 'rad/s',
                   'description_ja': '推定器入力の角速度 Z（機体座標系、フィルタ後）。',
                   'description_en': 'Estimator-input angular rate Z (body frame, '
                                     'post-filter).'},
               {   'name': 'accel_x',
                   'type': 'float',
                   'unit': 'm/s^2',
                   'description_ja': '推定器入力の加速度 X（機体座標系、フィルタ後）。',
                   'description_en': 'Estimator-input acceleration X (body frame, '
                                     'post-filter).'},
               {   'name': 'accel_y',
                   'type': 'float',
                   'unit': 'm/s^2',
                   'description_ja': '推定器入力の加速度 Y（機体座標系、フィルタ後）。',
                   'description_en': 'Estimator-input acceleration Y (body frame, '
                                     'post-filter).'},
               {   'name': 'accel_z',
                   'type': 'float',
                   'unit': 'm/s^2',
                   'description_ja': '推定器入力の加速度 Z（機体座標系、フィルタ後）。',
                   'description_en': 'Estimator-input acceleration Z (body frame, '
                                     'post-filter).'},
               {   'name': 'gyro_raw_x',
                   'type': 'float',
                   'unit': 'rad/s',
                   'description_ja': 'フィルタ前の生角速度 X。vehicle ファームは IMU 側 LPF を持たない ため '
                                     'raw == filtered（電文互換のため両方送っている）。',
                   'description_en': 'Pre-filter raw angular rate X. firmware/vehicle '
                                     'has no IMU-side LPF, so raw == filtered here '
                                     '(sent for wire compatibility with firmware that '
                                     'does filter).'},
               {   'name': 'gyro_raw_y',
                   'type': 'float',
                   'unit': 'rad/s',
                   'description_ja': 'フィルタ前の生角速度 Y（vehicle では gyro_y と同値）。',
                   'description_en': 'Pre-filter raw angular rate Y (equals gyro_y on '
                                     'vehicle).'},
               {   'name': 'gyro_raw_z',
                   'type': 'float',
                   'unit': 'rad/s',
                   'description_ja': 'フィルタ前の生角速度 Z（vehicle では gyro_z と同値）。',
                   'description_en': 'Pre-filter raw angular rate Z (equals gyro_z on '
                                     'vehicle).'},
               {   'name': 'accel_raw_x',
                   'type': 'float',
                   'unit': 'm/s^2',
                   'description_ja': 'フィルタ前の生加速度 X（vehicle では accel_x と同値）。',
                   'description_en': 'Pre-filter raw acceleration X (equals accel_x on '
                                     'vehicle).'},
               {   'name': 'accel_raw_y',
                   'type': 'float',
                   'unit': 'm/s^2',
                   'description_ja': 'フィルタ前の生加速度 Y（vehicle では accel_y と同値）。',
                   'description_en': 'Pre-filter raw acceleration Y (equals accel_y on '
                                     'vehicle).'},
               {   'name': 'accel_raw_z',
                   'type': 'float',
                   'unit': 'm/s^2',
                   'description_ja': 'フィルタ前の生加速度 Z（vehicle では accel_z と同値）。',
                   'description_en': 'Pre-filter raw acceleration Z (equals accel_z on '
                                     'vehicle).'}],
    'attitude': [   {   'name': 'timestamp_us',
                        'type': 'int',
                        'unit': 'us',
                        'description_ja': '機体起動基準のマイクロ秒タイムスタンプ。imu.csv と同一パケット '
                                          '由来なので、同じ行番号の imu.csv と同じ値になる。制御周期が 新しい IMU '
                                          '標本を得られなかった周期では、前の IMU 標本と同じ '
                                          '値が連続する（実機で観測済み。行の一意な識別には `seq` を 使うこと）。',
                        'description_en': 'Vehicle boot-relative microsecond '
                                          'timestamp. Comes from the same packet as '
                                          'imu.csv, so the value matches imu.csv at '
                                          'the same row index. Repeats the previous '
                                          "cycle's value when a control cycle did not "
                                          'receive a new IMU sample (observed on real '
                                          'hardware -- use `seq` to uniquely identify '
                                          'a row).'},
                    {   'name': 'seq',
                        'type': 'int',
                        'unit': 'n/a',
                        'description_ja': '制御周期の通し番号。実機取得では統合パケット 0x50 のヘッダ '
                                          'sequence（16bit、巻き戻りを取得側で展開）× 8 + パケット内 '
                                          'インデックス。SILS では制御周期カウンタ。旧 JSONL からの変換 では、imu '
                                          'ストリームの (timestamp_us, 同一時刻内の出現順) に 対応付けた '
                                          'imu 側の行番号を使う。対応が取れない行は空欄。',
                        'description_en': 'Control-cycle sequential number. For a real '
                                          'vehicle capture: the unified packet (0x50) '
                                          'header sequence (16-bit, unwrapped by the '
                                          'capture tool) x 8 + the in-packet '
                                          'sub-index. For SILS: the control-cycle '
                                          'counter. When converted from legacy JSONL: '
                                          "the imu stream's row number, matched by "
                                          '(timestamp_us, occurrence order within that '
                                          'timestamp). Empty when no match is found.'},
                    {   'name': 'quat_w',
                        'type': 'float',
                        'unit': 'unitless',
                        'description_ja': '姿勢推定クォータニオンの実部 w（機体座標系）。',
                        'description_en': 'Attitude estimate quaternion real part w '
                                          '(body frame).'},
                    {   'name': 'quat_x',
                        'type': 'float',
                        'unit': 'unitless',
                        'description_ja': '姿勢推定クォータニオンの虚部 x。',
                        'description_en': 'Attitude estimate quaternion imaginary part '
                                          'x.'},
                    {   'name': 'quat_y',
                        'type': 'float',
                        'unit': 'unitless',
                        'description_ja': '姿勢推定クォータニオンの虚部 y。',
                        'description_en': 'Attitude estimate quaternion imaginary part '
                                          'y.'},
                    {   'name': 'quat_z',
                        'type': 'float',
                        'unit': 'unitless',
                        'description_ja': '姿勢推定クォータニオンの虚部 z。',
                        'description_en': 'Attitude estimate quaternion imaginary part '
                                          'z.'},
                    {   'name': 'gyro_bias_x',
                        'type': 'float',
                        'unit': 'rad/s',
                        'description_ja': 'ESKF が推定したジャイロバイアス X。',
                        'description_en': 'ESKF-estimated gyro bias X.'},
                    {   'name': 'gyro_bias_y',
                        'type': 'float',
                        'unit': 'rad/s',
                        'description_ja': 'ESKF が推定したジャイロバイアス Y。',
                        'description_en': 'ESKF-estimated gyro bias Y.'},
                    {   'name': 'gyro_bias_z',
                        'type': 'float',
                        'unit': 'rad/s',
                        'description_ja': 'ESKF が推定したジャイロバイアス Z。',
                        'description_en': 'ESKF-estimated gyro bias Z.'},
                    {   'name': 'accel_bias_x',
                        'type': 'float',
                        'unit': 'm/s^2',
                        'description_ja': 'ESKF が推定した加速度バイアス X。',
                        'description_en': 'ESKF-estimated accelerometer bias X.'},
                    {   'name': 'accel_bias_y',
                        'type': 'float',
                        'unit': 'm/s^2',
                        'description_ja': 'ESKF が推定した加速度バイアス Y。',
                        'description_en': 'ESKF-estimated accelerometer bias Y.'},
                    {   'name': 'accel_bias_z',
                        'type': 'float',
                        'unit': 'm/s^2',
                        'description_ja': 'ESKF が推定した加速度バイアス Z。',
                        'description_en': 'ESKF-estimated accelerometer bias Z.'}],
    'posvel': [   {   'name': 'timestamp_us',
                      'type': 'int',
                      'unit': 'us',
                      'description_ja': '機体起動基準のマイクロ秒タイムスタンプ。制御周期が新しい IMU '
                                        '標本を得られなかった周期では、前の IMU 標本と同じ値が連続する '
                                        '（実機で観測済み。行の一意な識別には `seq` を使うこと）。',
                      'description_en': 'Vehicle boot-relative microsecond timestamp. '
                                        "Repeats the previous cycle's value when a "
                                        'control cycle did not receive a new IMU '
                                        'sample (observed on real hardware -- use '
                                        '`seq` to uniquely identify a row).'},
                  {   'name': 'seq',
                      'type': 'int',
                      'unit': 'n/a',
                      'description_ja': '制御周期の通し番号。実機取得では統合パケット 0x50 のヘッダ '
                                        'sequence（16bit、巻き戻りを取得側で展開）× 8 + パケット内 '
                                        'インデックス。SILS では制御周期カウンタ。旧 JSONL からの変換 では、imu '
                                        'ストリームの (timestamp_us, 同一時刻内の出現順) に 対応付けた imu '
                                        '側の行番号を使う。対応が取れない行は空欄。',
                      'description_en': 'Control-cycle sequential number. For a real '
                                        'vehicle capture: the unified packet (0x50) '
                                        'header sequence (16-bit, unwrapped by the '
                                        'capture tool) x 8 + the in-packet sub-index. '
                                        'For SILS: the control-cycle counter. When '
                                        "converted from legacy JSONL: the imu stream's "
                                        'row number, matched by (timestamp_us, '
                                        'occurrence order within that timestamp). '
                                        'Empty when no match is found.'},
                  {   'name': 'pos_x',
                      'type': 'float',
                      'unit': 'm',
                      'description_ja': 'ESKF が推定した位置 X（NED 座標系）。',
                      'description_en': 'ESKF-estimated position X (NED frame).'},
                  {   'name': 'pos_y',
                      'type': 'float',
                      'unit': 'm',
                      'description_ja': 'ESKF が推定した位置 Y（NED 座標系）。',
                      'description_en': 'ESKF-estimated position Y (NED frame).'},
                  {   'name': 'pos_z',
                      'type': 'float',
                      'unit': 'm',
                      'description_ja': 'ESKF が推定した位置 Z（NED 座標系、下向き正）。',
                      'description_en': 'ESKF-estimated position Z (NED frame, '
                                        'positive down).'},
                  {   'name': 'vel_x',
                      'type': 'float',
                      'unit': 'm/s',
                      'description_ja': 'ESKF が推定した速度 X（NED 座標系）。',
                      'description_en': 'ESKF-estimated velocity X (NED frame).'},
                  {   'name': 'vel_y',
                      'type': 'float',
                      'unit': 'm/s',
                      'description_ja': 'ESKF が推定した速度 Y（NED 座標系）。',
                      'description_en': 'ESKF-estimated velocity Y (NED frame).'},
                  {   'name': 'vel_z',
                      'type': 'float',
                      'unit': 'm/s',
                      'description_ja': 'ESKF が推定した速度 Z（NED 座標系、下向き正）。',
                      'description_en': 'ESKF-estimated velocity Z (NED frame, '
                                        'positive down).'}],
    'rate_ref': [   {   'name': 'timestamp_us',
                        'type': 'int',
                        'unit': 'us',
                        'description_ja': '機体起動基準のマイクロ秒タイムスタンプ。ControlTask は IMU '
                                          '標本の時刻をそのまま使うため、制御周期が新しい IMU 標本を '
                                          '得られなかった周期ではこの値が前の周期と同じになる -- しかし rate_ref '
                                          '自体は制御則がその周期に計算した新しい値であり、 timestamp_us '
                                          'が同じでも値は異なり得る（実機で観測済み）。 行の対応付けには '
                                          'timestamp_us ではなく `seq` を使うこと。',
                        'description_en': 'Vehicle boot-relative microsecond '
                                          'timestamp. ControlTask reuses the IMU '
                                          "sample's own timestamp, so this repeats the "
                                          "previous cycle's value when no new IMU "
                                          'sample arrived that cycle -- however '
                                          'rate_ref itself is freshly computed by the '
                                          'control law every cycle, so its value can '
                                          'differ even when timestamp_us is identical '
                                          '(observed on real hardware). Use `seq`, not '
                                          'timestamp_us, to pair rows across streams.'},
                    {   'name': 'seq',
                        'type': 'int',
                        'unit': 'n/a',
                        'description_ja': '制御周期の通し番号。実機取得では統合パケット 0x50 のヘッダ '
                                          'sequence（16bit、巻き戻りを取得側で展開）× 8 + パケット内 '
                                          'インデックス。SILS では制御周期カウンタ。旧 JSONL からの変換 では、imu '
                                          'ストリームの (timestamp_us, 同一時刻内の出現順) に 対応付けた '
                                          'imu 側の行番号を使う。対応が取れない行は空欄。',
                        'description_en': 'Control-cycle sequential number. For a real '
                                          'vehicle capture: the unified packet (0x50) '
                                          'header sequence (16-bit, unwrapped by the '
                                          'capture tool) x 8 + the in-packet '
                                          'sub-index. For SILS: the control-cycle '
                                          'counter. When converted from legacy JSONL: '
                                          "the imu stream's row number, matched by "
                                          '(timestamp_us, occurrence order within that '
                                          'timestamp). Empty when no match is found.'},
                    {   'name': 'rate_ref_roll',
                        'type': 'float',
                        'unit': 'rad/s',
                        'description_ja': '内側ループ（レート制御）のロール角速度目標。',
                        'description_en': 'Inner-loop (rate control) roll angular-rate '
                                          'reference.'},
                    {   'name': 'rate_ref_pitch',
                        'type': 'float',
                        'unit': 'rad/s',
                        'description_ja': '内側ループ（レート制御）のピッチ角速度目標。',
                        'description_en': 'Inner-loop (rate control) pitch '
                                          'angular-rate reference.'},
                    {   'name': 'rate_ref_yaw',
                        'type': 'float',
                        'unit': 'rad/s',
                        'description_ja': '内側ループ（レート制御）のヨー角速度目標。',
                        'description_en': 'Inner-loop (rate control) yaw angular-rate '
                                          'reference.'}],
    'motor': [   {   'name': 'timestamp_us',
                     'type': 'int',
                     'unit': 'us',
                     'description_ja': '機体起動基準のマイクロ秒タイムスタンプ。ControlTask は IMU '
                                       '標本の時刻をそのまま使うため、制御周期が新しい IMU 標本を '
                                       '得られなかった周期ではこの値が前の周期と同じになる -- しかし duty '
                                       '自体はミキサーがその周期に計算した新しい値であり、 timestamp_us '
                                       'が同じでも値は異なり得る（実機で観測済み、 rate_ref と同じ理由）。行の対応付けには '
                                       'timestamp_us ではなく `seq` を使うこと。',
                     'description_en': 'Vehicle boot-relative microsecond timestamp. '
                                       "Repeats the previous cycle's value when no new "
                                       'IMU sample arrived that cycle -- but the duty '
                                       'values themselves are freshly computed by the '
                                       'mixer every cycle, so they can differ even '
                                       'when timestamp_us is identical (observed on '
                                       'real hardware, same reason as rate_ref). Use '
                                       '`seq`, not timestamp_us, to pair rows across '
                                       'streams.'},
                 {   'name': 'seq',
                     'type': 'int',
                     'unit': 'n/a',
                     'description_ja': '制御周期の通し番号。実機取得では統合パケット 0x50 のヘッダ '
                                       'sequence（16bit、巻き戻りを取得側で展開）× 8 + パケット内 '
                                       'インデックス。SILS では制御周期カウンタ。旧 JSONL からの変換 では、imu '
                                       'ストリームの (timestamp_us, 同一時刻内の出現順) に 対応付けた imu '
                                       '側の行番号を使う。対応が取れない行は空欄。',
                     'description_en': 'Control-cycle sequential number. For a real '
                                       'vehicle capture: the unified packet (0x50) '
                                       'header sequence (16-bit, unwrapped by the '
                                       'capture tool) x 8 + the in-packet sub-index. '
                                       'For SILS: the control-cycle counter. When '
                                       "converted from legacy JSONL: the imu stream's "
                                       'row number, matched by (timestamp_us, '
                                       'occurrence order within that timestamp). Empty '
                                       'when no match is found.'},
                 {   'name': 'duty_FR',
                     'type': 'float',
                     'unit': '0..1',
                     'description_ja': '前右モータの実際の duty 比。`sf sysid fit` のプラント入力 '
                                       '（レートループ同定）。',
                     'description_en': 'Actual commanded duty ratio, front-right '
                                       'motor. The real plant input `sf sysid fit` '
                                       'uses for rate-loop identification.'},
                 {   'name': 'duty_RR',
                     'type': 'float',
                     'unit': '0..1',
                     'description_ja': '後右モータの実際の duty 比。',
                     'description_en': 'Actual commanded duty ratio, rear-right '
                                       'motor.'},
                 {   'name': 'duty_RL',
                     'type': 'float',
                     'unit': '0..1',
                     'description_ja': '後左モータの実際の duty 比。',
                     'description_en': 'Actual commanded duty ratio, rear-left motor.'},
                 {   'name': 'duty_FL',
                     'type': 'float',
                     'unit': '0..1',
                     'description_ja': '前左モータの実際の duty 比。',
                     'description_en': 'Actual commanded duty ratio, front-left '
                                       'motor.'}],
    'ctrl_output': [   {   'name': 'timestamp_us',
                           'type': 'int',
                           'unit': 'us',
                           'description_ja': '機体起動基準のマイクロ秒タイムスタンプ。ControlTask は IMU '
                                             '標本の時刻をそのまま使うため、制御周期が新しい IMU 標本を '
                                             '得られなかった周期ではこの値が前の周期と同じになる -- しかし '
                                             '指令推力・トルク自体は制御則がその周期に計算した新しい値であり、 '
                                             'timestamp_us が同じでも値は異なり得る（実機で観測済み、 '
                                             'rate_ref と同じ理由）。行の対応付けには timestamp_us '
                                             'ではなく `seq` を使うこと。',
                           'description_en': 'Vehicle boot-relative microsecond '
                                             "timestamp. Repeats the previous cycle's "
                                             'value when no new IMU sample arrived '
                                             'that cycle -- but the commanded '
                                             'thrust/torque themselves are freshly '
                                             'computed by the control law every cycle, '
                                             'so they can differ even when '
                                             'timestamp_us is identical (observed on '
                                             'real hardware, same reason as rate_ref). '
                                             'Use `seq`, not timestamp_us, to pair '
                                             'rows across streams.'},
                       {   'name': 'seq',
                           'type': 'int',
                           'unit': 'n/a',
                           'description_ja': '制御周期の通し番号。実機取得では統合パケット 0x50 のヘッダ '
                                             'sequence（16bit、巻き戻りを取得側で展開）× 8 + パケット内 '
                                             'インデックス。SILS では制御周期カウンタ。旧 JSONL からの変換 '
                                             'では、imu ストリームの (timestamp_us, 同一時刻内の出現順) '
                                             'に 対応付けた imu 側の行番号を使う。対応が取れない行は空欄。',
                           'description_en': 'Control-cycle sequential number. For a '
                                             'real vehicle capture: the unified packet '
                                             '(0x50) header sequence (16-bit, '
                                             'unwrapped by the capture tool) x 8 + the '
                                             'in-packet sub-index. For SILS: the '
                                             'control-cycle counter. When converted '
                                             "from legacy JSONL: the imu stream's row "
                                             'number, matched by (timestamp_us, '
                                             'occurrence order within that timestamp). '
                                             'Empty when no match is found.'},
                       {   'name': 'thrust',
                           'type': 'float',
                           'unit': 'N',
                           'description_ja': 'ミキサー手前のコントローラ指令推力（合計）。どのミキサーで '
                                             '飛んだかに依存しないプラント入力。',
                           'description_en': 'PRE-MIXER commanded total thrust. A '
                                             'mixer-agnostic plant input (does not '
                                             'depend on which mixer flew).'},
                       {   'name': 'torque_roll',
                           'type': 'float',
                           'unit': 'N*m',
                           'description_ja': 'ミキサー手前のコントローラ指令ロールトルク。',
                           'description_en': 'PRE-MIXER commanded roll body torque.'},
                       {   'name': 'torque_pitch',
                           'type': 'float',
                           'unit': 'N*m',
                           'description_ja': 'ミキサー手前のコントローラ指令ピッチトルク。',
                           'description_en': 'PRE-MIXER commanded pitch body torque.'},
                       {   'name': 'torque_yaw',
                           'type': 'float',
                           'unit': 'N*m',
                           'description_ja': 'ミキサー手前のコントローラ指令ヨートルク。',
                           'description_en': 'PRE-MIXER commanded yaw body torque.'}],
    'pilot': [   {   'name': 'timestamp_us',
                     'type': 'int',
                     'unit': 'us',
                     'description_ja': '機体起動基準のマイクロ秒タイムスタンプ。',
                     'description_en': 'Vehicle boot-relative microsecond timestamp.'},
                 {   'name': 'throttle',
                     'type': 'float',
                     'unit': '0..1',
                     'description_ja': '操縦スティックのスロットル入力。',
                     'description_en': 'Pilot stick throttle input.'},
                 {   'name': 'roll',
                     'type': 'float',
                     'unit': '-1..1',
                     'description_ja': '操縦スティックのロール入力。',
                     'description_en': 'Pilot stick roll input.'},
                 {   'name': 'pitch',
                     'type': 'float',
                     'unit': '-1..1',
                     'description_ja': '操縦スティックのピッチ入力。',
                     'description_en': 'Pilot stick pitch input.'},
                 {   'name': 'yaw',
                     'type': 'float',
                     'unit': '-1..1',
                     'description_ja': '操縦スティックのヨー入力。',
                     'description_en': 'Pilot stick yaw input.'}],
    'ctrl_ref': [   {   'name': 'timestamp_us',
                        'type': 'int',
                        'unit': 'us',
                        'description_ja': '機体起動基準のマイクロ秒タイムスタンプ。',
                        'description_en': 'Vehicle boot-relative microsecond '
                                          'timestamp.'},
                    {   'name': 'flight_mode',
                        'type': 'int',
                        'unit': 'enum',
                        'description_ja': '飛行モード。0=ACRO, 1=STABILIZE, 2=ALT_HOLD, '
                                          '3=POS_HOLD。',
                        'description_en': 'Flight mode. 0=ACRO, 1=STABILIZE, '
                                          '2=ALT_HOLD, 3=POS_HOLD.'},
                    {   'name': 'angle_ref_roll',
                        'type': 'float',
                        'unit': 'rad',
                        'description_ja': '外側ループ（姿勢制御）のロール角目標。',
                        'description_en': 'Outer-loop (attitude control) roll angle '
                                          'reference.'},
                    {   'name': 'angle_ref_pitch',
                        'type': 'float',
                        'unit': 'rad',
                        'description_ja': '外側ループ（姿勢制御）のピッチ角目標。',
                        'description_en': 'Outer-loop (attitude control) pitch angle '
                                          'reference.'},
                    {   'name': 'total_thrust',
                        'type': 'float',
                        'unit': 'N',
                        'description_ja': 'コントローラが計算した合計推力指令（50Hz）。',
                        'description_en': 'Controller-computed total thrust command '
                                          '(50 Hz).'},
                    {   'name': 'duty_FR',
                        'type': 'float',
                        'unit': '0..1',
                        'description_ja': '前右モータの duty 比（50Hz、CtrlRef パケット由来）。400Hz の '
                                          '実測値は motor.csv を使うこと。',
                        'description_en': 'Front-right motor duty ratio (50 Hz, from '
                                          'the CtrlRef packet). Prefer motor.csv for '
                                          'the 400 Hz measured value.'},
                    {   'name': 'duty_RR',
                        'type': 'float',
                        'unit': '0..1',
                        'description_ja': '後右モータの duty 比（50Hz、CtrlRef パケット由来）。',
                        'description_en': 'Rear-right motor duty ratio (50 Hz, from '
                                          'the CtrlRef packet).'},
                    {   'name': 'duty_RL',
                        'type': 'float',
                        'unit': '0..1',
                        'description_ja': '後左モータの duty 比（50Hz、CtrlRef パケット由来）。',
                        'description_en': 'Rear-left motor duty ratio (50 Hz, from the '
                                          'CtrlRef packet).'},
                    {   'name': 'duty_FL',
                        'type': 'float',
                        'unit': '0..1',
                        'description_ja': '前左モータの duty 比（50Hz、CtrlRef パケット由来）。',
                        'description_en': 'Front-left motor duty ratio (50 Hz, from '
                                          'the CtrlRef packet).'},
                    {   'name': 'alt_setpoint',
                        'type': 'float',
                        'unit': 'm',
                        'description_ja': '高度保持の目標高度（ALT_HOLD/POS_HOLD）。',
                        'description_en': 'Altitude-hold target altitude '
                                          '(ALT_HOLD/POS_HOLD).'},
                    {   'name': 'alt_vel_target',
                        'type': 'float',
                        'unit': 'm/s',
                        'description_ja': '高度制御の目標上昇率。',
                        'description_en': 'Altitude-loop target climb rate.'},
                    {   'name': 'climb_rate_cmd',
                        'type': 'float',
                        'unit': 'm/s',
                        'description_ja': '操縦入力から生成した上昇率指令。',
                        'description_en': 'Climb-rate command derived from pilot '
                                          'input.'},
                    {   'name': 'pos_setpoint_x',
                        'type': 'float',
                        'unit': 'm',
                        'description_ja': '水平位置保持の目標位置 X（POS_HOLD、NED 座標系）。',
                        'description_en': 'Position-hold target X (POS_HOLD, NED '
                                          'frame).'},
                    {   'name': 'pos_setpoint_y',
                        'type': 'float',
                        'unit': 'm',
                        'description_ja': '水平位置保持の目標位置 Y（POS_HOLD、NED 座標系）。',
                        'description_en': 'Position-hold target Y (POS_HOLD, NED '
                                          'frame).'}],
    'baro': [   {   'name': 'timestamp_us',
                    'type': 'int',
                    'unit': 'us',
                    'description_ja': '機体起動基準のマイクロ秒タイムスタンプ。',
                    'description_en': 'Vehicle boot-relative microsecond timestamp.'},
                {   'name': 'altitude',
                    'type': 'float',
                    'unit': 'm',
                    'description_ja': '気圧高度（相対値、基準は起動時気圧）。',
                    'description_en': 'Barometric altitude (relative to the power-on '
                                      'reference pressure).'},
                {   'name': 'pressure',
                    'type': 'float',
                    'unit': 'Pa',
                    'description_ja': '気圧。本書の SI 規約どおり Pa で統一する。電文上は hPa で '
                                      '送られる（data_stream_wire.hpp の WireBaro 構造体コメント '
                                      '「電文上は hPa、ファーム内部は Pa」参照）ため、変換時に ×100 して記録する。',
                    'description_en': "Pressure, unified to Pa per this document's SI "
                                      'convention. The wire sends hPa (see the '
                                      'WireBaro struct comment in '
                                      'data_stream_wire.hpp: "hPa on the wire, Pa '
                                      'inside the firmware"), so the conversion '
                                      'multiplies by 100 before recording.'}],
    'tof_bottom': [   {   'name': 'timestamp_us',
                          'type': 'int',
                          'unit': 'us',
                          'description_ja': '機体起動基準のマイクロ秒タイムスタンプ。',
                          'description_en': 'Vehicle boot-relative microsecond '
                                            'timestamp.'},
                      {   'name': 'distance',
                          'type': 'float',
                          'unit': 'm',
                          'description_ja': '下向き ToF センサが測定した距離。',
                          'description_en': 'Distance measured by the downward-facing '
                                            'ToF sensor.'},
                      {   'name': 'status',
                          'type': 'int',
                          'unit': 'enum',
                          'description_ja': 'センサ状態。0 = 有効値。',
                          'description_en': 'Sensor status. 0 = valid reading.'}],
    'tof_front': [   {   'name': 'timestamp_us',
                         'type': 'int',
                         'unit': 'us',
                         'description_ja': '機体起動基準のマイクロ秒タイムスタンプ。',
                         'description_en': 'Vehicle boot-relative microsecond '
                                           'timestamp.'},
                     {   'name': 'distance',
                         'type': 'float',
                         'unit': 'm',
                         'description_ja': '前向き ToF センサが測定した距離。',
                         'description_en': 'Distance measured by the forward-facing '
                                           'ToF sensor.'},
                     {   'name': 'status',
                         'type': 'int',
                         'unit': 'enum',
                         'description_ja': 'センサ状態。0 = 有効値。',
                         'description_en': 'Sensor status. 0 = valid reading.'}],
    'flow': [   {   'name': 'timestamp_us',
                    'type': 'int',
                    'unit': 'us',
                    'description_ja': '機体起動基準のマイクロ秒タイムスタンプ。',
                    'description_en': 'Vehicle boot-relative microsecond timestamp.'},
                {   'name': 'dx',
                    'type': 'int',
                    'unit': 'counts',
                    'description_ja': 'オプティカルフローの X 方向積算カウント。',
                    'description_en': 'Optical flow accumulated count, X axis.'},
                {   'name': 'dy',
                    'type': 'int',
                    'unit': 'counts',
                    'description_ja': 'オプティカルフローの Y 方向積算カウント。',
                    'description_en': 'Optical flow accumulated count, Y axis.'},
                {   'name': 'quality',
                    'type': 'int',
                    'unit': '0..255',
                    'description_ja': 'フローセンサの信頼度指標。',
                    'description_en': 'Flow sensor quality/confidence metric.'}],
    'mag': [   {   'name': 'timestamp_us',
                   'type': 'int',
                   'unit': 'us',
                   'description_ja': '機体起動基準のマイクロ秒タイムスタンプ。',
                   'description_en': 'Vehicle boot-relative microsecond timestamp.'},
               {   'name': 'x',
                   'type': 'float',
                   'unit': 'uT',
                   'description_ja': '地磁気 X（機体座標系）。',
                   'description_en': 'Magnetic field X (body frame).'},
               {   'name': 'y',
                   'type': 'float',
                   'unit': 'uT',
                   'description_ja': '地磁気 Y（機体座標系）。',
                   'description_en': 'Magnetic field Y (body frame).'},
               {   'name': 'z',
                   'type': 'float',
                   'unit': 'uT',
                   'description_ja': '地磁気 Z（機体座標系）。',
                   'description_en': 'Magnetic field Z (body frame).'}],
    'status': [   {   'name': 'timestamp_us',
                      'type': 'int',
                      'unit': 'us',
                      'description_ja': '機体起動基準のマイクロ秒タイムスタンプ。Status パケットは uptime_ms '
                                        'しか持たないため timestamp_us = uptime_ms * 1000。',
                      'description_en': 'Vehicle boot-relative microsecond timestamp. '
                                        'The Status packet only carries uptime_ms, so '
                                        'timestamp_us = uptime_ms * 1000.'},
                  {   'name': 'uptime_ms',
                      'type': 'int',
                      'unit': 'ms',
                      'description_ja': '起動からの経過時間。',
                      'description_en': 'Elapsed time since boot.'},
                  {   'name': 'voltage',
                      'type': 'float',
                      'unit': 'V',
                      'description_ja': 'バッテリ電圧。',
                      'description_en': 'Battery voltage.'},
                  {   'name': 'current_ma',
                      'type': 'float',
                      'unit': 'mA',
                      'description_ja': 'バッテリ電流。StatusPacket v3 (57B) 以降のみ送信。旧ファーム '
                                        '（v1/v2）を読んだ場合はこの列が空欄になる。',
                      'description_en': 'Battery current. Sent only by StatusPacket v3 '
                                        '(57B) and later; empty when the source '
                                        'firmware only sent v1/v2.'},
                  {   'name': 'flight_state',
                      'type': 'int',
                      'unit': 'enum',
                      'description_ja': '飛行状態（FlightState、起動シーケンス〜飛行〜着陸）。',
                      'description_en': 'Flight state (FlightState enum: boot '
                                        'sequence, flight, landing, ...).'},
                  {   'name': 'sensor_health',
                      'type': 'int',
                      'unit': 'bitmask',
                      'description_ja': 'センサ健全性のビットマスク。',
                      'description_en': 'Sensor health bitmask.'},
                  {   'name': 'eskf_status',
                      'type': 'int',
                      'unit': 'bitmask',
                      'description_ja': 'ESKF 状態のビットマスク。bit0 = 推定器初期化済み。',
                      'description_en': 'ESKF status bitmask. bit0 = estimator '
                                        'initialized.'},
                  {   'name': 'reset_reason',
                      'type': 'int',
                      'unit': 'enum',
                      'description_ja': '起動時リセット理由（esp_reset_reason()）。1=POWERON, '
                                        '3=SW, 4=PANIC, 5=INT_WDT, 6=TASK_WDT, '
                                        '9=BROWNOUT 等。',
                      'description_en': 'Boot-time reset reason from '
                                        'esp_reset_reason(). E.g. 1=POWERON, 3=SW, '
                                        '4=PANIC, 5=INT_WDT, 6=TASK_WDT, 9=BROWNOUT.'},
                  {   'name': 'pid_roll_kp',
                      'type': 'float',
                      'unit': 'unitless (gain)',
                      'description_ja': 'ロールレート PID の比例ゲイン（時刻付きで記録することで飛行中の '
                                        '自動チューニングによる変更を追える）。',
                      'description_en': 'Roll-rate PID proportional gain (timestamped '
                                        'so an in-flight autotune/gain change over the '
                                        'flight can be traced).'},
                  {   'name': 'pid_roll_ti',
                      'type': 'float',
                      'unit': 's',
                      'description_ja': 'ロールレート PID の積分時間。',
                      'description_en': 'Roll-rate PID integral time.'},
                  {   'name': 'pid_roll_td',
                      'type': 'float',
                      'unit': 's',
                      'description_ja': 'ロールレート PID の微分時間。',
                      'description_en': 'Roll-rate PID derivative time.'},
                  {   'name': 'pid_pitch_kp',
                      'type': 'float',
                      'unit': 'unitless (gain)',
                      'description_ja': 'ピッチレート PID の比例ゲイン。',
                      'description_en': 'Pitch-rate PID proportional gain.'},
                  {   'name': 'pid_pitch_ti',
                      'type': 'float',
                      'unit': 's',
                      'description_ja': 'ピッチレート PID の積分時間。',
                      'description_en': 'Pitch-rate PID integral time.'},
                  {   'name': 'pid_pitch_td',
                      'type': 'float',
                      'unit': 's',
                      'description_ja': 'ピッチレート PID の微分時間。',
                      'description_en': 'Pitch-rate PID derivative time.'},
                  {   'name': 'pid_yaw_kp',
                      'type': 'float',
                      'unit': 'unitless (gain)',
                      'description_ja': 'ヨーレート PID の比例ゲイン。',
                      'description_en': 'Yaw-rate PID proportional gain.'},
                  {   'name': 'pid_yaw_ti',
                      'type': 'float',
                      'unit': 's',
                      'description_ja': 'ヨーレート PID の積分時間。',
                      'description_en': 'Yaw-rate PID integral time.'},
                  {   'name': 'pid_yaw_td',
                      'type': 'float',
                      'unit': 's',
                      'description_ja': 'ヨーレート PID の微分時間。',
                      'description_en': 'Yaw-rate PID derivative time.'}],
    'eskf_cov': [   {   'name': 'timestamp_us',
                        'type': 'int',
                        'unit': 'us',
                        'description_ja': '機体起動基準のマイクロ秒タイムスタンプ。',
                        'description_en': 'Vehicle boot-relative microsecond '
                                          'timestamp.'},
                    {   'name': 'p_pos_x',
                        'type': 'float',
                        'unit': 'm^2',
                        'description_ja': 'ESKF 共分散行列 P の対角成分、位置 X の分散。',
                        'description_en': 'ESKF covariance matrix P diagonal entry, '
                                          'position-X variance.'},
                    {   'name': 'p_pos_y',
                        'type': 'float',
                        'unit': 'm^2',
                        'description_ja': 'ESKF 共分散行列 P の対角成分、位置 Y の分散。',
                        'description_en': 'ESKF covariance matrix P diagonal entry, '
                                          'position-Y variance.'},
                    {   'name': 'p_pos_z',
                        'type': 'float',
                        'unit': 'm^2',
                        'description_ja': 'ESKF 共分散行列 P の対角成分、位置 Z の分散。',
                        'description_en': 'ESKF covariance matrix P diagonal entry, '
                                          'position-Z variance.'},
                    {   'name': 'p_vel_x',
                        'type': 'float',
                        'unit': '(m/s)^2',
                        'description_ja': 'ESKF 共分散行列 P の対角成分、速度 X の分散。',
                        'description_en': 'ESKF covariance matrix P diagonal entry, '
                                          'velocity-X variance.'},
                    {   'name': 'p_vel_y',
                        'type': 'float',
                        'unit': '(m/s)^2',
                        'description_ja': 'ESKF 共分散行列 P の対角成分、速度 Y の分散。',
                        'description_en': 'ESKF covariance matrix P diagonal entry, '
                                          'velocity-Y variance.'},
                    {   'name': 'p_vel_z',
                        'type': 'float',
                        'unit': '(m/s)^2',
                        'description_ja': 'ESKF 共分散行列 P の対角成分、速度 Z の分散。',
                        'description_en': 'ESKF covariance matrix P diagonal entry, '
                                          'velocity-Z variance.'},
                    {   'name': 'p_att_x',
                        'type': 'float',
                        'unit': 'rad^2',
                        'description_ja': 'ESKF 共分散行列 P の対角成分、姿勢誤差角 X の分散。',
                        'description_en': 'ESKF covariance matrix P diagonal entry, '
                                          'attitude-error-angle-X variance.'},
                    {   'name': 'p_att_y',
                        'type': 'float',
                        'unit': 'rad^2',
                        'description_ja': 'ESKF 共分散行列 P の対角成分、姿勢誤差角 Y の分散。',
                        'description_en': 'ESKF covariance matrix P diagonal entry, '
                                          'attitude-error-angle-Y variance.'},
                    {   'name': 'p_att_z',
                        'type': 'float',
                        'unit': 'rad^2',
                        'description_ja': 'ESKF 共分散行列 P の対角成分、姿勢誤差角 Z の分散。',
                        'description_en': 'ESKF covariance matrix P diagonal entry, '
                                          'attitude-error-angle-Z variance.'},
                    {   'name': 'p_bg_x',
                        'type': 'float',
                        'unit': '(rad/s)^2',
                        'description_ja': 'ESKF 共分散行列 P の対角成分、ジャイロバイアス X の分散。',
                        'description_en': 'ESKF covariance matrix P diagonal entry, '
                                          'gyro-bias-X variance.'},
                    {   'name': 'p_bg_y',
                        'type': 'float',
                        'unit': '(rad/s)^2',
                        'description_ja': 'ESKF 共分散行列 P の対角成分、ジャイロバイアス Y の分散。',
                        'description_en': 'ESKF covariance matrix P diagonal entry, '
                                          'gyro-bias-Y variance.'},
                    {   'name': 'p_bg_z',
                        'type': 'float',
                        'unit': '(rad/s)^2',
                        'description_ja': 'ESKF 共分散行列 P の対角成分、ジャイロバイアス Z の分散。',
                        'description_en': 'ESKF covariance matrix P diagonal entry, '
                                          'gyro-bias-Z variance.'},
                    {   'name': 'p_ba_x',
                        'type': 'float',
                        'unit': '(m/s^2)^2',
                        'description_ja': 'ESKF 共分散行列 P の対角成分、加速度バイアス X の分散。',
                        'description_en': 'ESKF covariance matrix P diagonal entry, '
                                          'accel-bias-X variance.'},
                    {   'name': 'p_ba_y',
                        'type': 'float',
                        'unit': '(m/s^2)^2',
                        'description_ja': 'ESKF 共分散行列 P の対角成分、加速度バイアス Y の分散。',
                        'description_en': 'ESKF covariance matrix P diagonal entry, '
                                          'accel-bias-Y variance.'},
                    {   'name': 'p_ba_z',
                        'type': 'float',
                        'unit': '(m/s^2)^2',
                        'description_ja': 'ESKF 共分散行列 P の対角成分、加速度バイアス Z の分散。',
                        'description_en': 'ESKF covariance matrix P diagonal entry, '
                                          'accel-bias-Z variance.'}],
    'truth': [   {   'name': 'timestamp_us',
                     'type': 'int',
                     'unit': 'us',
                     'description_ja': 'シミュレータの仮想時計によるマイクロ秒タイムスタンプ。',
                     'description_en': "Microsecond timestamp from the simulator's "
                                       'virtual clock.'},
                 {   'name': 'pos_x',
                     'type': 'float',
                     'unit': 'm',
                     'description_ja': '物理モデルの真の位置 X（NED 座標系）。',
                     'description_en': 'Physics-model ground-truth position X (NED '
                                       'frame).'},
                 {   'name': 'pos_y',
                     'type': 'float',
                     'unit': 'm',
                     'description_ja': '物理モデルの真の位置 Y（NED 座標系）。',
                     'description_en': 'Physics-model ground-truth position Y (NED '
                                       'frame).'},
                 {   'name': 'pos_z',
                     'type': 'float',
                     'unit': 'm',
                     'description_ja': '物理モデルの真の位置 Z（NED 座標系、下向き正）。',
                     'description_en': 'Physics-model ground-truth position Z (NED '
                                       'frame, positive down).'},
                 {   'name': 'quat_w',
                     'type': 'float',
                     'unit': 'unitless',
                     'description_ja': '物理モデルの真の姿勢クォータニオン実部 w。',
                     'description_en': 'Physics-model ground-truth attitude quaternion '
                                       'real part w.'},
                 {   'name': 'quat_x',
                     'type': 'float',
                     'unit': 'unitless',
                     'description_ja': '物理モデルの真の姿勢クォータニオン虚部 x。',
                     'description_en': 'Physics-model ground-truth attitude quaternion '
                                       'imaginary part x.'},
                 {   'name': 'quat_y',
                     'type': 'float',
                     'unit': 'unitless',
                     'description_ja': '物理モデルの真の姿勢クォータニオン虚部 y。',
                     'description_en': 'Physics-model ground-truth attitude quaternion '
                                       'imaginary part y.'},
                 {   'name': 'quat_z',
                     'type': 'float',
                     'unit': 'unitless',
                     'description_ja': '物理モデルの真の姿勢クォータニオン虚部 z。',
                     'description_en': 'Physics-model ground-truth attitude quaternion '
                                       'imaginary part z.'},
                 {   'name': 'vel_x',
                     'type': 'float',
                     'unit': 'm/s',
                     'description_ja': '物理モデルの真の速度 X（NED 座標系）。',
                     'description_en': 'Physics-model ground-truth velocity X (NED '
                                       'frame).'},
                 {   'name': 'vel_y',
                     'type': 'float',
                     'unit': 'm/s',
                     'description_ja': '物理モデルの真の速度 Y（NED 座標系）。',
                     'description_en': 'Physics-model ground-truth velocity Y (NED '
                                       'frame).'},
                 {   'name': 'vel_z',
                     'type': 'float',
                     'unit': 'm/s',
                     'description_ja': '物理モデルの真の速度 Z（NED 座標系、下向き正）。',
                     'description_en': 'Physics-model ground-truth velocity Z (NED '
                                       'frame, positive down).'},
                 {   'name': 'rate_x',
                     'type': 'float',
                     'unit': 'rad/s',
                     'description_ja': '物理モデルの真の角速度 X（機体座標系）。',
                     'description_en': 'Physics-model ground-truth angular rate X '
                                       '(body frame).'},
                 {   'name': 'rate_y',
                     'type': 'float',
                     'unit': 'rad/s',
                     'description_ja': '物理モデルの真の角速度 Y（機体座標系）。',
                     'description_en': 'Physics-model ground-truth angular rate Y '
                                       '(body frame).'},
                 {   'name': 'rate_z',
                     'type': 'float',
                     'unit': 'rad/s',
                     'description_ja': '物理モデルの真の角速度 Z（機体座標系）。',
                     'description_en': 'Physics-model ground-truth angular rate Z '
                                       '(body frame).'}],
    'events': [   {   'name': 'timestamp_us',
                      'type': 'int',
                      'unit': 'us',
                      'description_ja': 'シミュレータの仮想時計によるマイクロ秒タイムスタンプ。',
                      'description_en': "Microsecond timestamp from the simulator's "
                                        'virtual clock.'},
                  {   'name': 'event',
                      'type': 'str',
                      'unit': 'n/a',
                      'description_ja': '事象名（シナリオ入力: モード切替・ステップ入力等）。',
                      'description_en': 'Event name (scenario input: mode switch, step '
                                        'input, etc.).'},
                  {   'name': 'value',
                      'type': 'str',
                      'unit': 'n/a',
                      'description_ja': '事象の値（自由形式の文字列。数値の場合も文字列化して記録）。',
                      'description_en': 'Event value (free-form string; numeric values '
                                        'are stringified).'}]}


def schema_for(stream_names):
    """Build a schema.json-compatible dict for the given stream names.

    Args:
        stream_names: iterable of stream names to include -- normally
            the streams actually present in one bundle, which may be
            fewer than the full STREAMS set (e.g. a bundle with no
            magnetometer data has no 'mag' stream). Names not found in
            STREAMS are silently skipped.

    Returns:
        {'format': FORMAT, 'version': VERSION, 'streams': {name: {file,
        source, nominal_rate_hz, required, columns}}} where columns is
        the list of per-column metadata dicts from _COLUMN_META.

    渡されたストリーム名から schema.json 相当の dict を作る。

    引数 stream_names は通常、そのバンドルに実際に存在するストリーム
    （STREAMS 全体より少ないことがある。例: 地磁気センサ無しのバンドル
    には 'mag' が無い）。STREAMS に無い名前は黙って無視する。
    """
    streams = {}
    for name in stream_names:
        if name not in STREAMS:
            continue
        info = STREAMS[name]
        streams[name] = {
            'file': info['file'],
            'source': info['source'],
            'nominal_rate_hz': info['nominal_rate_hz'],
            'required': info['required'],
            'columns': _COLUMN_META[name],
        }
    return {'format': FORMAT, 'version': VERSION, 'streams': streams}
