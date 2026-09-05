# Lesson 2: Open-Loop Control (Manual Mixing)

## Goal / 目標
Read stick values into variables and compute each motor's Duty using arithmetic (manual mixing).
スティック値を変数に読み取り、四則演算で各モータの Duty を手動計算する。

## Prerequisites / 前提条件
- Controller paired with StampFly (3-second button press to pair)

## API / 使用するAPI
| Function | Description | Range |
|----------|-------------|-------|
| `ws::rc_throttle()` | Throttle stick | 0.0 to 1.0 |
| `ws::rc_roll()` | Roll stick | -1.0 to +1.0 |
| `ws::rc_pitch()` | Pitch stick | -1.0 to +1.0 |
| `ws::rc_yaw()` | Yaw stick | -1.0 to +1.0 |
| `ws::is_armed()` | ARM status | true/false |
| `ws::motor_set_duty(id, v)` | Set individual motor | id=1-4, v=0.0-1.0 |

## Motor Mixing Sign Convention / モータミキシング符号規則
```
         Front
    FL(M4)  FR(M1)
        \ ^ /
         \|/
          X
         /|\
    RL(M3)  RR(M2)
         Rear
```

| Motor | T | Roll | Pitch | Yaw |
|-------|---|------|-------|-----|
| M1 FR | + | - | + | + |
| M2 RR | + | - | - | - |
| M3 RL | + | + | - | + |
| M4 FL | + | + | + | - |

## Steps / 手順
1. `sf lesson switch 2`
2. Pair controller (3s button hold on StampFly)
3. Read TRPY stick values into variables with scaling (`* 0.3f`)
4. Compute each motor Duty: e.g. `M1 = T + R - P - Y`
5. Set motors with `ws::motor_set_duty(id, duty)`
6. Verify: pitch stick creates front/rear speed difference
7. Try changing gain (0.3f) and observe behavior

## Key Concepts / キーコンセプト
- Variable declaration and arithmetic (`float t = ...`)
- Scaling stick input (`* 0.3f` for gentle response)
- Manual motor mixing (addition / subtraction per motor)
- Open-loop control limitations (no feedback)
