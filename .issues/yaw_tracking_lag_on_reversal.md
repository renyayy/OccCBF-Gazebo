# VelocityTrackingYaw: 方向転換時に向きと移動方向が乖離する

## 症状

ロボットがゴールを通り過ぎて反転する際、向きは正しくゴール方向を向くが、
その後ゴールから遠ざかる方向に移動する（向きと移動方向が逆になる）。

## 根本原因

`VelocityTrackingYaw` が `DoubleIntegrator2D` の場合に引数 `u` を無視し、
`robot_state[2:4]`（現在のオドメトリ速度）を参照している。

### 該当コード

**velocity_tracking_yaw.py:42-44**
```python
elif self.model == 'DoubleIntegrator2D':
    vx = robot_state[2, 0]  # 現在速度を使用
    vy = robot_state[3, 0]  # 引数 u は無視
```

**cbf_wrapper_node.py:157-180** の制御フロー:
```
1. yaw制御: desired_yaw = atan2(current_vy, current_vx)  ← 現在速度
2. 指令速度: vx = current_vx + accel * dt                 ← 加速度適用後
3. Twist発行: linear=(指令速度), angular.z=(yaw制御出力)
```

yaw制御が見ている速度方向と、実際にGazeboに送る速度方向が1フレーム以上ずれる。

## 発生シーケンス

```
フレームN: robot=[21,7.5], goal=[20,7.5], 現在速度=+0.5(→)
  CBF-QP → 加速度=-1.0(←), 指令速度=+0.45(→)
  yaw制御: atan2(0, +0.5)=0° → 変化なし
  → 正常動作

フレームN+k: 速度反転、現在速度=-0.05(←)
  CBF-QP → 加速度=-1.0, 指令速度=-0.10(←)
  yaw制御: atan2(0, -0.05)=π, yaw_err=π → u_att=clip(1.5π, ±1.0)=1.0(飽和)
  → ←に移動しつつ回転開始。向きはまだ→。見た目は逆走。

フレームN+k+1~: w_max=1.0 rad/sの飽和で180°回転に~π秒かかる
  → その間、向きと移動方向が逆のまま
```

## 修正方針

`VelocityTrackingYaw` の `DoubleIntegrator2D` 分岐で、
`robot_state[2:4]` ではなく `u`（指令速度）を使用する。

### velocity_tracking_yaw.py
```python
# Before:
elif self.model == 'DoubleIntegrator2D':
    vx = robot_state[2, 0]
    vy = robot_state[3, 0]

# After:
elif self.model == 'DoubleIntegrator2D':
    vx = u[0, 0]
    vy = u[1, 0]
```

### cbf_wrapper_node.py
指令速度の計算をyaw制御の前に移動し、指令速度をyaw制御に渡す:
```python
# 指令速度を先に計算
vx = self.X[2, 0] + float(u[0, 0]) * self.dt
vy = self.X[3, 0] + float(u[1, 0]) * self.dt
v_max = self.robot_spec['v_max']
v_norm = np.hypot(vx, vy)
if v_norm > v_max:
    vx, vy = vx / v_norm * v_max, vy / v_norm * v_max

# 指令速度でyaw制御
commanded_vel = np.array([[vx], [vy]])
u_yaw = self.attitude_controller.solve_control_problem(
    robot_state=self.X, current_yaw=self.current_yaw, u=commanded_vel
)
```

## 影響範囲

- `safe_control/attitude_control/velocity_tracking_yaw.py` (L42-44)
- `scripts/cbf_wrapper_node.py` (L157-175)
- `SingleIntegrator2D` の分岐は既に `u` を使用しており影響なし
