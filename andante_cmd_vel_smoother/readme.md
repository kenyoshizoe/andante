# andante_cmd_vel_smoother
## Description
cmd_velを1次遅れLPFに通して平滑化するノード。
linear.z / angular.x / angular.y は0になります。

## Topic
### Publisher
| Topic Name       | Message Type        | Description |
| ---------------- | ------------------- | ----------- |
| cmd_vel_filtered | geometry_msgs/Twist |             |
### Subscriber
| Topic Name | Message Type        | Description |
| ---------- | ------------------- | ----------- |
| cmd_vel    | geometry_msgs/Twist |             |

## Parameters
| Name           | Description                   | Default |
| -------------- | ----------------------------- | ------- |
| rate           | 更新周期                      | 30      |
| cutoff_freq    | LPFのカットオフ周波数         | 10      |
| max_linear_acc | 並進方向の最大加速度(m/s^2)   | 0.5     |
| max_angler_acc | 回転方向の最大加速度(rad/s^2) | 1.0     |
