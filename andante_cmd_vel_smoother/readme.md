# andante_cmd_vel_smoother
## Description
cmd_velを1次遅れLPFに通して平滑化するノード。
linear.z / angular.x / angular.y は0になります。

## Topic
### Publisher
| Topic Name | Message Type        | Description |
| ---------- | ------------------- | ----------- |
| cmd_vel    | geometry_msgs/Twist |             |
### Subscriber
| Topic Name         | Message Type        | Description                                                             |
| ------------------ | ------------------- | ----------------------------------------------------------------------- |
| cmd_vel_unfiltered | geometry_msgs/Twist |                                                                         |
| bumper             | create_msgs/Bumper  | バンパーが押された場合、emergency stateに入り、目標速度を0にする。      |
| cliff              | create_msgs/Cliff   | cliffセンサーが反応した場合、emergency stateに入り、目標速度を0にする。 |
| wheeldrop          | std_msgs/Empty      | パブリッシュされるとemergency stateに入り、目標速度を0にする。          |

## Parameters
| Name                 | Description                                                                        | Default |
| -------------------- | ---------------------------------------------------------------------------------- | ------- |
| rate                 | 更新周期                                                                           | 30      |
| cutoff_freq          | LPFのカットオフ周波数                                                              | 10      |
| max_linear_acc       | 並進方向の最大加速度(m/s^2)                                                        | 0.5     |
| max_angler_acc       | 回転方向の最大加速度(rad/s^2)                                                      | 1.0     |
| cmd_vel_reset_time   | この値の秒数だけ速度司令が来なかった場合、速度司令を0にリセットする。              | 5.0     |
| emergency_reset_time | emergency_stateに入った後、状況がクリアされてからemergency_stateを抜けるまでの秒数 | 5.0     |
