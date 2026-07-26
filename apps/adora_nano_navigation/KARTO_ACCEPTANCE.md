# Dora Karto 实车验收

## 当前结论

- 软件构建与单元/回归测试：通过（13/13）
- 合成数据确定性回放：通过
- ROS 源码、动态库、CMake、生产 YAML 和进程审计：`ROS_FREE=PASS`
- 真实小车建图、定位、自主导航：尚未验收

合成数据只能证明软件链路可重复，不能证明 MS200、底盘里程计、时间戳、标定和真实环境下的效果达到 ROS 2 `slam_toolbox` 水平。

## 实车测试前

```bash
cd ~/adora_nano_dora_navigation
python3 apps/adora_nano_navigation/adora_nav.py check --mode mapping
apps/adora_nano_navigation/verify_ros_free.sh
```

确认雷达和底盘均使用 `/dev/serial/by-id/...`，没有旧 Dora 流占用串口。键盘控制为空格停车；基线建图避免横移，保持低速。

## 新建图

```bash
python3 apps/adora_nano_navigation/adora_nav.py map --clean
```

完整绕行后停止 Dora，让 Stop 事件触发最终原子保存。必须生成：

- `ms200_keyboard_map.pgm`
- `ms200_keyboard_map.yaml`
- `ms200_keyboard_map.metadata.json`
- `ms200_keyboard_map.posegraph.dora`

通过标准：扫描都有前后里程计包围；无无法解释的连续丢帧；至少一个有效回环；终点闭合误差不超过 `0.10 m / 5°`；主要墙体双影间距不超过 `0.10 m`。

## 继续建图

```bash
python3 apps/adora_nano_navigation/adora_nav.py map --continue
```

确认旧关键帧保留，新区域加入后再次保存，旧地图区域不漂移或消失。

## 定位与导航

```bash
python3 apps/adora_nano_navigation/adora_nav.py nav --relative 0.5 0.0 --localize
```

沿完整路线检查定位置信度、短暂失配恢复和连续失配安全停车。随后执行多个自主导航目标，并人工遮挡雷达验证 `SafetyStop`。

## 数据采集

下一次实车测试应保存原始 `LaserScan/Odometry` 事件，而不仅是 stdout：

```bash
localization/karto_slam_dora_node/tools/record_dataset.py real_car_route.jsonl
```

真实数据集需要与 ROS 2 参考采用同一路线，之后用 `replay_dataset` 重放并比较地图指标。
