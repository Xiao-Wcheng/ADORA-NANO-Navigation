# Dora 自主导航项目收尾设计

日期：2026-07-24

## 目标

把 `/home/ubuntu2204/adora_nano_dora_navigation` 收敛为一个只包含当前生产方案、能够在无 ROS 环境中独立构建和运行的 Dora 自主导航项目。生产局部控制器唯一采用 ROS-free Nav2 Regulated Pure Pursuit（RPP）移植。

## 范围

- 永久删除不再参与生产运行的 DWB 与 MPPI 源码、构建目录及相关生产引用。
- 三套导航数据流统一使用 `planning/rpp_local_controller_dora_node`。
- 更新根入口测试、独立路径测试、生产接线测试和项目清单。
- 将错误命名的 MPPI 生产接线测试改为 RPP 接线测试。
- README、组件说明、操作说明、开源来源和验证报告统一描述当前 RPP 架构。
- 修正终点状态一致性：Supervisor 锁定 `REACHED` 后，安全底盘输出持续为零；RPP 不应继续造成可执行的终点旋转命令。
- 完成无电机静态验收与自动化验收。

## 不在范围内

- 不重新设计建图、定位、A* 或 RPP 算法。
- 不调整当前实车运动速度、碰撞半径、安全边距或地图膨胀参数。
- 不启动底盘或进行新的实车运动测试。
- 不安装 ROS，也不引入 ROS 运行时依赖。
- 不创建 Git 提交。

## 生产架构

生产数据链保持：

```text
MS200 雷达 + Feetech 底盘里程计
  -> Karto/OpenKarto 建图或定位
  -> Boost.Graph A* 全局规划
  -> Nav2 RPP ROS-free 局部控制
  -> 速度平滑
  -> Supervisor 安全门
  -> 底盘速度指令
```

`REACHED` 是终端锁定状态。锁定后 Supervisor 必须输出零速度，直到流程停止或接收明确的新目标。局部规划器内部状态不得绕过该安全门。

## 清理规则

- 删除 `planning/dwb_local_planner_dora_node/`。
- 删除 `planning/mppi_local_planner_dora_node/`。
- 生产 README、docs、scripts、tests、清单和 YAML 中不得存在 DWB/MPPI 组件引用。
- 历史设计/计划文档如继续保留，必须明确标注为历史记录，不得被生产一致性扫描误判为当前架构。
- 保留 RPP 上游来源说明和许可证。

## 测试与验收

采用测试先行：

1. 先修改或新增结构测试，使其在旧目录、旧引用或错误终点行为存在时失败。
2. 删除旧实现、更新接线和文档，使测试转绿。
3. 运行 RPP、Supervisor、A*、Karto、雷达驱动的现有 CTest。
4. 运行 Python 结构与入口测试。
5. 执行 `build_all.sh`、`check_ready.sh` 和所有生产 Dora YAML 验证。
6. 扫描可执行文件，确认没有 ROS 动态库依赖。
7. 确认没有导航、定位、键盘控制或底盘控制残留进程。

验收成功条件：

- 干净构建退出码为 0。
- 全部自动化测试零失败。
- `check_ready.sh` 输出 `READY`。
- 三套生产导航 YAML 均只引用 RPP。
- 项目生产范围内无 DWB/MPPI 残留。
- Supervisor 终点锁定零速度测试通过。
- 没有启动电机或遗留控制进程。

## 删除与恢复

旧 DWB/MPPI 目录按用户确认永久从当前项目删除。本轮不创建 Git 提交；若需要恢复，只能依赖 Ubuntu 端现有的外部备份目录，而不是当前工作树。
