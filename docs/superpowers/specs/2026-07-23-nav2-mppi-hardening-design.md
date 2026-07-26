# Nav2 MPPI Dora 适配加固设计

## 目标

修复 Dora 导航中 MPPI 参数未生效、候选轨迹偶发全部无效和实车低速死区问题，同时保持纯 Dora 运行，不依赖 ROS 2 运行时。

## 上游依据

- 控制优化：移植 Nav2 `nav2_mppi_controller` 的 critic 参数化、失败软重试和优化序列复位语义。
- 速度输出：移植 Nav2 `nav2_velocity_smoother` 的逐轴速度上下限、加减速度约束、deadband 和超时停车语义。
- 安全职责：保留独立 supervisor/安全停车层，不在 MPPI 内通过强制放大速度绕过碰撞结果。

## 设计

1. 为 `CriticConfig` 建立独立的环境变量装配函数，使 YAML 中现有权重实际进入优化器。
2. 在 `MppiOptimizer::compute` 内按 Nav2 的 `retry_attempt_limit` 语义进行有限次软复位和重新采样；每次仍无有限代价才报告失败。
3. 不把 MPPI 的微小速度直接放大。在 MPPI 与 supervisor 之间增加纯算法速度平滑模块，按 Nav2 Velocity Smoother 语义处理 deadband、加速度限制和零指令。
4. 诊断状态报告候选总数、有限轨迹数、碰撞轨迹数和重试次数。
5. Collision Monitor 风格的停止职责继续由 supervisor 承担；定位、雷达或规划超时仍立即输出零。

## 安全约束

- `SAFETY_MARGIN=0.04 m` 不降低。
- 任何上游零指令、超时、定位丢失或无安全轨迹均输出零。
- 不启动真实导航进行软件验证。
- 不引入 ROS 2 运行时依赖。
- 不提交 Git。

## 验收

- 自动测试证明 critic 参数进入评分配置。
- 自动测试证明第一次无解、软复位后有解时返回有效控制，达到重试上限后安全失败。
- 自动测试证明 deadband、加减速和零指令行为与 Nav2 Velocity Smoother 语义一致。
- MPPI、supervisor、全局规划器测试全部通过，完整工程构建通过。
