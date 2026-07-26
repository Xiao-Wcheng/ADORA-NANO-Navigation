# 全局路径规划算法

纯 Dora 全局路径规划器。A* 搜索核心使用成熟的开源 Boost.Graph，
不依赖 ROS；Dijkstra 作为兼容和对照算法保留。

## 功能特性

- ✅ Boost.Graph A*（8方向移动、Octile启发函数）
- ✅ 对角墙角穿越防护
- ✅ 起终点合法性和不可达检测
- ✅ 带安全墙角检查的视线路径简化
- ✅ Dijkstra算法实现（支持8方向移动）
- ✅ 栅格地图加载（PGM + YAML格式）
- ✅ 路径输出（文本格式）
- ✅ 性能统计（路径长度、计算时间）
- ✅ Rerun可视化支持
- ✅ DORA节点接口（JSON输出）

## 项目结构

```
planning/
├── CMakeLists.txt          # CMake构建配置
├── include/                # 头文件
│   ├── map_loader.h       # 地图加载器
│   ├── astar.h            # A*算法
│   └── dijkstra.h         # Dijkstra算法
├── src/                    # 源文件
│   ├── map_loader.cpp
│   ├── astar.cpp
│   ├── dijkstra.cpp
│   ├── dora_planning_node.cpp  # DORA节点实现
│   ├── dora_node_main.cpp      # DORA节点主程序
│   ├── test_planning.cpp  # 测试节点
│   └── visualize_planning.cpp
├── map/                    # 地图文件
│   ├── test_map.pgm
│   └── test_map.yaml
├── config.yaml            # DORA节点配置文件
├── visualize.py           # Python可视化脚本
└── README.md
```

## 依赖项
- CMake >= 3.10
- C++17编译器（g++/clang++）
- yaml-cpp库
- Boost.Graph头文件（`libboost-dev`）

### 可选（用于可视化）
- Python 3
- rerun-sdk
- Pillow
- PyYAML

## 运行
### 1. 安装依赖
```bash
# Ubuntu/Debian
sudo apt-get install cmake g++ libyaml-cpp-dev libboost-dev

# macOS
brew install cmake yaml-cpp
```

### 2. 编译项目
```bash
mkdir build
cd build
cmake ..
make
ctest --output-on-failure
```

### 3. 运行方式1：独立测试
运行测试节点（不使用DORA）：
```bash
cd build
./test_planning
```

### 3.运行方式2： 使用DORA节点

```bash
dora up
dora start dataflow.yml
```

标记起点（蓝色）和终点（黄色），A*路径显示为红色，Dijkstra路径显示为绿色。

**输入：**

- `start_point`: JSON格式的起点坐标 `{"x": 50, "y": 50}`
- `goal_point`: JSON格式的终点坐标 `{"x": 400, "y": 400}`

**输出：**
- `path`: JSON格式的路径数据

**输出格式：**
```json
{
  "algorithm": "astar",
  "path_found": true,
  "path_length": 494.85,
  "compute_time_ms": 12.34,
  "num_waypoints": 350,
  "waypoints": [
    {"x": 50, "y": 50},
    {"x": 51, "y": 51},
    ...
  ]
}
```

**环境变量：**
- `MAP_YAML_PATH`: 地图文件路径（默认：map/test_map.yaml）
- `ALGORITHM`: 算法类型，"astar" 或 "dijkstra"（默认：astar）


## 故障排除

### 问题1：DORA节点无法启动
- 检查是否已编译：`./build.sh`
- 检查可执行文件：`ls -la build/dora_planning_node`

### 问题2：地图加载失败
- 检查地图文件路径
- 确认test_map.yaml和test_map.pgm存在

### 问题3：Rerun窗口不显示
- 确保安装了rerun-sdk：`pip install rerun-sdk`
- 检查Python节点是否正常运行

### 问题4：路径规划失败
- 检查起点和终点是否在地图范围内
- 确认起点和终点不在障碍物上
- 查看控制台输出的错误信息
