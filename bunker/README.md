# Bunker Gazebo 仿真使用说明

本目录提供 ROS 2 Humble 与 Gazebo Classic 11 环境下的 Bunker、Bunker
Mini 和 Bunker Pro 2.0 仿真功能包。

Bunker 仿真使用 `gazebo_ros_planar_move` 插件：

- 订阅速度控制话题 `/cmd_vel`
- 发布里程计话题 `/odom`
- 发布 `odom -> base_link` 坐标变换

## 1. 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- `gazebo_ros_pkgs`
- `xacro`
- `robot_state_publisher`
- `joint_state_publisher`

安装常用依赖：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher
```

## 2. 编译功能包

```bash
cd ~/ugv_sim_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select \
  bunker_description \
  bunker_gazebo_sim \
  bunker_mini \
  bunker_pro2

source ~/ugv_sim_ws/install/setup.bash
```

## 3. 配置模型路径

> 三款车型的 URDF 都使用 `package://<package>/meshes/...` 引用 STL 网格。
> 启动前必须将对应功能包的安装目录加入 `GAZEBO_MODEL_PATH`，否则 Gazebo
> 中可能存在模型实体，但模型外观不显示，并出现 `No mesh specified` 或
> `File or path does not exist`。

每次打开新终端后，依次执行：

```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.bash
source ~/ugv_sim_ws/install/setup.bash

for pkg in bunker_description bunker_mini bunker_pro2; do
  GAZEBO_MODEL_PATH="$(ros2 pkg prefix "$pkg")/share:${GAZEBO_MODEL_PATH:-}"
done
export GAZEBO_MODEL_PATH
```

该写法通过 ROS 2 自动查找功能包安装位置，不依赖用户名或工作空间的绝对
路径。它等价于：

```bash
export GAZEBO_MODEL_PATH="/home/agilex/ugv_sim_ws/install/bunker_description/share:/home/agilex/ugv_sim_ws/install/bunker_mini/share:/home/agilex/ugv_sim_ws/install/bunker_pro2/share:${GAZEBO_MODEL_PATH:-}"
```

检查三款车型的路径是否都已加入：

```bash
echo "$GAZEBO_MODEL_PATH"
for pkg in bunker_description bunker_mini bunker_pro2; do
  ros2 pkg prefix "$pkg"
done
```

## 4. 启动仿真

启动标准 Bunker：

```bash
ros2 launch bunker_gazebo_sim bunker_gazebo_sim.launch.py \
  gui:=true verbose:=true
```

启动 Bunker Mini：

```bash
ros2 launch bunker_mini gazebo.launch.py gui:=true verbose:=true
```

启动 Bunker Pro 2.0：

```bash
ros2 launch bunker_pro2 gazebo.launch.py gui:=true verbose:=true
```

## 5. 启动参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `gui` | `true` | 是否启动 Gazebo 图形界面 |
| `verbose` | `false` | 是否输出 Gazebo 详细日志 |
| `paused` | `false` | 是否以暂停状态启动 |
| `use_sim_time` | `true` | 是否使用仿真时间 |
| `entity_name` | `bunker` | Gazebo 中的模型名称 |
| `x` | `0.0` | 初始 X 坐标 |
| `y` | `0.0` | 初始 Y 坐标 |
| `z` | `0.4` | 初始 Z 坐标 |
| `yaw` | `0.0` | 初始偏航角 |
| `world` | Gazebo 空世界 | 指定 world 文件 |

无图形界面运行：

```bash
ros2 launch bunker_gazebo_sim bunker_gazebo_sim.launch.py \
  gui:=false verbose:=true
```

指定初始位置：

```bash
ros2 launch bunker_gazebo_sim bunker_gazebo_sim.launch.py \
  x:=1.0 y:=2.0 z:=0.4 yaw:=1.57
```

## 6. 控制车辆

新建终端并加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_sim_ws/install/setup.bash
```

发布速度指令：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0}, angular: {z: 0.3}}" -r 10
```

停止车辆：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}"
```

查看里程计：

```bash
ros2 topic echo /odom
```

## 7. 常见问题

### Gazebo 中没有显示 Bunker 模型

检查模型路径：

```bash
echo "$GAZEBO_MODEL_PATH"
```

如果没有同时包含三款车型的 `share` 路径，重新执行：

```bash
for pkg in bunker_description bunker_mini bunker_pro2; do
  GAZEBO_MODEL_PATH="$(ros2 pkg prefix "$pkg")/share:${GAZEBO_MODEL_PATH:-}"
done
export GAZEBO_MODEL_PATH
```

关闭当前 Gazebo 后重新启动仿真。

### 出现 `No mesh specified`

典型日志：

```text
[Err] [Visual.cc:2956] No mesh specified
[Wrn] [SystemPaths.cc:459] File or path does not exist
```

这表示 Gazebo 无法解析 `model://<package>/meshes/...`。检查三个功能包及其
网格目录：

```bash
for pkg in bunker_description bunker_mini bunker_pro2; do
  ls "$(ros2 pkg prefix "$pkg")/share/$pkg/meshes"
done
```

确认文件存在后，重新设置 `GAZEBO_MODEL_PATH`。

### Gazebo 窗口空白或着色器加载失败

如果出现 `Unable to find shader lib` 或 `Failed to initialize scene`：

```bash
source /usr/share/gazebo/setup.bash
```

然后重新启动 Gazebo。

### 出现 Fuel 或 libcurl 网络警告

Bunker 网格位于本地，不需要从 Gazebo Fuel 下载。Fuel、模型数据库或
`libcurl` 网络警告通常不会阻止本地模型运行，应优先检查
`GAZEBO_MODEL_PATH`。

## 8. 关闭仿真

在运行 launch 的终端按 `Ctrl+C`。关闭后可以检查是否还有 Gazebo 进程：

```bash
pgrep -af 'gzserver|gzclient'
```
