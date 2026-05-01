# Pickup Full-System Runbook

这份文档只保留一条标准路径：

**完整 pickup 系统启动路径**

目标读者是：

- 第一次接手这个项目的测试者
- 想直接把整条 pickup 链路拉起来的人
- 不希望在“最小模式”和“完整模式”之间自己判断的人

## 1. 先说结论

当前仓库还没有真正的一键 full-system bringup。

所以现在最稳的做法不是“找一个万能 launch”，而是按固定顺序起下面这几块：

1. backend
2. 可选 UI
3. navigation stack
4. 可选 `rosbridge_server`，用于远端实时 SLAM 地图展示
5. MoveIt + `/pick_viperx`
6. `camera_vision`
7. `bt_executor_viperX`

然后通过：

- backend / UI 下单
- 或 `/order/new` 手工发单

来驱动完整 pickup 流程。

## 2. 当前 full-system 的重要规则

### 2.1 BT 主流程不要起 `vx300_auto_pick.launch.py`

完整 pickup 测试时，**不要**用：

```bash
ros2 launch robot_manipulation vx300_auto_pick.launch.py ...
```

原因是：

- 这个 launch 会把旧的 `vision_auto_pick.py` 一起拉起来
- 旧 `vision_auto_pick.py` 也会订阅 `/detections_json`
- 它也会向 `/pick_viperx` 发动作
- 会和 `bt_executor_viperX` 同时抢机械臂控制

所以 full-system 的正确组合是：

- 导航单独起
- 机械臂用 `vx300_moveit.launch.py`
- 视觉单独起 `camera_vision`
- 任务树单独起 `bt_executor_viperX`

### 2.2 串口使用稳定 by-id 名称

当前仓库默认已经切到稳定的 USB 设备路径，不再依赖 `ttyUSB0` / `ttyUSB1` 的分配顺序。

完整系统测试时，底盘和机械臂分别固定为：

- 底盘：`/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`
- 机械臂：`/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT88YTG6-if00-port0`

### 2.3 这份 runbook 默认关闭 Nav2 RViz

这份 runbook 默认把 Nav2 的 RViz 关掉。

原因是：

- 这条路径默认按远端 / 无桌面环境使用来写
- 我们希望实时地图展示走 `rosbridge_server`
- 这样不需要依赖本地桌面会话才能看地图

如果你确实要本地打开 Nav2 RViz 做调试，可以把：

- `--with-nav2-rviz false`

改成：

- `--with-nav2-rviz true`

### 2.4 BT 主入口

完整 pickup 主流程的入口是：

```bash
ros2 run robot_task_manager bt_executor_viperX
```

注意最后是大写 `X`。

## 3. 一次性准备

如果你改过源码，先重新构建：

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
colcon build --base-paths src --packages-select \
  robot_interfaces robot_navigation robot_manipulation robot_vision robot_task_manager
```

下面所有 ROS 终端都建议统一这样准备环境：

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
```

为了减少测试者出错，下面第 3 到第 7 个终端的命令块都已经把 `cd` 和 `source` 写完整了，可以直接复制。

## 4. 标准完整启动顺序

下面是当前推荐的 full-system 启动顺序。

### Terminal 1: Backend

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/order-api-postgre
npm install
npm run dev
```

这一步成功后，后端地址应该是：

- `http://localhost:3000`

前提：

- PostgreSQL 已经准备好
- backend schema / seed 已经初始化过

### Terminal 2: Fleet Manager UI

如果你需要前端管理界面，再开一个终端：

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/order-api-postgre/fleet-manager
npm install
npm run dev -- --host 0.0.0.0
```

前端地址通常是：

- `http://localhost:5174`

说明：

- pickup 主链真正依赖的是 backend
- fleet-manager 主要是可视化和管理入口

### Terminal 3: Navigation Stack

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
ros2 run robot_navigation nav_assistant localization-stack \
  --map-name ECEMain \
  --with-base-link-crop true \
  --bridge-axis-deadband 0.03 \
  --with-nav2-rviz false \
  --serial-port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

这一步会启动：

- localization
- Nav2
- semantic map service
- serial bridge

这份 runbook 故意把本地 Nav2 RViz 默认关掉。

### Terminal 4: 可选 rosbridge，用于远端实时 SLAM 地图

如果机器上还没装过，只需要安装一次：

```bash
sudo apt install ros-$ROS_DISTRO-rosbridge-server
```

启动命令：

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

在 embedded app 里连接：

- `ws://192.168.8.249:9090`

说明：

- 连接页默认会填当前 hostname 和端口 `9090`
- 如果是从另一台机器访问，记得把 hostname / IP 改成机器人主机地址
- 这就是本 runbook 推荐的远端实时 SLAM 地图展示方式

### Terminal 5: MoveIt + `/pick_viperx`

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
ros2 launch robot_manipulation vx300_moveit.launch.py \
  robot_model:=vx300s \
  robot_name:=vx300s \
  motor_port:=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT88YTG6-if00-port0
```

这一步会提供：

- MoveIt
- ViperX 控制器
- `/pick_viperx`

### Terminal 6: Camera Vision

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
ros2 run robot_vision camera_vision --ros-args \
  -p parent_frame:=vx300s/ee_gripper_link \
  -p camera_mount_frame:=camera_mount_frame \
  -p camera_optical_frame:=camera_color_optical_frame
```

如果要看实时图像窗口，可以加：

```bash
-p show_live_window:=true
```

### Terminal 7: BT Executor

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
ros2 run robot_task_manager bt_executor_viperX
```

这一步之后，BT 会开始：

- 轮询 backend 订单
- 构建 ViperX customer tree
- 解析 semantic target
- 先导航到底盘停靠位
- 再执行 scan / detect / pick / place

## 5. 发单方式

完整系统里，推荐优先用 backend / UI 发单。

如果只是测试链路，也可以保留一个手工发单的后备入口。

### 方式 A: 从 UI / backend 发单

这是最符合真实系统的方式。

链路是：

- 用户页面或后端 API 提交订单
- backend 写入订单
- `bt_executor_viperX` 轮询 `http://localhost:3000/api/order/latest`
- BT 自动接单执行

### 方式 B: 手工调用 `/order/new`

如果 UI 还没准备好，但你仍然想保持完整 ROS 系统都已经起来，可以手工发一个单：

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
ros2 service call /order/new robot_interfaces/srv/NewOrder "{order: {order_id: 1, role: customer, requester_id: test, items: [{product_id: '3', name: 'Apple', aisle: '0', rack: 0, shelf_level: 2, qty: 1, price: 0.0, stock: 1}]}}"
```

这仍然属于完整系统测试，因为：

- backend
- navigation
- MoveIt
- vision
- BT

都已经启动了。

## 6. 推荐测试商品名

为了减少名字不匹配的问题，优先使用 semantic map 里已经存在的商品名：

- `Green Tea`
- `Water`
- `Apple`
- `Orange`
- `Lemon`
- `Can`
- `Bag of Chips`

最稳的做法是：

- `product_id` 和 `name` 都填
- `name` 直接用上面这些值

## 7. 启动完成后的联调检查

### 检查 1: 机械臂 action server

```bash
ros2 action list | grep pick_viperx
```

应该看到：

- `/pick_viperx`

### 检查 2: 导航 action

```bash
ros2 action list | grep navigate_to_pose
```

应该看到：

- `/navigate_to_pose`

### 检查 3: semantic resolve service

```bash
ros2 service list | grep semantic_map
```

应该至少看到：

- `/semantic_map/resolve_target`

### 检查 4: 视觉检测

```bash
ros2 topic hz /detections_json
ros2 topic echo /detections_json --once
```

### 检查 5: BT 服务

```bash
ros2 service list | grep order
```

应该看到：

- `/order/new`

### 检查 6: backend

浏览器打开：

- `http://localhost:3000`

至少要确认 backend 进程已经正常监听。

### 检查 7: rosbridge 远端地图桥接

如果你起了可选的 bridge 终端，可以检查：

```bash
ros2 node list | grep rosbridge
```

然后在 embedded app 里确认连接地址是：

- `ws://192.168.8.249:9090`

## 8. 完整 pickup 的预期行为

当订单真正进入系统后，当前主流程应该表现为：

1. backend 有单
2. BT 轮询取单
3. semantic map 解析出当前商品对应的 `nav_goal`
4. 底盘导航到目标停车位
5. 机械臂到 scan pose
6. `camera_vision` 检测目标
7. BT 生成 `pregrasp / grasp / lift` 位姿
8. `/pick_viperx` 执行抓取
9. 机械臂放到 basket pose
10. 库存更新
11. 所有 item 完成后流程结束

## 9. 当前最容易出问题的地方

### 问题 1: 同时起了 BT 和旧 `vision_auto_pick.py`

结果：

- 两套上层同时向 `/pick_viperx` 发动作

规避方法：

- 只起 `vx300_moveit.launch.py`
- 不起 `vx300_auto_pick.launch.py`

### 问题 2: 底盘和机械臂串口搞反

完整系统测试时请固定写成：

- 导航：`--serial-port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`
- 机械臂：`motor_port:=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT88YTG6-if00-port0`

### 问题 3: 商品名不一致

如果下面三者不一致：

- order 里的 `name`
- semantic map 里的 `product_names`
- vision 里发布的类别名

就可能出现：

- semantic resolve 找不到
- 或 vision 能看到但 BT 不认

### 问题 4: 只看 UI，不看 ROS 节点是否真的起来

当前系统是多进程拼出来的。

所以 UI 能打开，不代表：

- Nav2 已经起来
- `/pick_viperx` 已经起来
- `camera_vision` 已经在发检测
- BT 已经能接单

必须按第 7 节逐项检查。

### 问题 5: 远端 SLAM 页面连不上

常见原因：

- `rosbridge_server` 没有启动
- embedded app 里还是错误的 hostname / IP
- 网络或防火墙没有放行 `9090` 端口

## 10. 给测试者的最终建议

如果你的目标是“完整系统能不能跑 pickup”，那就不要再区分最小模式了，直接按这份文档给出的终端顺序起。

当前标准 full-system 启动顺序就是：

**backend -> UI -> navigation -> 可选 rosbridge -> MoveIt -> camera_vision -> bt_executor_viperX**

再往后，问题就不该再归因于“没起对入口”，而应该开始检查：

- 订单是否写进 backend
- semantic map 是否解析成功
- Nav2 是否真的到位
- `/detections_json` 是否有目标
- `/pick_viperx` 是否执行成功

## 11. Patch Log 2026-04-08

### 11.1 Nav2 停靠距离调参

本轮 patch 的目标是：

- 验证 pickup 最后停靠阶段是否主要被 `local_costmap` 的 inflation 往货架外推
- 只改局部 costmap 的 inflation，不同时放松 global route planning

应用位置：

- `workspace/src/robot_navigation/config/nav2_params_smac_mppi_omni.yaml`

本轮范围：

- 只改 `local_costmap.inflation_layer`
- 保持 `global_costmap` 不变
- 保持 `robot_radius` 不变
- 保持 semantic `nav_pose` 不变

改前改后：

| 参数 | 改前 | 改后 |
| --- | --- | --- |
| `local_costmap.inflation_layer.inflation_radius` | `0.55` | `0.42` |
| `local_costmap.inflation_layer.cost_scaling_factor` | `3.0` | `6.0` |
| `global_costmap.inflation_layer.inflation_radius` | `0.55` | `0.55` |
| `global_costmap.inflation_layer.cost_scaling_factor` | `3.0` | `3.0` |

原因：

- 当前问题主要出在目标货架附近的最终停靠，而不是全局路径规划过远
- `local_costmap` 更直接影响 MPPI 最后 1 米左右的靠近与停车行为
- 把 local `inflation_radius` 调小，可以缩短货架外侧的 inflated keep-away 带
- 把 local `cost_scaling_factor` 调大，可以让 inflation 代价离开障碍边界后下降得更快，减少“还没到目标就被挤停”的情况
- `global_costmap` 本轮故意不动，是为了隔离变量，避免先引入整条路径规划的副作用

测试注意：

- 这次 patch 改的是 `robot_navigation` 的源码配置
- 如果你要让默认 localization stack 真正加载到这版参数，测试前需要重新构建 `robot_navigation`
- 如果没重建，launch 读到的可能还是旧的安装配置

### 11.2 Customer pickup 整单失败后的回 home 恢复

应用位置：

- `workspace/src/robot_task_manager/robot_task_manager/trees/customer_viperx.py`
- `workspace/src/robot_task_manager/robot_task_manager/bt_executor_viperx.py`
- `workspace/src/robot_task_manager/robot_task_manager/blackboard.py`

本轮目标：

- 当 customer pickup 在整单完成前失败时，底盘仍然尝试回到 `home_goal`
- 回 home 允许重试几次，但整单结果仍保持 `FAILED`
- 不把“订单主体失败后的恢复”与“成功收尾后的回 home”混为一类

行为变化：

- 改前：只在整单成功走完后执行 `SetHome -> MaybeNavigateToGoalPose`
- 改后：整单过程中任一关键步骤失败后，会进入 failure recovery 分支，执行 `SetHome -> Retry(MaybeNavigateToGoalPose)`，然后显式保持树结果为 `FAILURE`
- 同时，成功路径下的回 home 也改成了带重试的导航

新增参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `return_home_retry_attempts` | `3` | customer pickup 在成功收尾或失败恢复时，底盘回 `home_goal` 的导航重试次数 |

实现说明：

- blackboard 新增 `customer_order_items_completed`，用于区分“订单主体已经完成”还是“订单主体中途失败”
- 只有在 `repeat_each_item` 尚未完成时，才会触发失败后的回 home 恢复分支
- 如果恢复导航最终成功，树仍然返回 `FAILURE`，因此 backend 仍会把这单记为失败单，而不是成功单
- 如果成功路径下的回 home 在重试后仍失败，整单仍会按失败处理

触发条件：

- 成功路径：当 customer pickup 的 `repeat_each_item` 全部成功完成后，会执行一次带重试的回 `home_goal`
- 失败路径：只有 customer pickup 在整单完成前失败时，才会触发 `ReturnHomeOnFailure`
- 更准确地说，不是“某一步刚出问题就立刻回家”，而是该步骤最终向上返回 `FAILURE` 后，树才会切去回 home 恢复分支

当前会触发失败后回 home 的典型情况：

- `SetCurrentItem` 失败，例如订单状态异常或索引已经越界
- 到货架的 Nav2 导航最终失败
- 视觉搜索阶段最终失败，例如 center / left / right 扫描后仍未找到稳定目标，或等待超过 `search_timeout_sec`
- `PrepareDetectedPickPoses` 失败，例如检测点超出机械臂工作空间
- 抓取动作在 `RetryGrabFromPregrasp` 用完后仍失败
- 放篮流程失败，例如 basket slot 不可用或中间某个机械臂动作失败
- item 完成后的机械臂 `home_pose` 返回失败
- `ChangeInventory` 失败，例如 backend 库存接口请求失败

需要注意的边界：

- `ResolveCurrentItemSemanticTargetViperX` 默认会 fallback 到 legacy 坐标，所以 semantic resolve 出问题不一定会触发回 home；只有它最终真的返回 `FAILURE` 才会触发
- 检测阶段不是“看一眼没看到就失败”，而是会先经历等待新鲜 detection、稳定性计数，以及 center / left / right 三段扫描
- 这套自动回 home 逻辑当前只写在 customer ViperX pickup 树里，不适用于 employee/restock 树

测试注意：

- 这次 patch 只改了 customer ViperX 树，没有改 employee/restock 树
- 当前环境没有可用的 `colcon`，这里只做了 Python 语法级校验，没有做 ROS 运行验证
