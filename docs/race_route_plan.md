# 比赛路线执行计划

本文档按赛规和 3D 地图整理完整路线。路线目标是先稳定完成低风险基础分，再逐步增加台阶、抓放、检测和放置得分。

## 视觉触发约束

未确认赛场允许前，不规划额外自贴路线标志。视觉触发只允许使用：

- 抓取平台 1/2 识别标志。
- 检测平台警示标志。
- 物资标签。
- 台阶规则允许的 10cm x 10cm ArucoTag。
- 平台、障碍、台阶自身外观。
- 黑色导引线。

## 总路线

```text
启停区
-> 起点障碍
-> 避障区
-> 台阶
-> 抓取平台
-> 中转平台
-> 检测点
-> 放置平台
-> 终点障碍
-> 启停区
```

## 阶段计划

| 阶段名称 | 主负责人 | 触发条件 | 进入动作 | 退出条件 | 失败兜底 | 对应得分 |
| --- | --- | --- | --- | --- | --- | --- |
| START_ZONE_READY | 成员 3 | 裁判允许开始，Go2 在蓝色启停区，节点启动完成 | 发布 `/mission/run` 或 auto start；保持 `/mission/stop` 直到确认准备好 | 状态机进入 START_JUMP | 任何节点缺失则不启动，保持 stop | 准备阶段，无直接得分 |
| START_JUMP | 成员 2 | 状态机开始，起点障碍在前方 | 停止巡线，调用 `/locomotion/execute_motion` 的 `start_jump` | Go2 跳过障碍并稳定落地 | 调用 stop；本轮保守放弃该分或人工干预 | 起点障碍 10 |
| LINE_TO_OBSTACLE_ENTRY | 成员 1 | 起点跳障完成，黑线重新进入视野 | 发布 `/mission/start`，按黑线巡线到避障区入口 | 入口外观/黑线几何变化/调试确认的入口条件成立 | 丢线则低速搜索，超时 stop | 为避障区准备 |
| STOP_BEFORE_OBSTACLE | 成员 3 | 到达避障区入口 | 发布 `/mission/stop`，等待 `/navigation/cmd_vel` 为零 | Go2 停稳，成员 2 可接管动作 | 未停稳则重复 stop；超时进入 ERROR_STOP | 为避障区准备 |
| ENTER_OBSTACLE_ZONE | 成员 2 | 巡线已停止，避障动作开始 | `/gait/control_lock=true`，低速进入避障区 | 进入第一段通道，姿态稳定 | `OBSTACLE_STOP`，退出动作序列 | 避障区域 20 的一部分 |
| PASS_OBSTACLE_ZONE | 成员 2 | 已进入避障区通道 | 执行 `OBSTACLE_FORWARD_SLOW`、`OBSTACLE_TURN_LEFT`、`OBSTACLE_TURN_RIGHT`、`OBSTACLE_SIDE_ADJUST` 组合 | 通过挡板区主体，接近出口 | 任一动作异常立即 `OBSTACLE_STOP`，允许状态机重试一次 | 避障区域 20，碰板一次扣 5 |
| EXIT_OBSTACLE_ZONE | 成员 2 | 到达避障区出口段 | 低速退出，最后发送 stop/hold | 出口黑线或出口外观进入成员 1 可识别范围 | stop 并等待人工确认是否重试 | 避障区域 20 的出口条件 |
| RESUME_LINE_FOLLOWING | 成员 1/3 | 成员 2 释放 `/gait/control_lock`，出口黑线恢复 | 成员 3 发布 `/mission/start`；成员 1 重新巡线 | `line_visible=true` 且误差稳定连续多帧 | 搜索超时则 stop，避免乱冲 | 为台阶阶段准备 |
| LINE_TO_STAIRS | 成员 1 | 避障出口恢复巡线 | 沿黑线走到台阶正前方 | 台阶外观或允许的 10cm x 10cm ArucoTag 触发停车 | 丢线搜索；识别失败则保守 stop | 为台阶得分准备 |
| STAIRS_UP_DOWN | 成员 2 | 台阶前停稳 | 停止巡线，调用 `stairs_up_down` | 上下台阶完成，落地并恢复站姿 | stop/hold；必要时只保 20 分方案 | 台阶 20/30 |
| LINE_TO_PICK_PLATFORM | 成员 1 | 台阶动作完成，黑线恢复 | 巡线到抓取平台前 | 抓取平台外观、物资标签或平台前黑线位置触发停车 | 低速搜索或 stop | 为抓取平台准备 |
| DETECT_PICK_SIGN | 成员 3 | 抓取平台前停稳，识别标志可见 | 读取 `/perception/sign_detections` 中 1/2 标志 | 记录 `place_platform_1` 或 `place_platform_2` | 若未识别，用配置默认平台并记录风险 | 放置平台 30 的前置条件 |
| PICK_START_ITEM | 成员 3 | 起始物资可抓取 | 调用 `/arm/execute_task` 的 `pick_start_item` | 起始物资离开抓取平台顶面 | 机械臂回安全位，允许重试一次 | 抓取平台 10 |
| LINE_TO_TRANSFER_PLATFORM | 成员 1 | 起始物资已抓起，机械臂回安全位 | 巡线到中转平台 | 中转平台外观/物资标签/黑线位置触发停车 | stop，等待状态机处理 | 为中转得分准备 |
| DROP_START_ITEM | 成员 3 | 中转平台前停稳 | 调用 `/arm/execute_task` 的 `drop_start_item` | 起始物资稳定在中转平台顶面 | 回安全位并允许重试一次 | 中转平台卸载 20 |
| PICK_FIELD_ITEM | 成员 3 | 起始物资已卸载，场地物资可见 | 调用 `/arm/execute_task` 的 `pick_field_item` | 场地物资离开中转平台顶面 | 回安全位并允许重试一次 | 中转平台抓取 10 |
| LINE_TO_CHECK_POINT | 成员 1 | 场地物资已抓起 | 巡线到检测点 | 检测平台外观或警示标志进入识别范围，停稳 | stop，必要时跳过检测但继续后续 | 为检测平台准备 |
| DETECT_WARNING_SIGN | 成员 3 | 检测点停稳，警示标志可见 | 读取警示标志 | 映射到 `stretch`、`wave` 或 `blink_front_light_3` | 识别失败用配置默认动作并记录风险 | 检测平台 20 的前置条件 |
| DO_WARNING_ACTION | 成员 2/3 | 警示动作已确定 | 调用 `/locomotion/execute_motion` 的警示动作 | 正确动作完成 | stop/hold，失败则放弃该分继续 | 检测平台 20 |
| LINE_TO_PLACE_PLATFORM | 成员 1/3 | 警示动作完成 | 根据 `place_platform_1/2` 选择路线，巡线到对应放置平台 | 对应平台外观或黑线位置触发停车 | 若路线错误，stop；默认平台只作为兜底 | 为放置平台准备 |
| PLACE_FIELD_ITEM | 成员 3 | 对应放置平台前停稳 | 调用 `/arm/execute_task` 的 `place_field_item`，target 为平台 1/2 | 场地物资稳定在对应平台顶面 | 回安全位并允许重试一次 | 放置平台 30 |
| LINE_TO_FINISH_JUMP | 成员 1 | 放置完成，机械臂安全 | 巡线到终点障碍前 | 终点障碍外观或黑线位置触发停车 | 丢线搜索，超时 stop | 为终点障碍准备 |
| FINISH_JUMP | 成员 2 | 终点障碍前停稳 | 停止巡线，调用 `finish_jump` | 跳过终点障碍并稳定落地 | stop/hold，必要时保守放弃该分 | 终点障碍 10 |
| RETURN_TO_START_ZONE | 成员 1/3 | 终点跳障完成，黑线恢复 | 巡线回启停区 | 启停区蓝色区域外观/终点黑线位置触发停车 | 低速搜索，超时 stop | 为终点停靠准备 |
| STOP_IN_START_ZONE | 成员 1/3 | 到达启停区 | 发布 `/mission/stop`，发布零速度，保持站姿 | 四个足端完全在蓝色启停区内 | 若位置偏差，执行低速微调后再 stop | 终点停靠 10 |
| DONE | 成员 3 | 全部阶段完成或保守结束 | 停止导航和固定动作，记录结果 | 任务结束 | 保持 stop，不再输出非零速度 | 完成比赛 |

## 避障区展开方案

避障区不依赖实时复杂规划，先按保守固定动作序列实现。

| 子阶段 | 负责人 | 触发条件 | 动作 | 退出条件 | 失败兜底 |
| --- | --- | --- | --- | --- | --- |
| LINE_TO_OBSTACLE_ENTRY | 成员 1 | 起点障碍后恢复巡线 | 巡线到入口 | 入口外观/黑线位置触发 | 丢线搜索，超时 stop |
| STOP_BEFORE_OBSTACLE | 成员 3 | 入口条件成立 | `/mission/stop` | 速度归零并停稳 | 重复 stop，超时 ERROR_STOP |
| ENTER_OBSTACLE_ZONE | 成员 2 | 巡线停止 | 低速直行进入 | 进入通道第一段 | `OBSTACLE_STOP` |
| PASS_OBSTACLE_ZONE | 成员 2 | 已进入 | 慢速前进、左右转、侧向微调组合 | 接近出口 | `OBSTACLE_STOP`，状态机最多重试一次 |
| EXIT_OBSTACLE_ZONE | 成员 2 | 接近出口 | 低速退出并 stop | 出口黑线/外观可见 | stop 并等待恢复巡线 |
| RESUME_LINE_FOLLOWING | 成员 1/3 | control_lock 释放 | `/mission/start`，重新巡线 | 连续稳定识别黑线 | 搜索超时 stop |

## 阶段触发优先级

1. 规则允许或场地已有视觉对象触发。
2. 黑线几何和 `LineTrack` 稳定性触发。
3. 固定动作完成状态触发。
4. 超时兜底触发 stop 或跳过低优先级得分点。

## 必写测试命令

```bash
rg -n "<<<<<<<|=======|>>>>>>>" src
python3 -m py_compile src/rk_navigation/rk_navigation/line_follower_node.py
python3 -m py_compile src/rk_mission/rk_mission/mission_state_machine_node.py
python3 -m py_compile src/rk_locomotion/rk_locomotion/gait_control_node.py
PYTHONPATH=src/rk_perception:$PYTHONPATH python3 -m pytest src/rk_perception/test/test_real_line_tracker_node.py
colcon build --symlink-install
```

