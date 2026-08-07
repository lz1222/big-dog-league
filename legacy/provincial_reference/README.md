# Provincial Competition Reference

`final_provincial_reference.py` 是省赛历史运行代码，仅用于：

- 行为审计；
- 实机参数追溯；
- 巡线、白色障碍和 FrontJump 逻辑对比；
- 后续回归分析。

严格禁止：

- 作为国赛正式程序运行；
- 被任何 ROS launch 启动；
- 加入 setup.py console_scripts；
- 被 package.xml 或 setup.py 安装；
- 直接发布 /navigation/cmd_vel；
- 绕过 command_mux；
- 成为正式 SportClient.Move 控制入口；
- 被当前 ROS2 节点 import；
- 在功能 PR 中顺手修改。

正式比赛实现始终以 src/ 下 ROS2 模块为准。

参考文件：

legacy/provincial_reference/final_provincial_reference.py

固定 SHA-256：

221ec9c5ae99dfe32982ba68f99d5d4ca0fc4cef2fb85abc6cf9845b48107adb

对参考文件的任何修改必须通过独立审计，并同步更新哈希和相关审计文档。
