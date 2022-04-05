# 分布式通信模块
该模块实现无人机系统与地面站之间的分布式通信。
## 无人机端
该模块在无人机端实现对地面站指令的接收、对无人机系统中其他个体信息的接收和无人机自身信息的发送。
- 具体用法
   在终端运行/sysu_playDrone_ws/src/distributed_communication/launch/communication_UAV.launch文件，具体命令如下：
   `roslaunch distributed_communication communication_UAV.launch`
- 参数
   - index：无人机编号
   - num：无人机系统中个体的数量
   - rate：无人机发送自身信息的频率
- 输入
   无人机端通信模块的输入为自身信息，包括位姿、GPS信号和飞行状态等，并将其发送给系统中其他个体和地面站。
- 输出
   - 地面站控制指令：话题名为**key**，消息类型为**std_msgs::String**
   - 系统中其他个体的信息：话题名为**UAVs/status**，消息类型为g**round_control_station::StatusArrayNew**，该消息类型的详细信息请参考msg文件/sysu_playDrone_ws/src/ground_control_station/msg/StatusArrayNew.msg

## 地面站端
该模块在地面站端实现对控制指令的发送和对无人机系统中个体信息的接收。
- 具体用法
   在终端运行/sysu_playDrone_ws/src/distributed_communication/launch/communication_GCS.launch文件，具体命令如下：
   `roslaunch distributed_communication communication_GCS.launch`
- 参数
   - index：地面站编号，默认为0
   - num：无人机系统中个体的数量
- 输入
   地面站端通信模块的输入为控制指令，并将其发送给系统中所有的个体。
- 输出
   - 系统中其他个体的信息：话题名为**UAVs/status**，消息类型为g**round_control_station::StatusArrayNew**，该消息类型的详细信息请参考msg文件/sysu_playDrone_ws/src/ground_control_station/msg/StatusArrayNew.msg