## 程序节点

### 地面站端

#### 1. 地面站

```bash
roslaunch   ground_control_station    ground_control_station.launch  <参数>
```
- 参数：<uavNumbers> 控制的无人数量， 默认值为 8 

- 无人机状态监控，可视化，生成无人机控制指令

#### 2. 地面站键盘控制		

```bash
rosrun  ground_control_station  keyboard_control.py 
```

- 群体&单体控制无人机



#### 3. openvpn		

```bash
sudo openvpn /etc/openvpn/client1.ovpn 
```

- 地面站与无人机的通信



#### 4.地面站 消息订阅&发布器	

```bash
roslaunch cloud_server_communication groundStation.launch 
```

- 订阅无人机状态消息，将地面站的控制指令以 zmq 发布器发送到无人机



### 无人机端

#### 5. 无人机 消息订阅&发布器	

```
roslaunch cloud_server_communication client.launch 
```

- 订阅地面站的指令，发送自身状态到地面站

#### 6. 指令解析器		

```
roslaunch ground_control_station command_parser.launch 
```

- 解析地面站发来的控制指令

#### 7. dji_sdk		

```
roslaunch dji_sdk sdk.launch
```

- N3 飞控的控制接口

#### 8. openvpn		

```
sudo  openvpn /etc/openvpn/uav1.ovpn 
```

- 无人机与地面站通信
