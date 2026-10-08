<img width="400" height="225" alt="video_20261004_161734" src="https://github.com/user-attachments/assets/208acbea-bf12-454e-8abf-3b4c97e4cda3" /># 5-DOF Robotic Arm — RK3568 + STM32 异构控制系统（视觉模块暂未完成）

![Uploading video_20261004_161734.gif…]()

基于 **RK3568（ROS 2 上位机）+ STM32F407（FreeRTOS 下位机）** 双处理器异构架构的五自由度机械臂 + 夹爪控制系统。
两者经 **CAN 2.0B（250 kbps）** 互联，实现从 URDF 建模、运动学解算到实时舵机控制与可视化的完整链路。

```
运动学解算 (ROS 2)  ──CAN──▶  实时调度 (FreeRTOS)  ──UART──▶  6× 总线舵机
        ▲                                                          │
        └───────────────── 状态回传 ◀──────────────────────────────┘
```

---

## 系统架构

```
┌────────────────────── RK3568（Linux / ROS 2 Jazzy）──────────────────────┐
│                                                                          │
│   arm_server  (C++17 / rclcpp)                                           │
│     ├─ 发布  /joint_states     舵机位置 ──▶ 关节角度                     │
│     └─ 订阅  /joint_commands   关节角度 ──▶ 舵机位置                     │
│                                                                          │
│   arm_can  (静态库，不含 ROS 依赖)                                       │
│     ├─ SocketCAN 服务端：socket / bind / CAN_RAW_FILTER                  │
│     ├─ rx 线程        ← poll 阻塞收 0x200                                │
│     ├─ poll 定时器    ← 50 ms 轮询下一个舵机                             │
│     └─ 请求-应答门控   ← awaiting_ 标志，防止灌爆下位机队列              │
│                                                                          │
└──────────────────────────────────┬───────────────────────────────────────┘
                                   │  CAN 2.0B @ 250 kbps
                                   │  0x104 下行  /  0x200 上行
┌──────────────────────────────────┴───────────────────────────────────────┐
│                        STM32F407VET6（FreeRTOS / CMSIS-RTOS2）           │
│                                                                          │
│   CAN_Rx_Task  (Realtime3, P=51)  ──▶  命令队列（深 16）                 │
│   UART_Task    (Realtime1, P=49)  ◀──  出队执行                          │
│   CAN_Tx_Task  (High7,     P=47)  ──▶  状态快照回传                      │
│   led_running  (Low,       P=8)                                          │
│                                                                          │
│   USART2  半双工 @115200  ──▶  ZX20S ×1 (ID0)  +  ZX361S ×5 (ID1~5)      │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘
```

### 分工原则

| 处理器 | 职责 | 为什么 |
|---|---|---|
| **STM32F407** | 串口舵机时序、CAN 收发、任务调度 | 需要**确定性**（μs 级响应）；Linux 用户态调度抖动会让舵机丢帧 |
| **RK3568** | ROS 2 节点、运动学解算、可视化、网络 | 需要算力与生态，对实时性要求低 |

---

## 硬件清单

| 部件 | 型号 | 数量 | 说明 |
|---|---|---|---|
| 上位机 | **RK3568**（KickPi K1Mini） | 1 | aarch64 Linux，跑 ROS 2 Jazzy |
| 下位机 | **STM32F407VET6** | 1 | Cortex-M4 @168 MHz，FreeRTOS |
| 总线舵机 | **ZX20S** | 1 | 舵机 ID **0** → `joint1`（底座旋转） |
| 总线舵机 | **ZX361S** | 5 | 舵机 ID **1~5** → `joint2`~`joint5` + `gripper` |
| 总线 | CAN 2.0B | — | 250 kbps，`triple-sampling on` |

> ⚠️ **两种舵机型号的协议不完全兼容** —— ZX20S 与 ZX361S 的**温度单位不同、回复长度也不同**
> （11 字节 vs 15 字节），详见 [舵机串口协议](#舵机串口协议usart2-半双工-115200)。
> 这是固件放弃「固定长度读取」、改用「帧边界字符定界」的直接原因。

### 舵机 ↔ 关节映射

| 舵机 ID | 型号 | 关节 | 运动轴 | 行程 |
|---|---|---|---|---|
| 0 | **ZX20S** | `joint1` | 绕 Z（底座旋转） | 500 ~ 2500 |
| 1 | ZX361S | `joint2` | 绕 Y（大臂俯仰） | 500 ~ 2500 |
| 2 | ZX361S | `joint3` | 绕 Y（小臂俯仰） | 500 ~ 2500 |
| 3 | ZX361S | `joint4` | 绕 Y（腕部俯仰） | 500 ~ 2500 |
| 4 | ZX361S | `joint5` | 绕 Z（腕部自转） | 500 ~ 2500 |
| 5 | ZX361S | `gripper` | 夹爪开合 | 0 ~ 2000 |

> 舵机行程与 URDF 角度范围的对应关系配置在 `config/arm_params.yaml` 里，
> 详见 [配置说明](#配置说明)。

---

## 目录结构

```
.
├── CMakeLists.txt              # ament_cmake 包定义
├── package.xml                 # ROS 2 包描述
├── main.cpp                    # can_test：不依赖 ROS 的 CAN 调试工具
│
├── config/
│   └── arm_params.yaml         # ★ 舵机↔关节映射、方向矩阵、轮询周期
├── launch/
│   └── arm_server.launch.py    # 加载参数并启动 arm_server
│
├── include/  src/              # arm_can 库 + arm_server 节点
│   ├── can_init.{h,cpp}        #   SocketCAN 封装
│   ├── Servo_ctrl.{h,cpp}      #   舵机协议层：组帧 / 解析 / 状态缓存
│   ├── myqueue.h               #   单生产者单消费者环形缓冲
│   ├── app.cpp                 #   早期线程示例（未参与节点）
│   └── arm_server_node.cpp     #   ★ ROS 2 节点主体
│
├── can_server/                 # 上述源码的同步快照（由 push.sh 生成）
│
├── firmware/stm32/             # STM32F407 固件（CubeMX + CMake）
│   ├── Core/Src/robot_arm.c    #   ★ 舵机驱动 + DMA 收帧 + CAN 协议
│   ├── Core/Src/freertos.c     #   任务创建
│   ├── Core/Src/usart.c        #   USART2 半双工 + DMA
│   ├── MYWARE/aps6404/         #   PSRAM 驱动（预留）
│   └── robot_arm.ioc           #   CubeMX 工程
│
└── patches/
    └── rockchip_canfd-fix-rx-stats.patch   # 内核 CAN 驱动修复（见下）
```

---

## 通信协议

### CAN 帧格式

**下行 `0x104`** — 上位机 → STM32

| 字节 | 含义 |
|---|---|
| `data[0]` | 舵机 ID（0~5） |
| `data[1]` | 动作码 |
| `data[2..3]` | 目标位置（小端 uint16，500~2500） |
| `data[4..5]` | 到位时间 ms（小端 uint16） |
| `data[6..7]` | 保留，置 0 |

**动作码**：

| 值 | 宏 | 含义 |
|---|---|---|
| `1` | `ASKFORSETPOS` | 设置位置 |
| `2` | `ASKFORSTATUS` | 查询状态 |
| `3` | `ASKFORALL` | 设置位置并查询 |

**上行 `0x200`** — STM32 → 上位机

| 字节 | 含义 |
|---|---|
| `data[0]` | 舵机 ID |
| `data[1..2]` | 当前位置（小端 uint16） |
| `data[3]` | 温度 |
| `data[4]` | 电压 |

> 接收侧要求 `can_dlc >= 5`，且 `can_id & CAN_SFF_MASK == 0x200`。

**其他保留 ID**：`0x105` 双机同步、`0x201` 温度电压、`0x202` 错误上报。

### 舵机串口协议（USART2 半双工 @115200）

| 发送 | 含义 |
|---|---|
| `#%03dP%04dT%04d!` | 设置 ID 舵机在 T ms 内转到位置 P |
| `#%03dPRAD!` | 读角度 |
| `#%03dPRTE!` | 读温度 |
| `#%03dPRTV!` | 读电压 |

| 回复示例 | 舵机型号 | 长度 | 解析 |
|---|---|---|---|
| `#001T1845-08.1!` | **ZX361S**（ID 1~5） | 15 B | 温度 = `1845` → **℃×100**；电压在 `-` 之后 |
| `#000T25V05!` | **ZX20S**（ID 0） | 11 B | 温度 = `25` → **℃**；电压在 `V` 之后 |

> ⚠️ **两种型号的回复格式不兼容**：ZX20S 的温度单位是 **℃**、ZX361S 是 **℃×100**；
> 回复长度也不同（11 B vs 15 B）。
> 因此固件**不能按固定长度读取**，而是采用**帧边界字符 `#`…`!` 定界**，
> 并在解析层把 ZX20S 的温度乘 100 归一化，使上层拿到的单位一致。

---

## 快速开始

### 1. RK3568 侧

```bash
# ① 配置 CAN（bitrate 每次上电都要设）
sudo ip link set can0 type can bitrate 250000 triple-sampling on
sudo ip link set can0 up
# 等价：sudo ./push.sh 里用的也是这两条

# ② 编译工作空间
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone git@github.com:gggttt123w/ros2-5dof-arm.git
cd ~/ros2_ws && colcon build --packages-select CanServer
source install/setup.bash

# ③ 启动节点
ros2 launch CanServer arm_server.launch.py
```

**验证**：

```bash
ros2 topic hz /joint_states        # 期望 ~50 Hz
ros2 topic echo /joint_states --once
```

**不依赖 ROS 的调试工具**：

```bash
./build/CanServer/can_test         # 直接读写 CAN，用来排除 ROS 层问题
```

### 2. STM32 侧

```bash
cd firmware/stm32
cmake --preset Debug
cmake --build build/Debug
# 用 STM32CubeProgrammer / openocd 烧录 build/Debug/*.elf
```

### 3. 上位机发指令（可选）

```bash
# 让 joint1 转到 0.3 rad
ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState \
  "{name: ['joint1'], position: [0.3]}"
```

---

## 配置说明

`config/arm_params.yaml`：

```yaml
arm_server:
  ros__parameters:
    can_interface: "can0"
    poll_period_ms: 50                    # ← 轮询周期，见下

    joint_names: ["joint1","joint2","joint3","joint4","joint5","gripper"]
    servo_ids:   [0, 1, 2, 3, 4, 5]       # 关节下标 → 舵机 ID 的映射

    servo_min:   [500, 500, 500, 500, 500, 0]      # 舵机行程
    servo_max:   [2500,2500,2500,2500,2500,2000]
    angle_min:   [-1.5708,-1.5708,-1.5708,-1.5708,-3.1416, 0.0]   # URDF 角度范围
    angle_max:   [ 1.5708, 1.5708, 1.5708, 1.5708, 3.1416, 0.5]

    dir: [1.0, 1.0, -1.0, -1.0, 1.0, -1.0]         # ★ 方向矩阵
```

### `poll_period_ms` 怎么选

上位机**轮流**查询 6 个舵机，所以「全轴刷新时间 = `poll_period_ms` × 6」：

| 值 | 全轴刷新 | STM32 占用 | 备注 |
|---|---|---|---|
| 200 | 1200 ms | ~3% | 原值，RViz 明显滞后 |
| **50** | **300 ms** | ~12% | **当前值** |
| 35 | 210 ms | ~17% | 接近 STM32 单条命令耗时上限 |

再快就要改固件（加「一次查询全部舵机」的命令，省 5 次 CAN 往返）。

### `dir` 怎么标定的

用**单关节正弦激励探针**实测：只让一个关节在 ±30° 内摆动（用 `sin` 保证第一段一定是正方向），
观察实机转向，逐个比对 URDF 轴向得出。

| 关节 | URDF 轴 | 实测 + 方向 | `dir` |
|---|---|---|---|
| joint1 | `0 0 1` | 俯视逆时针 | `+1` |
| joint2 | `0 1 0` | 往正面 | `+1` |
| joint3 | `0 1 0` | 往后面 | `-1` |
| joint4 | `0 1 0` | 往后面 | `-1` |
| joint5 | `0 0 1` | 俯视逆时针 | `+1` |
| gripper | `0 1 0` | 闭合 | `-1` |

> **注意实现细节**：方向反转用的是**归一化量反射** `t → 1-t`，**不是角度取负**。
> 因为夹爪的角度范围是 `0 ~ 0.5`（不对称），取负会变成 `-0.5 ~ 0` 直接越界。
> 反射对任意范围都成立，且保证正反变换互逆。

---

## 关键技术点

### ① 中断优先级抢占导致 DMA 帧撕裂 ⭐

**现象**：系统随机卡死 —— 机械臂无响应、状态 LED 停止闪烁，**但停止发送命令后自行恢复**。

**排查过程**（跨 4 层）：

```
现象      LED 停闪 + 无响应，停止发送后恢复
第 1 层   固件：原实现用 HAL_UART_Receive(..., 1000) 阻塞轮询
          → 改成 DMA + 空闲中断 + 信号量，未根治
第 2 层   CAN 抓包：「20 条命令无一条回复，随后一次性涌出 20 条」
          → 典型的队列积压后突发（队列深 16）
第 3 层   Linux：确认 socket 收发 / 过滤器 / bind 正常，排除用户态
第 4 层   回到固件，读 stm32f4xx_hal_uart.c 的空闲中断分支
```

**根因**：

```c
uint16_t nb_remaining = __HAL_DMA_GET_COUNTER(huart->hdmarx);
if ((nb_remaining > 0) && (nb_remaining < huart->RxXferSize)) {
    ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);   /* ← 停掉 DMA */
    HAL_UARTEx_RxEventCallback(huart, ...);
}
```

HAL 只要发现空闲中断到来时 DMA **已经搬了哪怕 1 个字节**，就立即终止 DMA。

而时序是：发完命令后总线本来就空闲 → 空闲中断立刻触发；**但 CAN 接收中断（NVIC 优先级 51）会抢占 USART2 中断（优先级 5）**，
把这个空闲中断推迟到**舵机第一个字节到达之后**才处理 —— 此时 `nb_remaining = 31 < 32`，条件成立，
**HAL 提前终止 DMA，一帧被劈成两半**，数据错位，后续解析全失败。

**解决方案**：彻底放弃空闲中断判帧，改为**大缓冲 DMA + 在已收数据中搜索帧边界 `#`…`!`**。
不依赖任何中断时序，同时兼容两种长度不同的舵机回复。

**结果**：6 轴查询稳定、单次往返 12 ms、零丢帧，其他任务恢复正常调度。

---

### ② 请求-应答门控

上位机原本**定时**发查询、不关心是否收到回复。但 STM32 的命令队列只有 16 深、串口任务处理一条约 5 ms ——
一旦某条命令超时 1 s，这 1 s 内积累的命令会**灌满队列**，让串口任务长时间占用 CPU（优先级 49 高于 CAN 发送 47 和 LED 8），
最终导致**低优先级任务被饿死**。

```cpp
void on_poll() {
    if (awaiting_.load()) {                  // 上一条还没回复
        if (++stall_ < 30) return;           // 等 30 个轮询周期
        stall_ = 0;
        awaiting_.store(false);              // 超时自愈，强制重试
        RCLCPP_WARN(get_logger(), "STM32 无响应，重试");
    }
    servo_->Statue_get(next_id_);
    next_id_ = (next_id_ + 1) % servo_ids_.size();
    awaiting_.store(true);                   // 关门
}
```

`rx_loop` 收到回复后清 `awaiting_`（开门），下一次定时器触发才发下一条。

---

### ③ 单生产者-单消费者环形缓冲

CAN 接收缓冲是**无锁**的：

```c
/* head 只被生产者写，tail 只被消费者写 */
/* uint8_t 在 Cortex-M4 上单指令读写，天然原子 */
```

代价是必须严格保证**只有一个写者、一个读者**。

---

### ④ 运动学：位置与姿态解耦

5 自由度中，**位置链只有 3 个关节**：

```
joint1 ┐
joint2 ├─ 决定「末端在哪」  → 位置逆解（底座 yaw + 平面 2R）
joint3 ┘

joint4 ┐
joint5 ── 决定「末端朝哪」  → 腕部姿态，不参与位置解算
```

因为 `joint4` 绕**自身轴心**旋转（轴心位置不变）、`joint5` 绕**工具轴**自转（连朝向都不变）。

**逆解**：

```
θ1 = atan2(y, x)                          底座方位角
d  = √(r² + h²)                           r = √(x²+y²),  h = z − Z0
c3 = (d² − L1² − L2²) / (2·L1·L2)         余弦定理
θ3 = ±acos(c3)                            两解：肘上 / 肘下
k1 = L1 + L2·cos(θ3)
k2 = L2·sin(θ3)
θ2 = atan2(k1·r − k2·h,  k2·r + k1·h)
```

两个分支取**离上一次解更近**的那个，保证轨迹连续。

**奇异位形处理**：全伸展时（`θ2 = θ3 = 0`）处于工作空间边界 —— 实测末端位移 **12 mm**
会导致肘关节摆动 **48°**。因此轨迹的**圆心不放在奇点上**，整体往可达域内侧偏。

---

### ⑤ Rockchip CAN 驱动的空指针修复

`patches/rockchip_canfd-fix-rx-stats.patch` 修复了 `drivers/net/can/rockchip/rockchip_canfd.c` 中
错误处理路径的问题：

```c
struct can_frame *cf;              /* 原代码：未初始化 */
...
skb = alloc_can_err_skb(ndev, &cf);   /* 这个函数可能返回 NULL */
...
cf->can_id |= CAN_ERR_BUSOFF;      /* ← skb 为 NULL 时解引用空指针 */
```

**修复**：初始化 `cf = NULL`，并在所有 `cf->` 访问前加 `if (likely(skb))` 守卫。

> 影响：CAN 总线进入 error-passive / bus-off 状态时，内核可能因空指针解引用而崩溃 ——
> 对无人值守的机械臂控制器是致命的。

---

## License

[MIT](LICENSE) © 2026 yyyyjml23333
