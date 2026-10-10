# ROS 2 侧总结：arm_server 节点与全部例程代码

> 覆盖范围：RK3568 上的 `arm_server` 节点、我在 WSL 工作区生成的 `demo/` 例程、
> 以及板子 `~/Project/arm_demo/` 上的演示脚本。每个文件、每个函数都讲。

---

## 一、整体数据流

```
┌─ RK3568 ────────────────────────────────────────────────┐
│                                                          │
│  ┌──────────────┐   CAN 0x104   ┌────────┐              │
│  │  arm_server  │ ────────────► │ STM32  │              │
│  │  (C++ 节点)  │               │ 固件   │              │
│  │              │ ◄──────────── │        │              │
│  └──────┬───────┘   CAN 0x200   └───┬────┘              │
│         │                           │ UART 半双工        │
│    /joint_states                6 个总线舵机             │
│    /joint_commands                                       │
└─────────┬────────────────────────────────────────────────┘
          │
          │  SSH 管道（JSON 行）—— 绕开 WSL2 NAT 的 DDS 发现问题
          ▼
┌─ WSL2 开发机 ───────────────────────────────────────────┐
│                                                          │
│  bridge_rx.py ──► /joint_states ──► robot_state_publisher│
│                                          │               │
│                                          ▼               │
│                                    /tf + /robot_description
│                                          │               │
│                                          ▼               │
│                                       rviz2              │
└──────────────────────────────────────────────────────────┘
```

**两条独立的链路**：

| 链路 | 方向 | 用途 |
|---|---|---|
| CAN `0x104` / `0x200` | 双向 | 控制舵机 + 读回状态 |
| SSH + JSON | 单向（板子→开发机） | 把真实状态搬给 RViz 看 |

---

## 二、ROS 2 节点 `arm_server`

### 2.1 文件构成

板子上 `~/Project/CanServer/`：

| 文件 | 行数 | 职责 | 是否依赖 ROS |
|---|---|---|---|
| `src/arm_server_node.cpp` | 174 | **ROS 2 节点**：参数、话题、定时器、rx 线程 | ✅ 是 |
| `src/can_init.cpp` | 93 | SocketCAN 封装：建 socket / bind / 过滤器 / 读 / 写 | ❌ 纯 Linux |
| `include/can_init.h` | 51 | 上面的头 + CAN ID 宏 | ❌ |
| `src/Servo_ctrl.cpp` | 110 | 舵机协议层：组帧、解析 `0x200`、缓存状态 | ❌ |
| `include/Servo_ctrl.h` | 42 | 上面的头 + `ServoStatus` 结构体 | ❌ |
| `config/arm_params.yaml` | 10 | 标定参数（角度↔舵机映射、方向、轮询周期） | — |
| `launch/arm_server.launch.py` | 10 | 把 YAML 喂给节点 | — |
| `src/app.cpp` | 64 | 早期的线程示例，**和节点无关** | ❌ |
| `src/myqueue.cpp` | 1 | 空文件（`include/myqueue.h` 里有实现） | ❌ |

**设计原则：`arm_can` 库完全不含 ROS**，所以 `can_test` 那个独立可执行文件照样能编。
只有 `arm_server_node.cpp` 把 ROS 和 CAN 接起来。

### 2.2 类的成员

```cpp
class ArmServer : public rclcpp::Node {
    // ---- 参数 ----
    std::string can_iface_;                       // "can0"
    int poll_ms_;                                 // 50
    std::vector<std::string> joint_names_;        // joint1..joint5, gripper
    std::vector<int64_t>     servo_ids_;          // 0..5
    std::vector<double>      servo_min_, servo_max_;    // 舵机行程
    std::vector<double>      angle_min_, angle_max_;    // URDF 角度范围
    std::vector<double>      dir_;                // 每个关节的转动方向 ±1

    // ---- 通信 ----
    std::unique_ptr<Cansocket_object> can_;       // socket 封装
    std::unique_ptr<ServoCtrl_object> servo_;     // 协议层

    // ---- ROS 接口 ----
    rclcpp::Publisher<JointState>::SharedPtr    pub_;    // /joint_states
    rclcpp::Subscription<JointState>::SharedPtr sub_;    // /joint_commands
    rclcpp::TimerBase::SharedPtr poll_timer_, pub_timer_;

    // ---- 并发 ----
    std::thread       rx_thread_;        // 收 CAN 的独立线程
    std::atomic<bool> running_{true};
    uint8_t           next_id_ = 0;      // 轮询到哪个舵机
    std::atomic<bool> awaiting_{false};  // 有一条查询在等回复
    int               stall_ = 0;        // 连续几轮没回复
};
```

### 2.3 三股执行流

这是整个节点最需要理解的地方 —— **有三件事在并发跑**：

| 执行流 | 谁在跑 | 周期 | 干什么 |
|---|---|---|---|
| **① 轮询定时器** | `rclcpp` executor 线程 | `poll_ms_` = 50ms | 发一条 `Statue_get`，问下一个舵机 |
| **② 发布定时器** | 同一个 executor 线程 | 20ms | 把缓存的 `Info[]` 打包成 `/joint_states` |
| **③ rx 线程** | 自己 `std::thread` | 阻塞在 `Info_wait(100)` | 收 `0x200`，解析后写进 `Info[]` |

**为什么 rx 要单独开线程**：`Can_Read` 里的 `poll()` 是**阻塞**的，最长 100ms。
如果放在 executor 线程里，②的 20ms 发布定时器就会被卡住，`/joint_states` 会一顿一顿。

### 2.4 逐函数讲解

#### 构造函数 `ArmServer()`

```cpp
// ① 声明所有参数（带默认值）
can_iface_   = declare_parameter<std::string>("can_interface", "can0");
poll_ms_     = declare_parameter<int>("poll_period_ms", 50);
...
dir_ = declare_parameter<std::vector<double>>("dir", {1,1,-1,-1,1,-1});

// ② 建 CAN socket，只收 0x200
std::vector<struct can_filter> filters = {{0x200, CAN_SFF_MASK}};
can_ = std::make_unique<Cansocket_object>(addr, filters);
if (!can_->Can_Socket_Init(can_iface_)) { ... throw ... }

// ③ 建 ROS 话题和定时器
pub_ = create_publisher<JointState>("/joint_states", 10);
sub_ = create_subscription<JointState>("/joint_commands", 10, on_command);
poll_timer_ = create_wall_timer(50ms, on_poll);
pub_timer_  = create_wall_timer(20ms, on_publish);

// ④ 起 rx 线程
rx_thread_ = std::thread(&ArmServer::rx_loop, this);
```

**顺序有讲究**：参数必须在定时器之前声明完，因为 `on_publish` 一上来就要用 `dir_`。

#### `rx_loop()` —— 收数据

```cpp
void rx_loop() {
    while (running_) {
        if (servo_->Info_wait(100)) {   // 返回 true = 真收到了 0x200 并解析成功
            awaiting_.store(false);      // 清掉"等待中"，放开下一轮轮询
            stall_ = 0;                  // 清掉连续失败计数
        }
    }
}
```

`Info_wait` 内部做四步（在 `Servo_ctrl.cpp`）：

```cpp
bool ServoCtrl_object::Info_wait(uint16_t time_ms){
    struct can_frame frame;
    if(can.Can_Read(frame,time_ms) == false){ ...; return false; }   // ① poll 100ms
    if((frame.can_id & CAN_SFF_MASK) != GETANGLE){ ... }             // ② 帧 ID 必须是 0x200
    if(frame.can_dlc < 5){ ... }                                     // ③ 至少 5 字节
    uint8_t id = frame.data[0];
    if(id >= SERVONUMBER){ ... }                                     // ④ id 合法
    ServoStatus tmp{};
    tmp.position    = frame.data[1] | (frame.data[2] << 8);          // 小端
    tmp.temperature = frame.data[3];
    tmp.volt        = frame.data[4];
    { std::lock_guard<std::mutex> lock(mtx_); Info[id] = tmp; }      // 加锁写缓存
    return true;
}
```

**`mtx_` 保护的是 `Info[]`** —— rx 线程写、executor 线程（`on_publish`）读，必须同步。

#### `on_poll()` —— 请求-应答门控 ⭐

```cpp
void on_poll() {
    if (awaiting_.load()) {                 // 上一条还没回复
        if (++stall_ < 30) return;          // 再等 30 个周期（30 × 50ms = 1.5s）
        stall_ = 0;
        awaiting_.store(false);             // 超时，重新试
        RCLCPP_WARN(get_logger(), "STM32 无响应，重试");
    }
    servo_->Statue_get(next_id_);           // 发一条查询
    next_id_ = (next_id_ + 1) % servo_ids_.size();
    awaiting_.store(true);                  // 标记"等回复"
}
```

**这是防止灌爆 STM32 的关键**。STM32 的 `CMD_Queue` 只有 16 深、`UART_task` 每条命令要 ~5ms。
如果节点不看回复就继续发，队列会堆满，`UART_task` 会长时间占着 CPU（优先级 49 高于 CAN_Tx 的 47 和 LED 的 8）——
**这正是最早期「LED 饿死」的成因之一**。

门控逻辑：**没收到回复就不再发新的**，收到后 `rx_loop` 清 `awaiting_` 才放开。

#### `on_publish()` —— 发状态

```cpp
void on_publish() {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = now();
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        ServoStatus s = servo_->Read_Info(servo_ids_[i]);   // 加锁读
        msg.name.push_back(joint_names_[i]);
        msg.position.push_back(servo_to_angle(i, s.position));
    }
    pub_->publish(msg);
}
```

**20ms 一次（50Hz）**，但**关节值本身每 `poll_ms_ × 6` 才刷新一轮** —— 这是"滞后"的来源。

#### `on_command()` —— 收指令

```cpp
void on_command(const JointState::SharedPtr msg) {
    for (size_t k = 0; k < msg->name.size() && k < msg->position.size(); ++k) {
        int i = index_of(msg->name[k]);
        if (i < 0) continue;                        // URDF 里没有的名字，跳过
        uint16_t pos = angle_to_servo(i, msg->position[k]);
        servo_->Position_set(servo_ids_[i], pos, 200);   // 200ms 到位
    }
}
```

#### 映射函数 + `dir_` 方向标定 ⭐

```cpp
double servo_to_angle(size_t i, uint16_t pos) const {
    double sp = servo_max_[i] - servo_min_[i];
    double ap = angle_max_[i] - angle_min_[i];
    if (sp == 0.0) return angle_min_[i];

    double t = (pos - servo_min_[i]) / sp;    // 归一化到 0~1
    if (dir_[i] < 0.0) t = 1.0 - t;           // ← 方向反射
    return angle_min_[i] + t * ap;
}

uint16_t angle_to_servo(size_t i, double a) const {
    ...
    double t = (a - angle_min_[i]) / ap;
    if (dir_[i] < 0.0) t = 1.0 - t;           // ← 同一套反射
    double p = servo_min_[i] + t * sp;
    // 钳位
    return static_cast<uint16_t>(p);
}
```

**为什么用「反射 `t → 1-t`」而不是「取负 `-a`」**：

夹爪的角度范围是 `0 ~ 0.5`（不对称）。取负会变成 `-0.5 ~ 0`，直接越界。
反射对任意范围都成立，而且保证 `angle_to_servo(servo_to_angle(p)) == p`（互逆）。

**`dir` 的实测值**（用 `demo_probe.py` 探出来的）：

| 关节 | URDF 轴 | 实测 +方向 | `dir` |
|---|---|---|---|
| joint1 | `0 0 1` | 俯视逆时针 | `+1` |
| joint2 | `0 1 0` | 往正面 | `+1` |
| joint3 | `0 1 0` | 往后面 | `-1` |
| joint4 | `0 1 0` | 往后面 | `-1` |
| joint5 | `0 0 1` | 俯视逆时针 | `+1` |
| gripper | `0 1 0` | 闭合 | `-1` |

### 2.5 参数表（`config/arm_params.yaml`）

```yaml
arm_server:
  ros__parameters:
    can_interface: "can0"
    poll_period_ms: 50                    # ← 从 200 降到 50，滞后从 1.2s 降到 0.3s
    joint_names: ["joint1","joint2","joint3","joint4","joint5","gripper"]
    servo_ids:   [0, 1, 2, 3, 4, 5]

    # 舵机行程 ↔ URDF 角度 的线性映射
    servo_min:   [500, 500, 500, 500, 500, 0]
    servo_max:   [2500,2500,2500,2500,2500,2000]
    angle_min:   [-1.5708,-1.5708,-1.5708,-1.5708,-3.1416, 0.0]
    angle_max:   [ 1.5708, 1.5708, 1.5708, 1.5708, 3.1416, 0.5]

    dir: [1.0, 1.0, -1.0, -1.0, 1.0, -1.0]   # ← 方向标定
```

**`poll_period_ms` 怎么选**：

| 值 | 全 6 轴刷新 | STM32 占用 | 备注 |
|---|---|---|---|
| 200 | 1200 ms | ~3% | 原值，肉眼明显滞后 |
| **50** | **300 ms** | ~12% | **当前值，推荐** |
| 35 | 210 ms | ~17% | 接近 STM32 单条命令耗时（~6ms）× 6 |

**再往下就要改 STM32 固件**（加"一次查全部舵机"的命令，省 5 次 CAN 往返）。

---

## 三、我生成的例程代码

### 3.1 `demo_probe.py` —— 单关节探针（标定方向）

**位置**：板子 `~/Project/arm_demo/demo_probe.py`（56 行）

**作用**：只让**一个**关节在 ±小幅内做正弦摆动，用来实测两件事：
1. **方向** —— +角度时实机往哪边转（决定 `dir`）
2. **比例** —— 幅度对不对（验证 `servo_min/max` 标定）

**核心代码**：

```python
AMP    = 0.05      # 幅度（弧度）
PERIOD = 4.0       # 一个来回几秒
HZ     = 20

class Probe(Node):
    def tick(self):
        t = time.time() - self.t0
        a = AMP * math.sin(2*math.pi*t/PERIOD)     # ← 从 0 开始，先往正走
        m = JointState()
        m.name = [JOINTS[self.idx]]
        m.position = [a]
        self.pub.publish(m)
```

**关键点**：用 `sin` 而不是 `cos`，**第一段一定是 + 方向**。所以观察"先往哪边"就能直接判定 +方向。

**用法**：
```bash
python3 demo_probe.py 0     # joint1
python3 demo_probe.py 1     # joint2
python3 demo_probe.py 2     # joint3
```

**标定时把 `AMP` 调到 0.5236（30°）**，太小了 3° 肉眼分不清方向 —— 我们第一轮就是这么误判的。

---

### 3.2 `demo_ik_circle.py` —— 逆解扫弧 ⭐ 最核心的例程

**位置**：板子 `~/Project/arm_demo/demo_ik_circle.py`（173 行）

**这是唯一一个真正做"解算"的例程** —— 每一帧都走「给 Cartesian 目标点 → IK 解出关节角 → 下发」。

#### 3.2.1 FK（正解）

```python
L1 = 0.0985        # link2_len  大臂
L2 = 0.0950        # link3_len  小臂
Z0 = 0.0645 + 0.008   # base_height + j1_j2 = 0.0725（joint2 轴心离地高度）

def fk(t1, t2, t3):
    r = L1*math.sin(t2) + L2*math.sin(t2+t3)
    h = L1*math.cos(t2) + L2*math.cos(t2+t3)
    return (r*math.cos(t1), r*math.sin(t1), Z0 + h)
```

**推导**：URDF 里 `joint2/joint3/joint4` 的轴都是 `0 1 0`（绕 Y 转），连杆沿各自 +Z 延伸。
绕 +Y 转 θ 把 `(0,0,L)` 映射到 `(L·sinθ, 0, L·cosθ)`，所以在臂平面内有：

```
径向 r = L1·sin(θ2) + L2·sin(θ2+θ3)
高度 h = L1·cos(θ2) + L2·cos(θ2+θ3)
```

再乘上底盘旋转 `θ1` 得到世界坐标。**`θ2=0, θ3=0` 时大臂小臂都竖直 —— 和你实机在 1500 位置一致。**

#### 3.2.2 IK（逆解）

```python
def ik(x, y, z, t3_ref=0.0):
    t1 = math.atan2(y, x)              # ① 底盘直接解出来
    r  = math.hypot(x, y)
    h  = z - Z0
    d  = math.hypot(r, h)              # ② 目标点到 joint2 轴心的距离

    c3 = (d*d - L1*L1 - L2*L2) / (2.0*L1*L2)
    if c3 > 1.0 or c3 < -1.0:
        return None                    # 够不着
    a3 = math.acos(c3)                 # ③ 余弦定理求肘部角

    best = None
    for t3 in (a3, -a3):               # ④ 两个分支：肘上 / 肘下
        k1 = L1 + L2*math.cos(t3)
        k2 = L2*math.sin(t3)
        t2 = math.atan2(k1*r - k2*h, k2*r + k1*h)   # ⑤ 反解肩部角
        if best is None or abs(t3 - t3_ref) < abs(best[2] - t3_ref):
            best = (t1, t2, t3)        # 取离上次近的分支，保证连续
    return best
```

**⑤ 的公式推导**（这是最容易写错的一步）：

展开 `r` 和 `h` 的表达式，把 `θ2` 的项归并：

```
r = k1·sin(θ2) + k2·cos(θ2)
h = k1·cos(θ2) - k2·sin(θ2)

其中 k1 = L1 + L2·cos(θ3),  k2 = L2·sin(θ3)
```

解这个二元一次方程组（用 `k1²+k2² = d²` 化简）：

```
θ2 = atan2(k1·r - k2·h,  k2·r + k1·h)
```

#### 3.2.3 三个几何约束（都是实测踩出来的）

**① 起点在奇异位置**

实机竖直时 `θ2=θ3=0`，末端在 `(0, 0, Z0+L1+L2)` —— **正好在工作空间边界上，伸展度 100%**。
此时任何方向的小位移都会超出可达范围。

**② 所以圆只能"往下兜"**

```python
# 圆心取“当前末端正下方 R + 余量”
r_c = r0
h_c = h0 - RADIUS - MARGIN
r = r_c + RADIUS * math.sin(phi)
h = h_c + RADIUS * math.cos(phi)
```
这样圆上每一点的 `h ≤ h0`，`d ≤ L1+L2` 必然成立。

**③ 径向位移必须带符号**

第一版我写成 `x = r·cos(t1_0)`，后半圈 `r` 变负 → `atan2` 把 `θ1` 解成 180° → 被限幅挡掉 → **只走半圈就卡住**。

正确做法：**`θ1` 固定**，平面 2R 用**有符号**的径向坐标解：

```python
def ik_planar(rs, h, t3_ref):
    # rs 可以为负 —— 负值表示"大臂往后仰"，不是"底盘转 180°"
    d = math.hypot(rs, h)              # hypot 对负值自动取绝对值
    ...
    t2 = math.atan2(k1*rs - k2*h, k2*rs + k1*h)
```

#### 3.2.4 最终版本：底盘摆动扫弧

**约束**：实机 `j1` 只能 ±90°，**画不出整圆**。

**方案**：底盘在 ±`SWEEP` 内按正弦摆动，末端在水平面上扫出一段大圆弧。

```python
SWEEP  = math.radians(45)     # 底盘摆幅（实机上限 ±90°，留余量）
PERIOD = 10.0                 # 10 秒一个来回

# 阶段 1（3 秒）：平滑弯到工作姿态，让末端偏离转轴
if t < PREP_SEC:
    s = t / PREP_SEC
    s = s*s*(3-2*s)                          # smoothstep，起停都缓
    t2 = self.t2_0 + (WORK_J2 - self.t2_0) * s
    t3 = self.t3_0 + (WORK_J3 - self.t3_0) * s
    self.send([self.t1_0, t2, t3])
    return

# 阶段 2：底盘摆动，末端扫弧
u   = 2*math.pi*(t - PREP_SEC) / PERIOD
phi = SWEEP * math.sin(u)                    # ← sin 保证来回平滑
x = self.r_c * math.cos(self.t1_0 + phi)
y = self.r_c * math.sin(self.t1_0 + phi)
z = self.z_c + HEIGHT_AMP * math.sin(u)

sol = ik(x, y, z, t3_ref=WORK_J3)            # ← 每帧都在解算
```

**三层安全保护**（任何一层拦不住才会撞限位）：

| 层 | 位置 | 作用 |
|---|---|---|
| 1 | `ik()` 返回 `None` | 目标超出臂长，跳过该帧 |
| 2 | `LIMIT` 数组 | 解出的角度超出 ±40°~90° 就跳过 |
| 3 | `arm_server` 的 `angle_to_servo` | 再钳到 `servo_min/max` |

**关键参数**：

| 参数 | 作用 |
|---|---|
| `WORK_J2` / `WORK_J3` | 工作姿态，决定弧的半径 |
| `SWEEP` | 底盘摆幅，决定弧长 |
| `PERIOD` | 一个来回几秒 |
| `HEIGHT_AMP` | 给 0.02 就变成斜着的立体弧 |

---

### 3.3 `bridge_tx.py` / `bridge_rx.py` —— SSH 桥接

**为什么不用 DDS 直连**：开发机是 WSL2 的 **NAT 网络**，板子无法反向发现它。
而 DDS 的发现过程是**双向**的（板子组播"我在"，开发机也要能被板子发现），所以跨不过 NAT。

**解法**：走 SSH 管道，板子把数据以 JSON 行打到 stdout，开发机读进来重新发布。
**完全绕开 DDS 的跨机发现问题。**

#### `bridge_tx.py`（板子上跑，44 行）

```python
class Tx(Node):
    def __init__(self):
        super().__init__("joint_state_bridge_tx")
        self.create_subscription(JointState, "/joint_states", self.on_states, 10)

    def on_states(self, msg):
        print(json.dumps({
            "name": list(msg.name),
            "position": [float(p) for p in msg.position],
        }), flush=True)          # ← flush=True 必须，否则缓冲导致延迟
```

**只订阅，不发布** —— 板子侧完全不碰 `/joint_commands`，**没有任何路径能驱动机械臂**。

#### `bridge_rx.py`（开发机跑，111 行）

```python
GRIPPER_IN_MAX  = 0.5      # arm_server 的 gripper 角度范围
GRIPPER_OUT_MAX = 0.02     # URDF 棱柱关节范围（米，finger_travel）
FINGER_JOINTS   = ["gripper_finger_left_joint", "gripper_finger_right_joint"]

class Rx(Node):
    def feed(self, line):
        data = json.loads(line)
        names, pos = list(data["name"]), list(data["position"])

        if "gripper" in names:
            g = pos[names.index("gripper")]
            if g > GRIPPER_OUT_MAX:                    # arm_server 还在用 0~0.5
                g = g * GRIPPER_OUT_MAX / GRIPPER_IN_MAX
            names += FINGER_JOINTS                     # ← 展开成两个手指关节
            pos += [g, g]

        msg = JointState()
        msg.name, msg.position = names, pos
        self.pub.publish(msg)
```

**做了一件 robot_state_publisher 需要的事**：`arm_server` 只发一个 `gripper`，
但 URDF 里有两个棱柱手指关节（`gripper_finger_left/right_joint`）。这里自动展开。

**线程模型**（踩过坑）：

```python
executor = SingleThreadedExecutor()
executor.add_node(node)
spin_thread = threading.Thread(target=_spin_quiet, args=(executor,), daemon=True)

def _spin_quiet(executor):
    try:
        executor.spin()
    except Exception:
        pass                       # executor.shutdown() 时 spin() 会抛，吞掉

# 退出：
executor.shutdown()                # ① 先让 spin() 返回
spin_thread.join(timeout=2.0)      # ② 再 join
try: node.destroy_node()
except Exception: pass
try: rclpy.shutdown()              # ③ rclpy 的 SIGTERM 处理器可能已经关过
except Exception: pass
```

**不用这套会怎样**：
- 直接用 `rclpy.spin(node)` 的线程 → 退出时 C++ 层还有 joinable thread → `std::terminate`（core dump）
- 不加 try/except → SIGTERM 时 `rcl_shutdown already called` 抛异常

---

### 3.4 `viz.launch.py` / `run_viz.sh` —— 可视化

#### `viz.launch.py`（54 行）

```python
def generate_launch_description():
    robot_description = xacro.process_file(xacro_file).toxml()   # launch 时展开 xacro
    return LaunchDescription([
        Node(package='robot_state_publisher', ..., 
             parameters=[{'robot_description': robot_description}]),
        Node(package='rviz2', ..., arguments=['-d', rviz_config],
             condition=IfCondition(use_rviz)),
    ])
```

**和 `display.launch.py` 的区别（这是关键）**：

| | `display.launch.py` | `viz.launch.py` |
|---|---|---|
| `robot_state_publisher` | ✅ | ✅ |
| `rviz2` | ✅ | ✅ |
| **`joint_state_publisher`** | ✅ **会发全 0 的 `/joint_states`** | ❌ **不启动** |

`display.launch.py` 在 `use_gui:=false` 时会启动 `joint_state_publisher`，
**它自己也会往 `/joint_states` 发数据（全 0）**，和桥接过来的真实数据打架 → RViz 里模型会抖。
`viz.launch.py` 不启动任何关节值发布器，`/joint_states` 完全交给桥接。

#### `run_viz.sh`（100 行）

一键：**预检 → 桥接 → 启动 RViz**。四道预检：

```bash
① SSH 能连上板子吗
② 板上 ~/Project/arm_demo/bridge_tx.py 在吗    → 不在会告诉你 scp 命令
③ 板上 arm_server 在跑吗                        → 没跑会告诉你启动命令
④ 真的能抓到一帧 JSON 吗                        → 抓不到会打印手动排查命令
```

**踩过的三个坑**（都在这个脚本里修掉了）：

| 坑 | 症状 | 修法 |
|---|---|---|
| `set -u` | `AMENT_TRACE_SETUP_FILES: unbound variable` 直接退出 | ROS 的 `setup.bash` 会引用未定义变量，**不能开 `nounset`** |
| SSH 不 source ROS | 板子上 `import rclpy` 失败 → 管道立刻关 → 转发 0 帧 | 非交互 SSH 不读 `.bashrc`，**必须显式 `source /opt/ros/jazzy/setup.bash`** |
| `pipefail` 误判 | `ssh ... \| head -1 \| grep -q` 里 ssh 被 timeout 打断 → 整条管道判失败 | **先抓成字符串再匹配**：`SAMPLE="$(ssh ... \| head -1)"; case "$SAMPLE" in ...` |

---

### 3.5 `slider_teleop.py` / `teleop.launch.py` / `run_teleop.sh` —— 滑条遥操作

> ⚠️ **已弃用**（用户判断"太危险"）。保留代码备查。

**需求**：拖 `joint_state_publisher_gui` 的滑条 → 真机动。

**两个障碍**：

1. `joint_state_publisher_gui` 发的是 `/joint_states`，而 `arm_server` 收的是 `/joint_commands`
2. `/joint_states` 已经被桥接的真实数据占用了，GUI 再往上发会打架

**解法**：

```
joint_state_publisher_gui ──(remap)──► /joint_slider
                                            │
                              slider_teleop.py（钳位+死区+换算）
                                            │
                                            ▼
                                      /joint_commands ──► 真机
```

```python
# teleop.launch.py
Node(package='joint_state_publisher_gui', ...,
     remappings=[('/joint_states', '/joint_slider')])   # ← 关键的一行
```

```python
# slider_teleop.py
SAFE = {"joint1": (-1.20, 1.20), ...}       # ±69°，比 URDF 的 ±90° 收窄
DEADBAND = 0.01                              # 变化 < 0.57° 不发，省 CAN 带宽
FINGER_SCALE = 0.5 / 0.02                    # 手指行程 0~0.02m → gripper 0~0.5

def on_slider(self, msg):
    for name, val in zip(msg.name, msg.position):
        if name in ("gripper_finger_left_joint", "gripper_finger_right_joint"):
            key, v = "gripper", val * FINGER_SCALE     # 两个手指合并成 gripper
        elif name in SAFE:
            key, v = name, val
        else:
            continue
        v = clamp(v, *SAFE[key])                       # 钳位
        if abs(v - self.last.get(key, 1e9)) < DEADBAND: continue   # 死区
        ...
```

**实测验证过三个行为**：

| 输入滑条 | 实际下发 | 结果 |
|---|---|---|
| `joint1 = 0.7` | `joint1 = 0.7` | ✅ 原样通过 |
| `joint2 = -9.0` | `joint2 = -1.2` | ✅ 钳到安全上限 |
| `gripper_finger_left = 0.01` | `gripper = 0.25` | ✅ 行程换算正确 |

**为什么危险**（弃用的理由）：

1. 滑条范围就是 URDF 的机械极限（±90°），手一抖就到底
2. 没有力反馈 / 碰撞检测，撞到东西舵机只会硬顶
3. 滑条是"绝对目标位置"不是"增量"，中间没有速度规划

---

## 四、踩过的坑（按层次汇总）

### STM32 固件层

| 坑 | 症状 | 根因 | 修法 |
|---|---|---|---|
| **阻塞轮询饿死 LED** | LED 不闪 | `UART_task` 队列非空时 `continue`，不让出 CPU；优先级 49 > CAN_Tx 47 > LED 8 | 加 `osDelay(1)` |
| **固定读 11 字节** | 偶发读错位、温度 235℃ | 字节数不对就整体错位 | 改成找 `#...!` 帧边界 |
| **半双工回显** | 收到自己发的命令 | `HDSEL=1` + `RE=1`，发送时收到自己的字节 | 发送后清 RDR + 清 ORE |
| **IDLE 竞态** | 帧被劈成两半、串台 | CAN 中断（优先级 51）抢占 USART2（优先级 5），IDLE 被推迟到第一个字节之后，HAL 就把 DMA 停了 | **放弃 IDLE**，大缓冲 + 轮询 `!` |
| **callback 名字不匹配** | 全部超时，`R0`/`P0` | `HAL_UART_Receive_DMA` 调的是 `HAL_UART_RxCpltCallback`，不是 `HAL_UARTEx_RxEventCallback` | 统一用轮询，不用回调 |
| **温度单位不统一** | 伺服 0 温度恒为 0 | 伺服 0 回 `#000T25V05!`（℃），伺服 1~5 回 `#001T1845-08.1!`（℃×100） | 伺服 0 乘 100 归一化 |

### Linux / CAN 层

| 坑 | 修法 |
|---|---|
| `CAN_ERR_CNT = 0x200` 和 `GETANGLE` 撞车 | 加 `if (frame.can_id & CAN_ERR_FLAG) return false;` |
| `Can_Write` 发未初始化的 `data[dlc..7]` | `struct can_frame frame{};` |
| `bind` / `setsockopt` 返回值没检查 | 加上检查 |

### ROS 2 层

| 坑 | 修法 |
|---|---|
| `joint_state_publisher` 和桥接抢 `/joint_states` | 写专用的 `viz.launch.py`，不启动它 |
| `dir` 方向没标定 | 探针实测 + 反射式映射 |
| 滞后 1.2 秒 | `poll_period_ms` 200 → 50 |
| `set -u` 撞 ROS setup | 脚本不开 `nounset` |
| `pipefail` 误判自检 | 先抓字符串再匹配 |
| `rclpy.shutdown()` 二次调用 | try/except 兜住 |

---

## 五、当前状态与待办

### ✅ 已打通

```
URDF ──► IK 解算 ──► /joint_commands ──► CAN 0x104 ──► STM32 ──► 6 个舵机
                                                            │
RViz ◄── robot_state_publisher ◄── /joint_states ◄── CAN 0x200 ◄──┘
                                        ▲
                                   SSH 桥接
```

| 环节 | 状态 |
|---|---|
| STM32 固件 | ✅ DMA + 轮询判帧，6 舵机稳定 |
| CAN 链路 | ✅ 每条 `104` → 12ms 后 `200` |
| `arm_server` | ✅ 参数/方向/门控都到位，轮询 50ms |
| IK 扫弧 Demo | ✅ 底盘 ±45°，夹爪扫弧 |
| RViz 可视化 | ✅ 启动 + 跟随，滞后已从 1.2s 降到 ~0.3s |

### ⬜ 待办

| # | 项 | 位置 | 影响 |
|---|---|---|---|
| 1 | 夹爪单位不匹配（`angle_max[5]` 是 0.5，URDF 期望 0.02） | `arm_params.yaml` | `bridge_rx.py` 已自动缩放兜底，根治要改 YAML |
| 2 | URDF 两个手指关节，`joint_names` 只有一个 | `arm_server_node.cpp` | RViz 右手指不动（bridge 已兜底） |
| 3 | `CAN_ERR_CNT` 和 `GETANGLE` 撞车 | `Servo_ctrl.cpp:72` | 错误帧会被误认成状态帧 |
| 4 | `Can_Write` 未初始化字节 | `can_init.cpp:50` | 目前无害，隐患 |
| 5 | `bind` / `setsockopt` 没检查返回 | `can_init.cpp:32-37` | 失败会静默 |
| 6 | `package.xml` 名字是大写 `CanServer` | `package.xml` | ROS 2 规范要小写 |
| 7 | `demo/` 没进版本管理 | WSL 工作区 | 建议提交 |
| 8 | **滞后想再降** | STM32 固件 | 加"一次查全部舵机"命令，能压到 ~150ms |

### 文件清单

**WSL `~/ros2_project/my_robot_arm/demo/`**（8 个）
```
README.md  bridge_tx.py  bridge_rx.py  viz.launch.py  run_viz.sh
slider_teleop.py  teleop.launch.py  run_teleop.sh   ← 后三个已弃用
```

**板子 `~/Project/arm_demo/`**（3 个）
```
bridge_tx.py  demo_probe.py  demo_ik_circle.py
```
