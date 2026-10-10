# RK3568 机械臂 · ros2_control 接入实施记录

> **状态**：✅ 第一部分（让它动起来）已跑通 ｜ ⬜ 第二部分（MoveIt2 规划）待做
>
> 本文档记录从「裸机 CAN 舵机」到「ROS 2 标准控制框架」的完整实施过程，
> **所有代码都是板子上实际在跑的版本**。

---

# 目录

```
第一部分  让它动起来（ros2_control）
  1. 架构总览
  2. 文件清单
  3. robot_arm.ros2_control.xacro
  4. ArmSystemHardware 插件（hpp + cpp + xml）
  5. CMakeLists.txt / package.xml
  6. ros2_controllers.yaml
  7. arm_control.launch.py
  8. 编译与启动验证
  9. 发轨迹让它动
 10. 踩过的坑（6 个）

第二部分  接 MoveIt2（规划）
 11. MoveIt2 与 ros2_control 的分工
 12. moveit_config 的 5 个文件
 13. 5-DOF 的 IK 难题
 14. 自定义 IK 插件
 15. 端到端 launch

附录  A. 命令速查   B. 排查表
```

---

# 1. 架构总览

## 分层

```
┌─────────────────────────────────────────────────────────────┐
│ 应用层     RViz 拖末端 / send_traj.py / moveit_py            │  ⬜ 待做
├─────────────────────────────────────────────────────────────┤
│ 规划层     ★ MoveIt2 move_group ★                           │  ⬜ 待做
│      · 逆运动学（末端位姿 → 关节角）                          │
│      · 碰撞检测                                             │
│      · 路径规划（OMPL 采样）                                 │
│      · 时间参数化                                            │
├─────────────────────────────────────────────────────────────┤
│ 控制层     joint_trajectory_controller                      │  ✅ 已通
│      · 把轨迹按时间插值                                       │
├─────────────────────────────────────────────────────────────┤
│ 硬件层     ArmSystemHardware（自研插件）                      │  ✅ 已通
│      · 实现 hardware_interface::SystemInterface              │
│      · 复用已有的 arm_can 库（SocketCAN + 舵机协议）           │
├─────────────────────────────────────────────────────────────┤
│ 总线       CAN 2.0B @250kbps                                 │  ✅
├─────────────────────────────────────────────────────────────┤
│ 执行       STM32F407 (FreeRTOS) → USART2 半双工 → 6× 舵机     │  ✅
└─────────────────────────────────────────────────────────────┘
```

## 数据流

```
ros2 action send_goal /arm_controller/follow_joint_trajectory
              ↓
   joint_trajectory_controller          （ros2_controllers）
      把「终点角度」插值成「随时间变化的位置序列」
              ↓ 写 command_interface
   ArmSystemHardware::write()           （自研插件）
      角度 → 舵机位置 → servo_->Position_set()
              ↓ CAN 0x104
   STM32F407 → USART2 → 6× 舵机
              ↑ CAN 0x200
   ArmSystemHardware::read()            （自研插件）
      舵机位置 → 关节角 → state_interface
              ↓
   joint_state_broadcaster → /joint_states
```

---

# 2. 文件清单

| # | 文件 | 位置 | 行数 | 作用 |
|---|---|---|---|---|
| 1 | `robot_arm.ros2_control.xacro` | `my_robot_arm_description/urdf/` | 53 | 声明硬件插件 + 关节接口 |
| 2 | `arm_system_hardware.hpp` | `CanServer/include/` | 76 | 插件类声明 |
| 3 | `arm_system_hardware.cpp` | `CanServer/src/` | 289 | 插件实现 |
| 4 | `arm_system_hardware.xml` | `CanServer/` | 6 | pluginlib 插件声明 |
| 5 | `CMakeLists.txt` | `CanServer/` | 72 | 构建 + 安装 |
| 6 | `package.xml` | `CanServer/` | 18 | 依赖声明 |
| 7 | `ros2_controllers.yaml` | `CanServer/config/` | 37 | 控制器参数 |
| 8 | `arm_control.launch.py` | `CanServer/launch/` | 51 | 一键启动 |

**另外要在主 xacro 里加一行 include**（`robot_arm.urdf.xacro` 第 317 行）：

```xml
  <xacro:include filename="$(find my_robot_arm_description)/urdf/robot_arm.ros2_control.xacro"/>
```

---

# 3. `robot_arm.ros2_control.xacro`

**作用**：告诉 `ros2_control` —— 「这台机械臂用哪个插件、有哪些关节、哪些接口」。

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!--
    关节顺序 = <param> 数组顺序 = 舵机 id 顺序，必须一致！
    硬件插件按 info_.joints 的下标去索引这些数组。
  -->
  <ros2_control name="MyRobotArm" type="system">
    <hardware>
      <plugin>CanServer/ArmSystemHardware</plugin>

      <param name="can_interface">can0</param>
      <param name="cmd_time_ms">150</param>          <!-- 与控制器周期对齐，见阶段2 -->

      <param name="servo_ids">0,1,2,3,4,5</param>
      <param name="servo_min">500,500,500,500,500,0</param>
      <param name="servo_max">2500,2500,2500,2500,2500,2000</param>

      <!-- ★ 注意：这里从「角度」改成「位置」，夹爪是直线（米）不是角度 -->
      <param name="pos_min">-1.5708,-1.5708,-1.5708,-1.5708,-3.1416,0.0</param>
      <param name="pos_max">1.5708,1.5708,1.5708,1.5708,3.1416,0.020</param>

      <param name="dir">1.0,1.0,-1.0,-1.0,1.0,-1.0</param>
    </hardware>

    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
    <joint name="joint3">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
    <joint name="joint4">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
    <joint name="joint5">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>

    <!-- 夹爪：只暴露左指。右指是 mimic，由 MoveIt/Gazebo 层面处理，不占接口 -->
    <joint name="gripper_finger_left_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>

</robot>
```

## 关键点

| 项 | 说明 |
|---|---|
| `<plugin>` | **必须和 `arm_system_hardware.xml` 里的 `name` 完全一致**（`CanServer/ArmSystemHardware`） |
| `<param>` | 会以 `hardware_parameters` 传给插件的 `on_init(info)`，用 `info.hardware_parameters.at("key")` 取 |
| `<joint>` | **每个要控制的关节必须在这里声明**，且 `name` 要和 URDF 里的关节名一致 |
| `command_interface` | 我们只用 `position` |
| `state_interface` | `position` 就够了（插件额外导出了 `velocity`，但这里没声明） |

## ⚠️ 两个坑

**① 文件第 1 行前面不能有空格**

XML 规范要求 `<?xml?>` 必须是文件第一个字符：

```bash
# 自查（期望输出 3c3f786d6c207665 = "<?xml ve"）
head -c 8 robot_arm.ros2_control.xacro | xxd -p

# 有前导空格就修
sed -i '1s/^[[:space:]]*//' robot_arm.ros2_control.xacro
```

**报错长这样**：
```
XML parsing error: XML or text declaration not at start of entity: line 1, column 1
```

**② `pos_max` 的逗号后别留空格**

```xml
<param name="pos_max"> 1.5708, 1.5708, ...</param>   ← 第 1 个值前有空格
<param name="pos_max">1.5708,1.5708,...</param>      ← 干净
```

---

# 4. ArmSystemHardware 插件

## 4.1 头文件 `include/arm_system_hardware.hpp`

```cpp
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "can_init.h"
#include "Servo_ctrl.h"

namespace arm_hardware {

class ArmSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArmSystemHardware)

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
      export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
      export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── 从 xacro 的 <param> 读来的标定参数 ──
  std::string          can_interface_ = "can0";
  int                  cmd_time_ms_   = 150;
  int                  poll_div_      = 30;    // 每 N 个控制周期问一个舵机
  std::vector<uint8_t> servo_ids_;
  std::vector<double>  servo_min_, servo_max_;
  std::vector<double>  pos_min_,   pos_max_;
  std::vector<double>  dir_;

  // ── 复用你已有的、不依赖 ROS 的 CAN / 舵机类 ──
  // ⚠️ ServoCtrl_object 持有 Cansocket_object& 引用
  //    所以 can_ 必须先声明（后析构）
  std::unique_ptr<Cansocket_object> can_;
  std::unique_ptr<ServoCtrl_object> servo_;

  // ── 接口缓存（下标 = info_.joints 的下标）──
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // ── 运行时状态 ──
  size_t tick_     = 0;      // 控制周期计数
  size_t next_id_  = 0;      // 轮询到哪个舵机
  double cmd_prev_[8] = {0}; // 上次下发的命令（死区用）
  bool first_write_ = true;

  double   servoToAngle(size_t i, uint16_t pos) const;
  uint16_t angleToServo(size_t i, double a) const;

};

}  // namespace arm_hardware
```

## 4.2 实现 `src/arm_system_hardware.cpp`

```cpp
#include "arm_system_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <net/if.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace arm_hardware {

// ─────────── 工具：解析 "1.0, 2.0, 3.0" ───────────
static bool splitDoubles(const std::string & s, std::vector<double> & out)
{
  out.clear();
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    const auto b = tok.find_first_not_of(" \t\r\n");
    const auto e = tok.find_last_not_of(" \t\r\n");
    if (b == std::string::npos) continue;
    try {
      out.push_back(std::stod(tok.substr(b, e - b + 1)));
    } catch (...) {
      return false;
    }
  }
  return !out.empty();
}

static bool getParam(const hardware_interface::HardwareInfo & info,
                     const std::string & key, std::string & out)
{
  auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) return false;
  out = it->second;
  return true;
}

// ─────────── on_init：读参数 + 建 CAN ───────────
hardware_interface::CallbackReturn ArmSystemHardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const size_t n = info_.joints.size();

  // ① 读参数
  std::string v;
  if (getParam(info_, "can_interface", v)) can_interface_ = v;
  if (getParam(info_, "cmd_time_ms",  v)) cmd_time_ms_   = std::stoi(v);
  if (getParam(info_, "poll_div",     v)) poll_div_      = std::max(1, std::stoi(v));

  std::vector<double> d;
  if (!getParam(info_, "servo_ids", v) || !splitDoubles(v, d)) {
    RCLCPP_FATAL(get_logger(), "缺少或无法解析参数 servo_ids");
    return hardware_interface::CallbackReturn::ERROR;
  }
  servo_ids_.clear();
  for (double x : d) servo_ids_.push_back(static_cast<uint8_t>(x));

  auto need = [&](const char * key, std::vector<double> & dst) -> bool {
    std::string s;
    if (!getParam(info_, key, s) || !splitDoubles(s, dst)) {
      RCLCPP_FATAL(get_logger(), "缺少或无法解析参数 %s", key);
      return false;
    }
    if (dst.size() != n) {
      RCLCPP_FATAL(get_logger(), "参数 %s 有 %zu 项，但关节有 %zu 个",
                   key, dst.size(), n);
      return false;
    }
    return true;
  };

  if (!need("servo_min", servo_min_) ||
      !need("servo_max", servo_max_) ||
      !need("pos_min",   pos_min_)   ||
      !need("pos_max",   pos_max_)   ||
      !need("dir",       dir_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (servo_ids_.size() != n) {
    RCLCPP_FATAL(get_logger(), "servo_ids 有 %zu 项，但关节有 %zu 个",
                 servo_ids_.size(), n);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ② 建 CAN（完全复用你已有的库）
  struct sockaddr_can addr {};
  addr.can_family  = AF_CAN;
  addr.can_ifindex = static_cast<int>(if_nametoindex(can_interface_.c_str()));
  if (addr.can_ifindex == 0) {
    RCLCPP_FATAL(get_logger(), "找不到网络接口 %s", can_interface_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::vector<struct can_filter> filters = {{GETANGLE, CAN_SFF_MASK}};
  can_ = std::make_unique<Cansocket_object>(addr, filters);
  if (!can_->Can_Socket_Init(can_interface_)) {
    RCLCPP_FATAL(get_logger(), "CAN 初始化失败 (%s): %s",
                 can_interface_.c_str(), can_->Can_geterr().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  servo_ = std::make_unique<ServoCtrl_object>(*can_);

  // ③ 初始化缓存
  hw_commands_.assign(n, 0.0);
  hw_positions_.assign(n, 0.0);
  hw_velocities_.assign(n, 0.0);

  RCLCPP_INFO(get_logger(),
              "ArmSystemHardware 就绪：%zu 关节, iface=%s, cmd_time=%dms, poll_div=%d",
              n, can_interface_.c_str(), cmd_time_ms_, poll_div_);
  for (size_t i = 0; i < n; ++i) {
    RCLCPP_INFO(get_logger(),
                "  [%zu] %-28s servo_id=%u  servo=[%.0f,%.0f]  pos=[%.4f,%.4f]  dir=%+.0f",
                i, info_.joints[i].name.c_str(), servo_ids_[i],
                servo_min_[i], servo_max_[i], pos_min_[i], pos_max_[i], dir_[i]);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────── 导出接口 ───────────
std::vector<hardware_interface::StateInterface>
ArmSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    out.emplace_back(info_.joints[i].name,
                     hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    out.emplace_back(info_.joints[i].name,
                     hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface>
ArmSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    out.emplace_back(info_.joints[i].name,
                     hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return out;
}

// ─────────── 生命周期 ───────────
hardware_interface::CallbackReturn ArmSystemHardware::on_activate(
    const rclcpp_lifecycle::State &)
{
  const size_t n = info_.joints.size();

  // ① 发 6 条查询
  for (size_t i = 0; i < n; ++i) {
    servo_->Statue_get(servo_ids_[i]);
  }

  // ② ★ 按【时间】等，收集到全部或超时为止
  //    （原来是 Info_wait(10) 一失败就 break —— 这是主因）
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(1000);
  int received = 0;
  while (received < static_cast<int>(n) &&
         std::chrono::steady_clock::now() < deadline) {
    if (servo_->Info_wait(20)) ++received;
  }
  if (received < static_cast<int>(n)) {
    RCLCPP_WARN(get_logger(), "只收到 %d/%zu 个舵机回复，用已有数据继续",
                received, n);
  }

  // ③ ★ 读到的位置必须 clamp，且 cmd_prev_ 用同一个口径
  for (size_t i = 0; i < n; ++i) {
    const ServoStatus s = servo_->Read_Info(servo_ids_[i]);
    double pos = servoToAngle(i, s.position);           // 内部已 clamp
    pos = std::clamp(pos, pos_min_[i], pos_max_[i]);    // 再兜一层
    hw_positions_[i] = pos;
    hw_commands_[i]  = pos;
    cmd_prev_[i]     = pos;      // ★ 关键：和 write() 里 clamp 后的值一致
  }

  RCLCPP_INFO(get_logger(), "已激活，命令初值 = 当前关节角：");
  for (size_t i = 0; i < n; ++i) {
    RCLCPP_INFO(get_logger(), "  %-28s %.4f rad",
                info_.joints[i].name.c_str(), hw_commands_[i]);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "已停用");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────── ★ read：从 CAN 收状态 ───────────
hardware_interface::return_type ArmSystemHardware::read(
    const rclcpp::Time &, const rclcpp::Duration &)
{
  const size_t n = info_.joints.size();

  // ① 非阻塞把已经回来的帧全收掉
  //    ⚠️ 如果 Info_wait 在失败时打日志，这里会刷屏 —— 见下面的注意事项
  for (int guard = 0; guard < 16; ++guard) {
    if (!servo_->Info_wait(0)) break;
  }

  // ② 每 poll_div_ 个周期问一个舵机（轮转）
  if (++tick_ % static_cast<size_t>(poll_div_) == 0) {
    servo_->Statue_get(servo_ids_[next_id_]);
    next_id_ = (next_id_ + 1) % n;
  }

  // ③ 从缓存映射到关节角
  for (size_t i = 0; i < n; ++i) {
    const ServoStatus s = servo_->Read_Info(servo_ids_[i]);
    hw_positions_[i] = servoToAngle(i, s.position);
  }
  return hardware_interface::return_type::OK;
}

// ─────────── ★ write：把命令下发到 CAN ───────────
hardware_interface::return_type ArmSystemHardware::write(
    const rclcpp::Time &, const rclcpp::Duration &)
{
  const size_t n = info_.joints.size();
  if (first_write_) {
    for (size_t i = 0; i < n; ++i) {
      cmd_prev_[i] = std::clamp(hw_commands_[i], pos_min_[i], pos_max_[i]);
    }
    first_write_ = false;
    RCLCPP_INFO(get_logger(), "首次 write：仅同步初值，不下发命令");
    return hardware_interface::return_type::OK;
  }
  constexpr double DEADBAND = 0.005;
  for (size_t i = 0; i < n; ++i) {
    double cmd = hw_commands_[i];
    // 钳到关节限位（MoveIt2 规划过，但这里再兜一层）
    cmd = std::clamp(cmd, pos_min_[i], pos_max_[i]);

    if (std::fabs(cmd - cmd_prev_[i]) < DEADBAND) continue;

    const uint16_t pos = angleToServo(i, cmd);
    if (!servo_->Position_set(servo_ids_[i], pos,
                              static_cast<uint16_t>(cmd_time_ms_))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Position_set 失败 servo_id=%u: %s",
                           servo_ids_[i], servo_->Servo_errget().c_str());
    }
    cmd_prev_[i] = cmd;
  }
  return hardware_interface::return_type::OK;
}

// ─────────── 映射（和你 arm_server 里完全一致）───────────
double ArmSystemHardware::servoToAngle(size_t i, uint16_t pos) const
{
  const double sp = servo_max_[i] - servo_min_[i];
  const double ap = pos_max_[i]   - pos_min_[i];
  if (sp == 0.0) return pos_min_[i];

  double t = (static_cast<double>(pos) - servo_min_[i]) / sp;
  t = std::clamp(t, 0.0, 1.0);            // ★★★ 加这一行
  if (dir_[i] < 0.0) t = 1.0 - t;
  return pos_min_[i] + t * ap;
}

uint16_t ArmSystemHardware::angleToServo(size_t i, double a) const
{
  const double sp = servo_max_[i] - servo_min_[i];
  const double ap = pos_max_[i]   - pos_min_[i];
  if (ap == 0.0) return static_cast<uint16_t>(servo_min_[i]);

  double t = (a - pos_min_[i]) / ap;
  if (dir_[i] < 0.0) t = 1.0 - t;          // ★ 反射
  t = std::clamp(t, 0.0, 1.0);
  return static_cast<uint16_t>(servo_min_[i] + t * sp);
}

}  // namespace arm_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(arm_hardware::ArmSystemHardware,
                       hardware_interface::SystemInterface)
```

## 4.3 插件声明 `arm_system_hardware.xml`

```xml
<library path="arm_system_hardware">
  <class name="CanServer/ArmSystemHardware"
         type="arm_hardware::ArmSystemHardware"
         base_class_type="hardware_interface::SystemInterface">
    <description>CAN bus hardware interface for the 5-DOF arm (reuses arm_can)</description>
  </class>
</library>
```

## 4.4 三个设计要点

### ① 复用已有的 `arm_can` 库

`Cansocket_object` 和 `ServoCtrl_object` **本来就不依赖 ROS**，所以插件直接复用，一行没改：

```cpp
can_   = std::make_unique<Cansocket_object>(addr, filters);
servo_ = std::make_unique<ServoCtrl_object>(*can_);
```

**这也是当初把 CAN 层做成独立库的回报** —— 上层从 `arm_server` 节点换成 `ros2_control` 插件，底层完全不用动。

### ② ⚠️ 声明顺序不能反

`ServoCtrl_object` 持有 `Cansocket_object&` **引用**：

```cpp
std::unique_ptr<Cansocket_object> can_;    // ← 必须先声明（后析构）
std::unique_ptr<ServoCtrl_object> servo_;  // ← 后声明（先析构）
```

**反过来会 use-after-free。**

### ③ `read()` 的轮询策略

`read()` 每个控制周期（20ms）都会被调用，但**不能每周期都问 6 个舵机**（会淹总线）。

策略：
```cpp
// ① 非阻塞把已经回来的帧收掉
for (int guard = 0; guard < 16; ++guard)
    if (!servo_->Info_wait(0)) break;

// ② 每 poll_div_ 个周期问一个舵机（轮转）
if (++tick_ % poll_div_ == 0) {
    servo_->Statue_get(servo_ids_[next_id_]);
    next_id_ = (next_id_ + 1) % n;
}

// ③ 从缓存映射到关节角
for (...) hw_positions_[i] = servoToAngle(i, servo_->Read_Info(...).position);
```

**`poll_div = 30` → 30×20ms = 600ms 问一个 → 6 个舵机轮一圈 3.6 秒。**

⚠️ **做 MoveIt2 时这个太慢**，建议改成 **`poll_div: 6`**（120ms 轮一圈）。

---

# 5. `CMakeLists.txt` / `package.xml`

## `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.10)

project(CanServer VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ─────────── 依赖 ───────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ─────────── 库：不依赖 ROS 的 SocketCAN + 舵机协议层 ───────────
add_library(arm_can SHARED
    src/can_init.cpp
    src/Servo_ctrl.cpp
)
target_include_directories(arm_can PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(arm_can PUBLIC Threads::Threads)

# ─────────── 库：ros2_control 硬件接口插件 ───────────
add_library(arm_system_hardware SHARED
    src/arm_system_hardware.cpp
)
target_include_directories(arm_system_hardware PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(arm_system_hardware PUBLIC
    arm_can
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    rclcpp_lifecycle::rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(
    hardware_interface arm_system_hardware.xml)

# ─────────── 可执行文件 ───────────
add_executable(arm_server src/arm_server_node.cpp)
target_link_libraries(arm_server arm_can rclcpp::rclcpp ${sensor_msgs_TARGETS})

add_executable(can_test ./main.cpp)
target_link_libraries(can_test arm_can)

# ─────────── 安装 ───────────
# 可执行文件
install(TARGETS arm_server can_test
        DESTINATION lib/${PROJECT_NAME})

# ★ 插件和它依赖的库都必须在 lib/（pluginlib 从这里加载）
install(TARGETS arm_system_hardware arm_can
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION lib)

# 头文件（给别包 include 用）
install(DIRECTORY include/
        DESTINATION include)

# ★★★ 就是这行被删了 —— launch 和 config 必须装到 share/ ★★★
install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})

ament_package()
```

## `package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>CanServer</name>
  <version>0.1.0</version>
  <description>RK3568 CAN server for the 5-DOF arm</description>
  <maintainer email="39530733@qq.com">yjml233</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>hardware_interface</depend>
  <depend>pluginlib</depend>
  <depend>rclcpp_lifecycle</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## ⚠️ 最容易犯的错：删掉已有的 install 规则

**这次踩的坑**：加硬件插件时，把原来这行删掉了：

```cmake
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
```

**后果**：
```
Error: file 'arm_control.launch.py' was not found in the share directory of package 'CanServer'
```

**自查命令**（一个 ament 包至少要这几条）：
```bash
grep -cE "install\\(TARGETS|install\\(DIRECTORY" CMakeLists.txt   # 少于 3 就要警惕
```

**完整的 install 清单应该是**：
```cmake
install(TARGETS arm_server can_test DESTINATION lib/${PROJECT_NAME})     # 可执行文件
install(TARGETS arm_system_hardware arm_can LIBRARY DESTINATION lib ...)   # 插件 + 库
install(DIRECTORY include/ DESTINATION include)                            # 头文件
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})       # ★ 别漏
pluginlib_export_plugin_description_file(hardware_interface arm_system_hardware.xml)
```

---

# 6. `config/ros2_controllers.yaml`

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50              # Hz ← 20ms 控制周期

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - gripper_finger_left_joint

    command_interfaces:
      - position
    state_interfaces:
      - position

    open_loop_control: false
    allow_partial_joints_goal: false
    allow_nonzero_velocity_at_trajectory_end: false

    constraints:
      stopped_velocity_tolerance: 0.05
      goal_time: 1.0                      # 轨迹结束后允许的稳定时间
      joint1: {trajectory: 0.15, goal: 0.10}
      joint2: {trajectory: 0.15, goal: 0.10}
      joint3: {trajectory: 0.15, goal: 0.10}
      joint4: {trajectory: 0.15, goal: 0.10}
      joint5: {trajectory: 0.15, goal: 0.10}
      gripper_finger_left_joint: {trajectory: 0.005, goal: 0.002}
```

## 参数说明

| 参数 | 说明 |
|---|---|
| `update_rate: 50` | controller_manager 周期 20ms。**A55 上 50Hz 稳妥**，比舵机响应快就够了 |
| `joints` | **必须和 xacro 里的 `<joint>` 一一对应**，顺序无所谓但名字要准 |
| `command_interfaces: [position]` | 只控位置 |
| `open_loop_control: false` | **用真实状态**（`/joint_states`）而不是期望值 |
| `allow_partial_joints_goal: false` | 必须给全部 6 个关节 |
| `constraints.*.trajectory` | 轨迹跟踪容差。**设太严会一直报 "goal not reached"**（因为 `read()` 有 120~600ms 延迟） |

---

# 7. `launch/arm_control.launch.py`

```python
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ① 展开 URDF（含 <ros2_control> 段）
    xacro_file = os.path.join(
        get_package_share_directory('my_robot_arm_description'),
        'urdf', 'robot_arm.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # ② 控制器配置
    ctrl_yaml = os.path.join(
        get_package_share_directory('CanServer'),
        'config', 'ros2_controllers.yaml')

    return LaunchDescription([
        # robot_state_publisher：发 TF + /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen',
        ),

        # ★ ros2_control_node：加载硬件接口 + 控制器管理器
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_desc}, ctrl_yaml],
            output='screen',
        ),

        # 加载 joint_state_broadcaster（把 state_interface 发成 /joint_states）
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
            output='screen',
        ),

        # 加载 joint_trajectory_controller（接收轨迹并驱动硬件）
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller', '-c', '/controller_manager'],
            output='screen',
        ),
    ])
```

## 四个节点的作用

| 节点 | 作用 | 能省吗 |
|---|---|---|
| `robot_state_publisher` | URDF → TF + `/robot_description` | ❌ 必须 |
| `ros2_control_node` | controller_manager + 加载硬件插件 | ❌ 必须 |
| `spawner joint_state_broadcaster` | state_interface → `/joint_states` | ❌ 必须（否则 MoveIt2 读不到状态） |
| `spawner arm_controller` | 接收轨迹并驱动硬件 | ❌ 必须 |

---

# 8. 编译与启动验证

## 编译

```bash
cd ~/ros2/ros2_ws
colcon build --packages-select CanServer --symlink-install
source install/setup.bash
```

## ★ 启动前的必做检查

### ① CAN 接口必须 UP（bitrate 每次上电都要重设！）

```bash
sudo ip link set can0 type can bitrate 250000 triple-sampling on
sudo ip link set can0 up

# 验证
ip -details link show can0 | grep -E "state|bitrate"
# 期望：state UP，bitrate 250000
```

**不做这一步的症状**：
```
Write : fail to write!
Position_set 失败 servo_id=0: Pos_set : CanERROR!
```

**建议做成开机自启**：
```bash
sudo tee /etc/systemd/system/can0.service > /dev/null <<'EOF'
[Unit]
Description=CAN0 interface setup
After=network.target

[Service]
Type=oneshot
ExecStart=/sbin/ip link set can0 type can bitrate 250000 triple-sampling on
ExecStart=/sbin/ip link set can0 up
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable --now can0.service
```

### ② ★ 把机械臂摆到安全姿态

**启动瞬间如果代码有问题，机械臂会猛冲**（这次踩过）。先把臂摆到不会撞到东西的位置。

## 启动

```bash
ros2 launch CanServer arm_control.launch.py
```

**成功日志**：
```
[controller_manager]: Loading hardware 'MyRobotArm'
[controller_manager]: Loaded hardware 'MyRobotArm' from plugin 'CanServer/ArmSystemHardware'
[controller_manager.hardware_component.system.MyRobotArm]: ArmSystemHardware 就绪：6 关节, iface=can0, cmd_time=150ms, poll_div=30
[controller_manager.hardware_component.system.MyRobotArm]:   [0] joint1                       servo_id=0  servo=[500,2500]  pos=[-1.5708,1.5708]  dir=+1
...
[controller_manager]: Successful initialization of hardware 'MyRobotArm'
[controller_manager.hardware_component.system.MyRobotArm]: 已激活，命令初值 = 当前关节角：
[controller_manager.hardware_component.system.MyRobotArm]:   joint1                       -0.0123 rad
...
[controller_manager.hardware_component.system.MyRobotArm]: 首次 write：仅同步初值，不下发命令
[controller_manager]: Loading controller : 'arm_controller' of type 'joint_trajectory_controller/JointTrajectoryController'
[controller_manager]: Successfully switched controllers!
[controller_manager]: Loading controller : 'joint_state_broadcaster'
[controller_manager]: Successfully switched controllers!
```

**最关键的两行**：
- `已激活，命令初值 = 当前关节角` → 下面 6 个值应该**接近机械臂的真实姿态**
- `首次 write：仅同步初值，不下发命令` → **看到这句就说明启动保护生效了**

## 验证（另开终端）

```bash
source ~/ros2/ros2_ws/install/setup.bash

# ① 控制器状态
ros2 control list_controllers
# 期望：joint_state_broadcaster  active
#       arm_controller           active

# ② 硬件接口
ros2 control list_hardware_interfaces
# 期望：6 个 command interface [available] [claimed]
#       12 个 state interface（6 position + 6 velocity）

# ③ 能读到真实位置
ros2 topic echo /joint_states --once

# ④ 频率
ros2 topic hz /joint_states
# 期望：~50 Hz

# ⑤ CAN 上真的有帧
candump can0
# 期望：
#   can0  104   [8]  01 02 00 00 00 00 00 00     ← write 发的查询
#   can0  200   [8]  01 DC 05 13 46 00 00 00     ← read 收的
#                     ↑  ↑^^^^  ↑  ↑
#                    id  pos    温 压
#                    =1  1500   19  70
```

**`pos=1500` → `servoToAngle` → `0.0 rad`** —— 和 `/joint_states` 里的值对得上就说明映射正确。

---

# 9. 发轨迹让它动

## 方式 A：脚本（推荐，自动读当前位置）

`~/send_traj.py`：

```python
#!/usr/bin/env python3
"""发关节轨迹：  ./send_traj.py j1 j2 j3 j4 j5 gripper [时长秒]"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5',
          'gripper_finger_left_joint']


def main():
    target = [float(x) for x in sys.argv[1:7]]
    dur = float(sys.argv[7]) if len(sys.argv) > 7 else 3.0

    rclpy.init()
    node = Node('send_traj')

    cur = None

    def cb(msg):
        nonlocal cur
        try:
            cur = [msg.position[msg.name.index(j)] for j in JOINTS]
        except (ValueError, IndexError):
            pass

    node.create_subscription(JointState, '/joint_states', cb, 10)
    while cur is None:
        rclpy.spin_once(node, timeout_sec=0.5)
    print(f"当前位置: {['%.4f' % v for v in cur]}")

    cli = ActionClient(node, FollowJointTrajectory,
                       '/arm_controller/follow_joint_trajectory')
    if not cli.wait_for_server(timeout_sec=5.0):
        print("❌ 找不到 action server")
        return

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    goal.trajectory.points = [
        JointTrajectoryPoint(positions=cur,
                             time_from_start=Duration(sec=0, nanosec=0)),
        JointTrajectoryPoint(positions=target,
                             time_from_start=Duration(
                                 sec=int(dur),
                                 nanosec=int((dur % 1) * 1e9))),
    ]

    print(f"目标位置: {['%.4f' % v for v in target]}  时长 {dur}s")
    fut = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut)
    gh = fut.result()
    if not gh.accepted:
        print("❌ 目标被拒绝")
        return
    print("✅ 已接受，执行中…")

    res_fut = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut)
    print(f"结果: error_code = {res_fut.result().result.error_code}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**用法**：

```bash
chmod +x ~/send_traj.py

python3 ~/send_traj.py 0.3 0 0 0 0 0 3     # joint1 → 0.3 rad（先小幅试！）
python3 ~/send_traj.py 0 0 0 0 0 0 3       # 全部回中位
python3 ~/send_traj.py 0 0.3 0 0 0 0 3     # joint2
```

**⚠️ 第一次一定只用 0.2~0.3 rad** —— 确认方向对了再加大。

## 方式 B：命令行单点轨迹

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \\
  control_msgs/action/FollowJointTrajectory \\
  "{trajectory: {joint_names: ['joint1','joint2','joint3','joint4','joint5','gripper_finger_left_joint'],
     points: [{positions: [0.3, 0.0, 0.0, 0.0, 0.0, 0.0],
               time_from_start: {sec: 3, nanosec: 0}}]}}"
```

**如果报 "Trajectory is not started from current state"** → 用方式 A。

---

# 10. 踩过的坑（6 个）

## 🔴 坑 1：`install(DIRECTORY ...)` 被删 → launch 找不到

**症状**：
```
file 'arm_control.launch.py' was not found in the share directory of package 'CanServer'
```

**根因**：改 `CMakeLists.txt` 加插件时，把 `install(DIRECTORY launch config ...)` 删了。

**修复**：加回去（见第 5 节）。

**自查**：
```bash
find install/CanServer/share/CanServer -name "*.py" -o -name "*.yaml"
# 应该有 launch/ 和 config/ 下的文件
```

## 🔴 坑 2：启动瞬间机械臂猛冲（**最危险**）

**症状**：`ros2 launch` 时 6 个舵机同时冲向端点，然后回位。

**根因链**：
```
on_activate 里 Info_wait(10) 第一帧没回来就 break
        ↓
Info[] 缓存还是默认值（position = 0）
        ↓
servoToAngle(i, 0) 算出【越界】角度（-2.36 / +2.36 / -4.71）
        ↓
hw_commands_[i] = 越界值 且 cmd_prev_[i] = 同样的越界值   ← 都没 clamp
        ↓
write() 里 cmd = clamp(越界值) → 端点，但 cmd_prev_ 还是越界值
        ↓
|端点 - 越界值| = 0.79~1.57 >> DEADBAND(0.005) → 判定「有变化」
        ↓
★ 全部下发 → 猛冲 ★
```

**四道防线（都已加上）**：

```cpp
// 防线① servoToAngle 里 clamp
double t = (pos - servo_min_[i]) / sp;
t = std::clamp(t, 0.0, 1.0);          // ★
if (dir_[i] < 0.0) t = 1.0 - t;
return pos_min_[i] + t * ap;

// 防线② on_activate 按【时间】等，不按次数，不提前 break
const auto deadline = std::chrono::steady_clock::now() + 1000ms;
int received = 0;
while (received < n && std::chrono::steady_clock::now() < deadline)
    if (servo_->Info_wait(20)) ++received;

// 防线③ 读到的位置必须 clamp，且 cmd_prev_ 用同一个口径
double pos = servoToAngle(i, s.position);
pos = std::clamp(pos, pos_min_[i], pos_max_[i]);   // ★
hw_commands_[i] = pos;
cmd_prev_[i]    = pos;                              // ★ 一致

// 防线④ write() 第一次不下发（最保险的一道闸）
if (first_write_) {
    for (size_t i = 0; i < n; ++i)
        cmd_prev_[i] = std::clamp(hw_commands_[i], pos_min_[i], pos_max_[i]);
    first_write_ = false;
    RCLCPP_INFO(get_logger(), "首次 write：仅同步初值，不下发命令");
    return hardware_interface::return_type::OK;
}
```

**教训**：
> **任何硬件接口在启动瞬间都不该发命令。**
> **两处 clamp 的口径必须一致** —— 一处 clamp 一处不 clamp，差值就会被误判为「有变化」。

## 🔴 坑 3：`can0` 没 UP → 全部 Write 失败

**症状**：
```
Write : fail to write!
Position_set 失败 servo_id=0: Pos_set : CanERROR!
```

**根因**：`if_nametoindex()` 只看**接口存在**，不看 **UP/DOWN**。所以 `on_init` 会"成功"，但所有写操作都失败。

**修复**：
```bash
sudo ip link set can0 type can bitrate 250000 triple-sampling on
sudo ip link set can0 up
```

**建议**：在 `on_init` 里加接口状态检查（见第 12 节的加固项）。

## 🟡 坑 4：`Info_wait(0)` 刷屏日志

`read()` 每周期非阻塞探测，而 `Info_wait` 失败时如果打日志就会每 20ms 刷一条。

**修复**（`Servo_ctrl.cpp`）：
```cpp
if (!can.Can_Read(frame, time_ms)) {
    if (time_ms > 0) err_ = can.Can_geterr();   // ← 非阻塞探测不打日志
    return false;
}
```

## 🟡 坑 5：头文件扩展名 `.h` vs `.hpp` 不一致

**症状**：`#include "can_init.h"` 找不到文件。

**根因**：`include/` 里被误改名成 `.hpp`，但源码还引 `.h`。

**修复**：
```bash
cd include
for f in can_init Servo_ctrl myqueue app; do
  [ -f "$f.hpp" ] && mv -v "$f.hpp" "$f.h"
done
```

## 🟡 坑 6：`can_server/` 和 root 双份源码不同步

**症状**：改了 root 的文件，`colcon` 编的还是旧的。

**根因**：`ros2_ws/src/CanServer` 当时软链接到 `can_server/`，而开发改的是 root。

**修复**（根治）：
```bash
cd ~/ros2/ros2_ws/src
rm CanServer
ln -s /home/kickpi/Project/CanServer CanServer      # 指向 root
touch /home/kickpi/Project/CanServer/can_server/COLCON_IGNORE   # 避免同名包冲突
```

**这样**：
- 开发 → 改 root，colcon 直接编
- 推送 → 跑 `push.sh` 把 root rsync 到 `can_server/`，再 git push

---

# 第二部分 · 接 MoveIt2（规划）

# 11. MoveIt2 与 ros2_control 的分工

| 层 | 职责 | 你现在的状态 |
|---|---|---|
| **MoveIt2** | 逆运动学、碰撞检测、路径规划、时间参数化 | ⬜ **待做** |
| **ros2_control** | 接收轨迹、按时间插值、驱动硬件 | ✅ **已通** |

**现在能做 / 不能做**：

| 能力 | 现在 |
|---|---|
| 手发关节角轨迹 → 机械臂动 | ✅ |
| **给末端三维坐标 → 自动规划过去** | ❌ 需要 MoveIt2 |
| **碰撞检测** | ❌ |
| **关节限位下的规划** | ❌（现在只有静默钳位） |

**接 MoveIt2 的本质**：填 5 个配置文件，把 `move_group` 的输出接到**你已经跑通的 `arm_controller`** 上。**硬件层一行都不用改。**

---

# 12. `moveit_config` 的 5 个文件

`my_robot_arm_moveit_config` 现在是空骨架，要填：

```
my_robot_arm_moveit_config/
├── config/
│   ├── my_robot_arm.srdf          ← ① 规划组 + 禁用碰撞对
│   ├── kinematics.yaml            ← ② IK 求解器（★ 5-DOF 要处理）
│   ├── joint_limits.yaml          ← ③ 速度/加速度限制
│   ├── moveit_controllers.yaml    ← ④ ★ 指向你的 arm_controller
│   └── ompl_planning.yaml         ← ⑤ 规划器参数
└── launch/
    └── arm_moveit_headless.launch.py   ← 不用 Setup Assistant 的 RViz 版
```

## ① `my_robot_arm.srdf`

```xml
<robot name="my_robot_arm">

  <virtual_joint name="virtual_joint" type="fixed"
                 parent_frame="world" child_link="base_footprint"/>

  <group name="arm">
    <chain base_link="base_link" tip_link="link5"/>
  </group>

  <group name="gripper">
    <joint name="gripper_finger_left_joint"/>
    <joint name="gripper_finger_right_joint"/>
  </group>

  <end_effector name="gripper_ee" parent_link="link5"
                group="gripper" parent_group="arm"/>

  <!-- ★★★ 相邻连杆必须禁用碰撞，否则规划永远失败且不告诉你为什么 ★★★ -->
  <disable_collisions link1="base_footprint"      link2="base_link"   reason="Adjacent"/>
  <disable_collisions link1="base_link"           link2="link1"       reason="Adjacent"/>
  <disable_collisions link1="link1"               link2="link2"       reason="Adjacent"/>
  <disable_collisions link1="link2"               link2="link3"       reason="Adjacent"/>
  <disable_collisions link1="link3"               link2="link4"       reason="Adjacent"/>
  <disable_collisions link1="link4"               link2="link5"       reason="Adjacent"/>
  <disable_collisions link1="link5"               link2="gripper_base" reason="Adjacent"/>
  <disable_collisions link1="gripper_base"        link2="gripper_finger_left"  reason="Adjacent"/>
  <disable_collisions link1="gripper_base"        link2="gripper_finger_right" reason="Adjacent"/>
  <disable_collisions link1="gripper_finger_left" link2="gripper_finger_right" reason="Never"/>
</robot>
```

## ② `kinematics.yaml`

```yaml
# ⚠️ 默认的 KDL 是 6-DOF 数值解，你是 5-DOF，大概率解不出来
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 3
```

**先试 KDL**，不行就上第 14 节的自定义插件。

## ③ `joint_limits.yaml`

```yaml
joint_limits:
  joint1:
    has_velocity_limits: true
    max_velocity: 1.5              # rad/s ← 先保守，实测再调
    has_acceleration_limits: true
    max_acceleration: 3.0
    has_position_limits: true
    min_position: -1.5708
    max_position:  1.5708
  # joint2 / joint3 / joint4 / joint5 同理
  gripper_finger_left_joint:
    has_velocity_limits: true
    max_velocity: 0.05             # m/s
    has_position_limits: true
    min_position: 0.0
    max_position: 0.02
```

**⚠️ 这些值 MoveIt2 会用来做时间参数化**：
- 填太大 → 舵机跟不上，执行时抖
- 填太小 → 规划出来慢得没法用

## ④ `moveit_controllers.yaml` ★ 最关键的一个

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller             # ← 就是你刚跑通的那个！
  arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - gripper_finger_left_joint
```

**这一个文件就把 MoveIt2 接到了你已验证的链路上。**

## ⑤ `ompl_planning.yaml`

```yaml
planning_plugin: ompl_interface/OMPLPlanner
request_adapters: >-
  default_planner_request_adapters/AddTimeOptimalParameterization
  default_planner_request_adapters/ResolveConstraintFrames
  default_planner_request_adapters/FixWorkspaceBounds
  default_planner_request_adapters/FixStartStateBounds
  default_planner_request_adapters/FixStartStateCollision
  default_planner_request_adapters/FixStartStatePathConstraints

planner_configs:
  RRTConnect:
    type: geometric::RRTConnect
    range: 0.0

arm:
  default_planner_config: RRTConnect
  planner_configs:
    - RRTConnect
```

**A55 上推荐 `RRTConnect`**（比 RRT* 快，5-DOF 场景够用）。

## ⑥ headless launch

**MoveIt Setup Assistant 生成的 `demo.launch.py` 会启动 RViz** —— 板子没显示屏，要写精简版：

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "my_robot_arm", package_name="my_robot_arm_moveit_config"
    ).to_moveit_configs()

    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[moveit_config.robot_description]),

        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["0", "0", "0", "0", "0", "0", "world", "base_footprint"]),

        # ★ 唯一需要的 MoveIt 节点
        Node(package="moveit_ros_move_group", executable="move_group",
             output="screen",
             parameters=[moveit_config.to_dict()]),

        # ❌ 不启动 rviz2
        # ❌ 不启动 moveit_ros_visuals
    ])
```

**内存占用**：
```
robot_state_publisher   ~30 MB
static_transform_publisher  ~15 MB
move_group              ~300~500 MB   ← 大头
────────────────────────────────────
合计                    ~350~550 MB
```

**RK3568（3.8G）够** —— 但要先关掉 VS Code Server（它吃 2GB）。

---

# 13. 5-DOF 的 IK 难题

## 问题

MoveIt2 默认用 **KDL**，它是**数值求解器，需要 6 个自由度**。

你是 5-DOF（`joint1`~`joint5`），**KDL 大概率解不出来**。

## 三个选择

| 方案 | 工作量 | 说明 |
|---|---|---|
| **A. 先试 KDL** | 10 分钟 | 万一能凑合 |
| **B. 自定义 IK 插件** ⭐ | 1.5 天 | **把手推解析逆解封装成 MoveIt2 插件** |
| **C. IKFast** | 难 | OpenRAVE 工具链老旧 |

## 为什么方案 B 值得做

**① 你的解析逆解本来就在解位置**
```
位置链：joint1（底座 yaw）+ joint2/joint3（平面 2R）  → 3 个自由度
姿态链：joint4 + joint5                              → 不参与位置
```

**② 顺便解决了现在的最大功能缺口**

现在 `angle_to_servo` 超限是**静默钳位**（机械臂走到错的地方还不报错）。
换成 IK 插件后**超限返回 `NO_IK_SOLUTION`**，MoveIt2 会据此重新规划。

**③ 简历上能写**

> **「因机械臂为 5 自由度、KDL 无法求解，将自研解析逆解封装为 MoveIt2 的 `KinematicsBase` 插件」**

**比"用了 MoveIt2"具体一个量级。**

---

# 14. 自定义 IK 插件

## 包结构

```
arm_ik_plugin/
├── CMakeLists.txt
├── package.xml
├── arm_ik_plugin.xml
├── include/arm_ik_plugin/arm_analytic_ik_plugin.hpp
└── src/arm_analytic_ik_plugin.cpp
```

## 头文件骨架

```cpp
#pragma once
#include <moveit/kinematics_base/kinematics_base.h>

namespace arm_ik_plugin {

class ArmAnalyticIKPlugin : public kinematics::KinematicsBase
{
public:
  bool initialize(const rclcpp::Node::SharedPtr & node,
                  const std::string & group_name,
                  const std::string & base_frame,
                  const std::vector<std::string> & tip_frames,
                  double search_discretization) override;

  bool getPositionIK(const geometry_msgs::msg::Pose & ik_pose,
                     const std::vector<double> & ik_seed_state,
                     std::vector<double> & solution,
                     moveit_msgs::msg::MoveItErrorCodes & error_code,
                     const kinematics::KinematicsQueryOptions & options
                       = kinematics::KinematicsQueryOptions()) const override;

private:
  // ★ 你的解析逆解
  bool analyticIK(double x, double y, double z,
                  double seed_t3, std::vector<double> & out) const;

  double L1_, L2_, Z0_;          // 0.0985 / 0.0950 / 0.0725
  std::vector<double> min_, max_;
};

}  // namespace arm_ik_plugin
```

## `getPositionIK` 核心

```cpp
bool ArmAnalyticIKPlugin::getPositionIK(
    const geometry_msgs::msg::Pose & ik_pose,
    const std::vector<double> & ik_seed_state,
    std::vector<double> & solution,
    moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions &) const
{
  const double x = ik_pose.position.x;
  const double y = ik_pose.position.y;
  const double z = ik_pose.position.z;

  std::vector<double> sol;
  if (!analyticIK(x, y, z, ik_seed_state[2], sol)) {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;                    // ★ 不可达：正确拒绝（不再是静默钳位）
  }

  // 姿态自由度沿用 seed（5-DOF 解不了完整 6D）
  sol[3] = ik_seed_state[3];
  sol[4] = ik_seed_state[4];

  // ★ 关节限位校验 —— 超限就拒绝
  for (size_t i = 0; i < sol.size(); ++i) {
    if (sol[i] < min_[i] || sol[i] > max_[i]) {
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
      return false;
    }
  }

  solution = sol;
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return true;
}
```

## 插件声明 `arm_ik_plugin.xml`

```xml
<library path="arm_ik_plugin">
  <class name="arm_ik_plugin/ArmAnalyticIKPlugin"
         type="arm_ik_plugin::ArmAnalyticIKPlugin"
         base_class_type="kinematics::KinematicsBase">
    <description>Analytic IK for 5-DOF arm (base yaw + planar 2R)</description>
  </class>
</library>
```

## 接到 `kinematics.yaml`

```yaml
arm:
  kinematics_solver: arm_ik_plugin/ArmAnalyticIKPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 3
```

---

# 15. 端到端

## 完整 launch

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "my_robot_arm", package_name="my_robot_arm_moveit_config"
    ).to_moveit_configs()

    return LaunchDescription([
        # ── 硬件层（你已有的）──
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[moveit_config.robot_description,
                         os.path.join(..., "ros2_controllers.yaml")]),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster"]),
        Node(package="controller_manager", executable="spawner",
             arguments=["arm_controller"]),

        # ── 规划层 ──
        Node(package="moveit_ros_move_group", executable="move_group",
             parameters=[moveit_config.to_dict()]),
    ])
```

## 验证顺序（一步步，别跳）

```
① 只起 hardware_interface → 确认 on_init 日志的 6 个关节参数对    ✅ 已过
② 加 joint_state_broadcaster → /joint_states 有真实值             ✅ 已过
③ 加 arm_controller → send_traj.py 能动                          ✅ 已过
④ 加 move_group → ros2 node list | grep move_group               ⬜
⑤ 测 KDL 能不能解 5-DOF                                          ⬜
⑥ 不能解 → 写 arm_ik_plugin                                      ⬜
⑦ 用 moveit_py 发一个末端位姿目标 → 看能不能规划并执行             ⬜
```

## 用 `moveit_py` 发末端目标

```python
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

moveit = MoveItPy(node_name="moveit_py")
arm = moveit.get_planning_component("arm")
arm.set_goal_state(pose_stamped_msg=target_pose, pose_link="link5")

plan_result = arm.plan()
if plan_result:
    moveit.execute(plan_result.trajectory, controllers=[])
```

---

# 附录 A · 命令速查

```bash
# ── 环境 ──
source /opt/ros/jazzy/setup.bash
source ~/ros2/ros2_ws/install/setup.bash

# ── CAN ──
sudo ip link set can0 type can bitrate 250000 triple-sampling on
sudo ip link set can0 up
ip -details link show can0
candump can0
cansend can0 104#0102000000000000

# ── 编译 ──
cd ~/ros2/ros2_ws
colcon build --packages-select CanServer --symlink-install
colcon list

# ── 启动 ──
ros2 launch CanServer arm_control.launch.py

# ── 验证 ──
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states --once
ros2 topic hz /joint_states
ros2 node list | grep move_group

# ── 发轨迹 ──
python3 ~/send_traj.py 0.3 0 0 0 0 0 3

# ── 清理重编 ──
rm -rf build install log && colcon build --symlink-install
```

---

# 附录 B · 排查表

| 症状 | 大概率原因 | 解决 |
|---|---|---|
| `file 'xxx.launch.py' was not found in share` | `CMakeLists` 缺 `install(DIRECTORY launch config ...)` | 加回去，重编 |
| `Write : fail to write!` | `can0` 没 UP | `ip link set can0 up` |
| `Position_set 失败 ... CanERROR!` | 同上 | 同上 |
| **启动瞬间机械臂猛冲** | `on_activate` 读到无效值 + 没 clamp + 首次 write 就下发 | 加四道防线（坑 2） |
| `Failed to load plugin 'CanServer/ArmSystemHardware'` | 插件 `.so` 不在 `lib/`，或 xml 的 `name` 不匹配 | 查 `install/CanServer/lib/` 和 xml |
| `Package 'CanServer' not found` | 缺 `package.xml` 或没 source | `colcon list` 确认 |
| `XML or text declaration not at start of entity` | xacro 第 1 行有前导空格 | `sed -i '1s/^[[:space:]]*//'` |
| `Duplicate package names not supported` | root 和 `can_server/` 都有 package.xml | 给一个加 `COLCON_IGNORE` |
| 机械臂不动，也无报错 | 轨迹容差太严，JTC 提前判定完成 | 调大 `constraints` |
| `goal not reached` | `read()` 刷新太慢（`poll_div` 太大） | `poll_div: 6` |
| MoveIt2 规划永远失败 | SRDF 缺 `disable_collisions` | 补齐所有相邻连杆对 |

---

# 进展追踪

```
✅ 阶段 0  环境（ROS 2 Jazzy + MoveIt2 28 包 + ros2_control）
✅ 阶段 1  CanServer 编译（含 ArmSystemHardware 插件）
✅ 阶段 2  robot_arm.ros2_control.xacro 能展开
✅ 阶段 3  ros2_controllers.yaml + arm_control.launch.py
✅ 阶段 4  CAN → 舵机 全链路验证（50Hz，读到真实位置）
✅ 阶段 5  发轨迹让实机动 + 启动猛冲 bug 修复
⬜ 阶段 6  填 my_robot_arm_moveit_config（5 个文件）
⬜ 阶段 7  测 KDL 能否解 5-DOF
⬜ 阶段 8  写 arm_ik_plugin（如需）
⬜ 阶段 9  端到端：末端位姿 → 规划 → 执行
```