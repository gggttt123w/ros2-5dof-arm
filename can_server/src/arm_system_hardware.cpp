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
  if (getParam(info_, "send_div",     v)) send_div_      = std::max(1, std::stoi(v));

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
  RCLCPP_INFO(get_logger(),
              "  下发: 每 %d 个控制周期一次（≈%.0f ms），舵机到位时间跟随该间隔",
              send_div_, send_div_ * 20.0);
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
    double pos = validReading(i, s.position)
                     ? servoToAngle(i, s.position)
                     : 0.5 * (pos_min_[i] + pos_max_[i]);  // 无效则取中点
    pos = std::clamp(pos, pos_min_[i], pos_max_[i]);
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

  // ★★★ 运动期间【不要查询舵机】—— 这是「一卡一卡」的真正根因 ★★★
  //
  //   ZX361S 是半双工总线舵机。实测（绕开 ros2_control，裸 CAN 直接测）：
  //       完全不开查询       → 3 秒行程正常走完   ✅
  //       5 / 20 / 50 Hz 查询 → 全部停在原地      ❌
  //
  //   即：任何查询（#P%03dPRAD! / PRTE / PRTV）都会【打断】正在执行的
  //   P...T... 行程。于是：
  //       read() 查询 → 打断 write() 刚下发的运动 → 舵机停住
  //       → 下一周期 JTC 发现没动又发 → 又被下一次查询打断
  //       → 舵机走走停停 = 一卡一卡
  //
  //   正确做法：正在运动时用【命令值】当状态（舵机一定会走到），
  //             等它停下来之后再查一次真实位置做校准。
  ++send_age_;
  const size_t moving_win = static_cast<size_t>(send_div_) * 6;

  if (send_age_ < moving_win)
  {
    // 运动期间：用命令值当状态
    for (size_t i = 0; i < n; ++i)
    {
      // 来回过一次舵机量化，让报出来的值和真实分辨率一致
      hw_positions_[i] = servoToAngle(i, angleToServo(i, hw_commands_[i]));
    }
    return hardware_interface::return_type::OK;
  }

  if (send_age_ == moving_win)
  {
    // ★★★ 刚停下来这一拍：先把 6 个舵机【全部】查一遍 ★★★
    //
    //   运动期间没有查询，Info 缓存里还是【运动之前】的旧位置。
    //   如果直接按 poll_div_ 轮询，要 6 个周期才轮完，前几拍读到的
    //   是过期值 —— 实测会造成 hw_positions_ 突跳 1.12 rad，于是：
    //       State tolerances failed for joint 2:
    //       Position Error: -1.122340, Position Tolerance: 1.000000
    //       arm_controller: Aborted due to state tolerance violation
    //
    //   所以这里一次性刷新全部，把过期缓存冲掉，再填状态。
    for (size_t i = 0; i < n; ++i)
    {
      servo_->Statue_get(servo_ids_[i]);
    }
    for (int k = 0; k < 80; ++k)      // 最多收 80 帧
    {
      if (!servo_->Info_wait(10))
      {
        break;
      }
    }
    for (size_t i = 0; i < n; ++i)
    {
      const ServoStatus s = servo_->Read_Info(servo_ids_[i]);
      if (validReading(i, s.position))
      {
        hw_positions_[i] = servoToAngle(i, s.position);
      }
      // 无效读数（STM32 查询超时会回 0）：保留上一次的值
    }
    return hardware_interface::return_type::OK;
  }

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
  //    ★ 无效读数保护 ★
  //    STM32 的 Servo_position_get 在查询超时时返回 0（见 robot_arm.c），
  //    而 0 远在舵机标定范围之外。直接用会把状态打飞，触发：
  //        State tolerances failed for joint 0
  //        arm_controller: Aborted due to state tolerance violation
  //    所以超出标定范围的读数一律丢弃，保留上一次的值。
  for (size_t i = 0; i < n; ++i) {
    const ServoStatus s = servo_->Read_Info(servo_ids_[i]);
    if (validReading(i, s.position))
    {
      hw_positions_[i] = servoToAngle(i, s.position);
    }
  }
  return hardware_interface::return_type::OK;
}

// ─────────── ★ write：把命令下发到 CAN ───────────
hardware_interface::return_type ArmSystemHardware::write(
    const rclcpp::Time &, const rclcpp::Duration & period)
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
  // ★★★ 舵机的「到位时间」必须 ≈ 控制周期 ★★★
  //
  // 舵机收到 Position_set(id, pos, T) 时，会从【当前实际位置】
  // 重新开始 T 毫秒的行程。如果 T 远大于控制周期，舵机每个
  // 周期只能走 Δ×(period/T)，实际速度被压到指令速度的
  // period/T 倍 —— 表现为 JTC 报 PATH_TOLERANCE_VIOLATED(-4)。
  //
  // 所以这里用【控制周期】作为到位时间，cmd_time_ms 只做兜底
  // （首次调用 period 可能为 0）。
  double period_ms = period.seconds() * 1000.0;
  if (period_ms < 1.0)
  {
    period_ms = 20.0;  // 兜底：假设 50 Hz
  }

  // ★★★ 降频下发 —— 解决「一卡一卡」的关键 ★★★
  //
  //   舵机协议 #P%04dT%04d! 是「在 T 毫秒内从当前位置走到 P」，
  //   每收到新命令都会从当前位置【重新开始】这段行程。
  //
  //   每 20ms 发一次、T=20ms：舵机每 20ms 被打断一次，永远停在
  //   加速阶段 → 一卡一卡，平均速度也上不去。
  //
  //   每 send_div_×20ms 发一次、T=send_div_×20ms：舵机每次都能
  //   走完一段完整行程 → 顺滑。
  //
  //   代价：跟随滞后最多一个下发间隔（send_div_=5 时 100ms）。
  //         对舵机这种执行器，顺滑远比 100ms 的跟随误差重要。
  if (send_div_ > 1 &&
      (++send_tick_ % static_cast<size_t>(send_div_)) != 0)
  {
    return hardware_interface::return_type::OK;
  }

  const uint16_t t_ms = static_cast<uint16_t>(
      std::clamp(period_ms * static_cast<double>(send_div_), 20.0, 400.0));

  constexpr double DEADBAND = 0.005;
  for (size_t i = 0; i < n; ++i) {
    double cmd = hw_commands_[i];
    // 钳到关节限位（MoveIt2 规划过，但这里再兜一层）
    cmd = std::clamp(cmd, pos_min_[i], pos_max_[i]);

    if (std::fabs(cmd - cmd_prev_[i]) < DEADBAND) continue;

    const uint16_t pos = angleToServo(i, cmd);
    if (!servo_->Position_set(servo_ids_[i], pos, t_ms)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Position_set 失败 servo_id=%u: %s",
                           servo_ids_[i], servo_->Servo_errget().c_str());
    }
    cmd_prev_[i] = cmd;
    send_age_ = 0;   // ★ 刚下发过 → read() 接下来不再查询，避免打断
  }
  return hardware_interface::return_type::OK;
}

// ─────────── 读数合法性 ───────────
bool ArmSystemHardware::validReading(size_t i, uint16_t pos) const
{
  // STM32 查询失败时会回 0，超出标定范围的一律当无效
  return pos >= servo_min_[i] && pos <= servo_max_[i];
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


