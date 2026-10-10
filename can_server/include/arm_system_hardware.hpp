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
  // cmd_time_ms_：舵机到位时间的【兜底值】。
  //   正常运行时 write() 用控制周期（period）作为到位时间，
  //   这个值只在首次调用（period 为 0）时使用。
  int                  cmd_time_ms_   = 150;
  // send_div_：每 N 个控制周期才【下发一次 CAN 命令】。
  //
  //   ★ 这是「一卡一卡」的关键参数 ★
  //
  //   舵机协议 #P%04dT%04d! 的语义是「在 T 毫秒内从当前位置走到 P」——
  //   每次收到新命令都会从当前位置【重新开始】这段行程。
  //
  //   每 20ms 发一次、T=20ms 时，舵机每 20ms 就因新命令被打断，
  //   永远停在加速阶段，从没走完一段完整行程 → 明显一卡一卡。
  //
  //   改成每 send_div_×20ms 发一次、T 同样取 send_div_×20ms，
  //   舵机每次都能走完完整行程 → 顺滑。
  //
  //   取值：3→60ms，5→100ms，10→200ms。
  //         越小跟得越准但越卡；越大越顺滑但跟随越滞后。
  int                  send_div_      = 5;

  // poll_div_：每 N 个控制周期查询一个舵机。
  //   全轴刷新 = poll_div × N_joints / update_rate。
  //   ⚠️ 对 trajectory_controller 来说 > 0.5s 就太慢了，
  //   推荐 2（=240ms @50Hz）。
  int                  poll_div_      = 2;
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
  size_t send_tick_ = 0;     // 控制周期计数（write 用）
  // send_age_：距上次【真正下发 CAN 命令】过了多少个控制周期。
  //   ★ 用来判断舵机是否正在运动 ★
  //   ZX361S 是半双工总线舵机，任何查询（PRAD/PRTE/PRTV）都会
  //   【打断】正在执行的 P...T... 行程。所以运动期间不能查询。
  size_t send_age_  = 9999;
  size_t next_id_  = 0;      // 轮询到哪个舵机
  double cmd_prev_[8] = {0}; // 上次下发的命令（死区用）
  bool first_write_ = true;

  /// 读数合法性：超出舵机标定范围的一律无效（STM32 查询失败会返回 0）
  bool     validReading(size_t i, uint16_t pos) const;

  double   servoToAngle(size_t i, uint16_t pos) const;
  uint16_t angleToServo(size_t i, double a) const;

};

}  // namespace arm_hardware


