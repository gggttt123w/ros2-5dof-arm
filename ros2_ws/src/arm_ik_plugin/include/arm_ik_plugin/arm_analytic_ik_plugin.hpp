#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>

namespace arm_ik_plugin
{

/**
 * @brief 5 自由度机械臂的解析逆解插件
 *
 * KDL 是 6 自由度数值解，对本机械臂（5 个转动关节）从随机种子出发
 * 无法收敛，表现为 MoveIt 报：
 *     RRTConnect: Unable to sample any valid states for goal tree
 *
 * 本插件直接用解析法求位置逆解：
 *   · 只保证【位置】(x,y,z) —— 这正是 5-DOF 能完全控制的部分
 *   · 姿态不参与求解（末端 roll 由 joint5 提供，pitch/yaw 由 j2~j4 决定）
 *   · 解出来先做关节限位检查，再用正解自校验
 */
class ArmAnalyticIKPlugin : public kinematics::KinematicsBase
{
public:
  ArmAnalyticIKPlugin() = default;

  bool initialize(const rclcpp::Node::SharedPtr & node,
                  const moveit::core::RobotModel & robot_model,
                  const std::string & group_name,
                  const std::string & base_frame,
                  const std::vector<std::string> & tip_frames,
                  double search_discretization) override;

  // ── 必需的 7 个纯虚函数 ──

  bool getPositionIK(const geometry_msgs::msg::Pose & ik_pose,
                     const std::vector<double> & ik_seed_state,
                     std::vector<double> & solution,
                     moveit_msgs::msg::MoveItErrorCodes & error_code,
                     const kinematics::KinematicsQueryOptions & options =
                       kinematics::KinematicsQueryOptions()) const override;

  using kinematics::KinematicsBase::searchPositionIK;

  bool searchPositionIK(const geometry_msgs::msg::Pose & ik_pose,
                        const std::vector<double> & ik_seed_state,
                        double timeout,
                        std::vector<double> & solution,
                        moveit_msgs::msg::MoveItErrorCodes & error_code,
                        const kinematics::KinematicsQueryOptions & options =
                          kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose & ik_pose,
                        const std::vector<double> & ik_seed_state,
                        double timeout,
                        const std::vector<double> & consistency_limits,
                        std::vector<double> & solution,
                        moveit_msgs::msg::MoveItErrorCodes & error_code,
                        const kinematics::KinematicsQueryOptions & options =
                          kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose & ik_pose,
                        const std::vector<double> & ik_seed_state,
                        double timeout,
                        std::vector<double> & solution,
                        const IKCallbackFn & solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes & error_code,
                        const kinematics::KinematicsQueryOptions & options =
                          kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose & ik_pose,
                        const std::vector<double> & ik_seed_state,
                        double timeout,
                        const std::vector<double> & consistency_limits,
                        std::vector<double> & solution,
                        const IKCallbackFn & solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes & error_code,
                        const kinematics::KinematicsQueryOptions & options =
                          kinematics::KinematicsQueryOptions()) const override;

  bool getPositionFK(const std::vector<std::string> & link_names,
                     const std::vector<double> & joint_angles,
                     std::vector<geometry_msgs::msg::Pose> & poses) const override;

  const std::vector<std::string> & getJointNames() const override
  {
    return joint_names_;
  }

  const std::vector<std::string> & getLinkNames() const override
  {
    return link_names_;
  }

private:
  /// 给定腕部绝对角 a4 和肘部构型，解一组关节角；不可达时 q 为空
  void solveArm(double X, double Y, double Z, double a4, int elbow,
                const std::vector<double> & seed,
                std::vector<double> & q) const;

  /// ★ 枚举【全部】可行解析解，按离种子的距离升序排列。
  ///   带碰撞回调的 searchPositionIK 会逐个试，谁通过用谁 ——
  ///   否则只返回「离种子最近」的那组，很容易被否掉，
  ///   OMPL 就报 Unable to sample any valid states for goal tree。
  void solveAll(double X, double Y, double Z,
                const std::vector<double> & seed,
                std::vector<std::vector<double>> & out) const;

  /// 正解（位置）—— 只用来自校验
  void forwardArm(const std::vector<double> & q,
                  double & X, double & Y, double & Z) const;

  bool withinLimits(const std::vector<double> & q) const;

  static double squaredDistance(const std::vector<double> & a,
                                const std::vector<double> & b);

  // ── 几何常量（单位 m，从 URDF 量得并经 /compute_fk 验证）──
  static constexpr double L1 = 0.0985;   // j2 -> j3
  static constexpr double L2 = 0.0950;   // j3 -> j4
  static constexpr double L3 = 0.0685;   // j4 -> link5 原点
  static constexpr double Z0 = 0.0725;   // base -> j2 轴线

  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;
};

}  // namespace arm_ik_plugin
