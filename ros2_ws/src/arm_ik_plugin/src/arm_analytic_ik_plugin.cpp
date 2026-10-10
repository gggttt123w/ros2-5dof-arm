#include "arm_ik_plugin/arm_analytic_ik_plugin.hpp"

#include <algorithm>
#include <cmath>

#include <pluginlib/class_list_macros.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace arm_ik_plugin
{

namespace
{
constexpr double kSelfCheckTol = 1e-6;   // 正解自校验容差 (m)
constexpr double kSweepStep = M_PI / 30.0;  // 腕部绝对角扫描步长 = 6°
constexpr int    kSweepHalf = 30;           // ±30 步 → ±180°
}  // namespace

// ─────────── 工具 ───────────
double ArmAnalyticIKPlugin::squaredDistance(const std::vector<double> & a,
                                            const std::vector<double> & b)
{
  double s = 0.0;
  const size_t n = std::min(a.size(), b.size());
  for (size_t i = 0; i < n; ++i)
  {
    const double d = a[i] - b[i];
    s += d * d;
  }
  return s;
}

// ─────────── initialize ───────────
bool ArmAnalyticIKPlugin::initialize(
    const rclcpp::Node::SharedPtr & node,
    const moveit::core::RobotModel & robot_model,
    const std::string & group_name,
    const std::string & base_frame,
    const std::vector<std::string> & tip_frames,
    double search_discretization)
{
  // ⚠️ 基类 initialize 之前 node_ 还是空的，必须用参数 node 打日志
  RCLCPP_INFO(node->get_logger(),
              "ArmAnalyticIKPlugin::initialize: group=%s base=%s tips=%zu",
              group_name.c_str(), base_frame.c_str(), tip_frames.size());

  // ★★★ 这里【不能】调用 kinematics::KinematicsBase::initialize()！★★★
  //
  //   MoveIt 2.12 中该基类方法的默认实现只是个「你必须 override」的桩，
  //   一旦被调用就报：
  //       IK plugin for group 'arm' relies on deprecated API.
  //       Please implement initialize(rclcpp::Node::SharedPtr, RobotModel, ...).
  //   并返回 false —— 插件就永远加载不上。
  //
  //   正确做法：自己设置 node_，再调用 protected 的 storeValues()
  //   （它负责填 robot_model_ / group_name_ / base_frame_ / tip_frames_）。
  node_ = node;
  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);

  if (!robot_model_)
  {
    RCLCPP_ERROR(node_->get_logger(), "  robot_model_ 为空 —— storeValues 没填上");
    return false;
  }

  const moveit::core::JointModelGroup * jmg =
      robot_model_->getJointModelGroup(group_name_);
  if (!jmg)
  {
    RCLCPP_ERROR(node_->get_logger(), "  找不到规划组 '%s'", group_name_.c_str());
    return false;
  }

  joint_names_ = jmg->getVariableNames();
  link_names_ = jmg->getLinkModelNames();

  if (joint_names_.size() != 5)
  {
    RCLCPP_WARN(node_->get_logger(),
                "  组 '%s' 有 %zu 个关节（本插件为 5 关节机械臂设计）——"
                "该组求不了逆解，但初始化照常成功，以免拖垮别的组。",
                group_name_.c_str(), joint_names_.size());
    return true;
  }

  RCLCPP_INFO(node_->get_logger(),
              "ArmAnalyticIKPlugin 就绪：组=%s base=%s tip=%s 关节=%zu",
              group_name_.c_str(), base_frame_.c_str(),
              tip_frames_.empty() ? "?" : tip_frames_[0].c_str(),
              joint_names_.size());
  RCLCPP_INFO(node_->get_logger(),
              "  几何: L1=%.4f L2=%.4f L3=%.4f Z0=%.4f  "
              "扫描 %d 个腕部角 ×2 肘部构型",
              L1, L2, L3, Z0, 2 * kSweepHalf + 1);
  return true;
}

// ─────────── 正解（位置）── 与 URDF 逐点验证过，误差 0.000000 ───────────
void ArmAnalyticIKPlugin::forwardArm(const std::vector<double> & q,
                                     double & X, double & Y, double & Z) const
{
  const double a2 = q[1];
  const double a3 = q[1] + q[2];
  const double a4 = q[1] + q[2] + q[3];
  const double r = L1 * std::sin(a2) + L2 * std::sin(a3) + L3 * std::sin(a4);
  Z = Z0 + L1 * std::cos(a2) + L2 * std::cos(a3) + L3 * std::cos(a4);
  X = r * std::cos(q[0]);
  Y = r * std::sin(q[0]);
}

// ─────────── 关节限位 ───────────
bool ArmAnalyticIKPlugin::withinLimits(const std::vector<double> & q) const
{
  if (!robot_model_)
  {
    // 退路：本机械臂的硬编码限位
    static const double lo[5] = {-1.5708, -1.5708, -1.5708, -1.5708, -3.1416};
    static const double hi[5] = {1.5708, 1.5708, 1.5708, 1.5708, 3.1416};
    for (size_t i = 0; i < q.size() && i < 5; ++i)
    {
      if (q[i] < lo[i] - 1e-9 || q[i] > hi[i] + 1e-9)
      {
        return false;
      }
    }
    return true;
  }

  for (size_t i = 0; i < q.size() && i < joint_names_.size(); ++i)
  {
    const auto & b = robot_model_->getVariableBounds(joint_names_[i]);
    if (!b.position_bounded_)
    {
      continue;
    }
    if (q[i] < b.min_position_ - 1e-9 || q[i] > b.max_position_ + 1e-9)
    {
      return false;
    }
  }
  return true;
}

// ─────────── 解析逆解（单组）───────────
void ArmAnalyticIKPlugin::solveArm(double X, double Y, double Z,
                                   double a4, int elbow,
                                   const std::vector<double> & seed,
                                   std::vector<double> & q) const
{
  q.clear();

  const double q1 = std::atan2(Y, X);
  const double r = std::hypot(X, Y);

  // 把腕部那段 L3（方向 a4）减掉，得到 j4 轴线该在的位置
  const double rp = r - L3 * std::sin(a4);
  const double zp = (Z - Z0) - L3 * std::cos(a4);

  const double d2 = rp * rp + zp * zp;

  // ★★★ 注意：a2 / a3 是【绝对角】（相对 +z 轴），不是相对角！★★★
  //
  //   位置展开：r = L1 sin(a2) + L2 sin(a3)
  //             z = L1 cos(a2) + L2 cos(a3)
  //   平方相加：d² = L1² + L2² + 2 L1 L2 cos(a3 - a2)
  //
  //   所以余弦定理给出的是 cos(a3 - a2)，【不是】cos(a3)。
  //   （一开始写成 cos(a3)，正解自校验就把它挡下来了，误差 ~1e-2。）
  const double c = (d2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);  // = cos(a3 - a2)
  if (std::abs(c) > 1.0)
  {
    return;  // 该 a4 下不可达
  }

  const double dtheta = static_cast<double>(elbow) *
                        std::acos(std::clamp(c, -1.0, 1.0));  // = a3 - a2 = q3
  const double a2 = std::atan2(rp, zp) -
                    std::atan2(L2 * std::sin(dtheta), L1 + L2 * std::cos(dtheta));
  const double a3 = a2 + dtheta;

  const double q5 = seed.size() > 4 ? seed[4] : 0.0;  // roll 不影响位置，沿用种子
  q = {q1, a2, dtheta, a4 - a3, q5};
}

// ─────────── 解析逆解（全部可行解，按离种子的距离排序）───────────
void ArmAnalyticIKPlugin::solveAll(double X, double Y, double Z,
                                   const std::vector<double> & seed,
                                   std::vector<std::vector<double>> & out) const
{
  out.clear();

  // 腕部绝对角候选：种子附近优先，然后 ±180° 扫一遍
  const double seed_a4 = seed[1] + seed[2] + seed[3];

  auto tryOne = [&](double a4, int elbow)
  {
    std::vector<double> q;
    solveArm(X, Y, Z, a4, elbow, seed, q);
    if (q.size() != 5)
    {
      return;
    }
    if (!withinLimits(q))
    {
      return;
    }

    // ★ 自校验：用正解确认真的到位（防公式 / 符号错误）
    double fx = 0.0, fy = 0.0, fz = 0.0;
    forwardArm(q, fx, fy, fz);
    const double err = std::sqrt((fx - X) * (fx - X) +
                                 (fy - Y) * (fy - Y) +
                                 (fz - Z) * (fz - Z));
    if (err > kSelfCheckTol)
    {
      return;
    }
    out.push_back(q);
  };

  tryOne(seed_a4, +1);
  tryOne(seed_a4, -1);
  for (int i = -kSweepHalf; i <= kSweepHalf; ++i)
  {
    if (i == 0)
    {
      continue;  // 已经试过种子值
    }
    const double a4 = i * kSweepStep;
    tryOne(a4, +1);
    tryOne(a4, -1);
  }

  std::sort(out.begin(), out.end(),
            [&](const std::vector<double> & a, const std::vector<double> & b)
            {
              return squaredDistance(a, seed) < squaredDistance(b, seed);
            });
}

// ─────────── getPositionIK ───────────
bool ArmAnalyticIKPlugin::getPositionIK(
    const geometry_msgs::msg::Pose & ik_pose,
    const std::vector<double> & ik_seed_state,
    std::vector<double> & solution,
    moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions & /*options*/) const
{
  solution.clear();

  if (ik_seed_state.size() < 5)
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  std::vector<std::vector<double>> sols;
  solveAll(ik_pose.position.x, ik_pose.position.y, ik_pose.position.z,
           ik_seed_state, sols);

  if (sols.empty())
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  solution = sols.front();  // 已按离种子的距离排好序
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return true;
}

// ─────────── searchPositionIK（不带回调）───────────
bool ArmAnalyticIKPlugin::searchPositionIK(
    const geometry_msgs::msg::Pose & ik_pose,
    const std::vector<double> & ik_seed_state,
    double /*timeout*/,
    std::vector<double> & solution,
    moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions & options) const
{
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool ArmAnalyticIKPlugin::searchPositionIK(
    const geometry_msgs::msg::Pose & ik_pose,
    const std::vector<double> & ik_seed_state,
    double /*timeout*/,
    const std::vector<double> & /*consistency_limits*/,
    std::vector<double> & solution,
    moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions & options) const
{
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

// ─────────── searchPositionIK（带碰撞回调）───────────
//
// ★ 关键：目标采样器会带一个回调（通常是碰撞检查）。
//   只返回「离种子最近」的那一组解，很可能被回调否掉，
//   于是 OMPL 报：Unable to sample any valid states for goal tree。
//   所以要【把所有可行解逐个喂给回调】，谁先通过就用谁。
//   （KDL 用的是「多次随机重启」，这里是「穷举全部解析解」，更彻底。）
//
bool ArmAnalyticIKPlugin::searchPositionIK(
    const geometry_msgs::msg::Pose & ik_pose,
    const std::vector<double> & ik_seed_state,
    double /*timeout*/,
    std::vector<double> & solution,
    const kinematics::KinematicsBase::IKCallbackFn & solution_callback,
    moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions & /*options*/) const
{
  solution.clear();

  if (ik_seed_state.size() < 5)
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  std::vector<std::vector<double>> sols;
  solveAll(ik_pose.position.x, ik_pose.position.y, ik_pose.position.z,
           ik_seed_state, sols);

  if (sols.empty())
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  if (!solution_callback)
  {
    solution = sols.front();
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return true;
  }

  moveit_msgs::msg::MoveItErrorCodes cb_code;
  for (const auto & q : sols)
  {
    cb_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    solution_callback(ik_pose, q, cb_code);
    if (cb_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      solution = q;
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      return true;
    }
  }

  error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
  return false;
}

bool ArmAnalyticIKPlugin::searchPositionIK(
    const geometry_msgs::msg::Pose & ik_pose,
    const std::vector<double> & ik_seed_state,
    double timeout,
    const std::vector<double> & /*consistency_limits*/,
    std::vector<double> & solution,
    const kinematics::KinematicsBase::IKCallbackFn & solution_callback,
    moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions & options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution,
                          solution_callback, error_code, options);
}

// ─────────── getPositionFK：交给 RobotModel，转换到 base_frame ───────────
bool ArmAnalyticIKPlugin::getPositionFK(
    const std::vector<std::string> & link_names,
    const std::vector<double> & joint_angles,
    std::vector<geometry_msgs::msg::Pose> & poses) const
{
  poses.clear();
  if (!robot_model_)
  {
    return false;
  }

  const moveit::core::JointModelGroup * jmg =
      robot_model_->getJointModelGroup(group_name_);
  if (!jmg)
  {
    return false;
  }

  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  state.setJointGroupPositions(jmg, joint_angles);
  state.updateLinkTransforms();

  // 模型根坐标系 → base_frame_（本机械臂是 base_footprint → base_link）
  const Eigen::Isometry3d T_base = state.getGlobalLinkTransform(base_frame_);

  for (const auto & name : link_names)
  {
    if (!robot_model_->hasLinkModel(name))
    {
      poses.clear();
      return false;
    }
    poses.push_back(tf2::toMsg(T_base.inverse() * state.getGlobalLinkTransform(name)));
  }
  return true;
}

}  // namespace arm_ik_plugin

PLUGINLIB_EXPORT_CLASS(arm_ik_plugin::ArmAnalyticIKPlugin,
                       kinematics::KinematicsBase)
