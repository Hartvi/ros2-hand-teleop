#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <trac_ik/trac_ik.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

class TracIkNode : public rclcpp::Node
{
public:
  TracIkNode() : Node("trac_ik_node")
  {
    // Parameters: set these to match your URDF link names.
    base_link_ = this->declare_parameter<std::string>("base_link", "base_link");
    tip_link_ = this->declare_parameter<std::string>("tip_link", "end_effector_link");
    this->declare_parameter<std::string>("robot_description", "");

    timeout_ = this->declare_parameter<double>("timeout", 0.01);
    eps_ = this->declare_parameter<double>("eps", 1e-5);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/ik_target", 10, std::bind(&TracIkNode::on_target_pose, this, std::placeholders::_1));

    sol_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/main_joint_states", 10);

    init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(0),
        [this]()
        {
          init_timer_->cancel();
          this->init_solver(); // safe shared_from_this() here
        });
    max_dq = this->declare_parameter<double>("max_dq", 0.1);
  }

private:
  void init_solver()
  {
    std::string urdf;
    this->get_parameter("robot_description", urdf);
    if (urdf.empty())
    {
      RCLCPP_ERROR(get_logger(),
                   "robot_description is empty. Pass it as a parameter to this node from launch.");
      return;
    }
    // Construct TRAC-IK using node shared ptr + parameter name
    ik_ = std::make_unique<TRAC_IK::TRAC_IK>(
        this->shared_from_this(), // <-- required on Jazzy
        base_link_,
        tip_link_,
        "robot_description", // <-- parameter name, NOT XML
        timeout_,
        eps_,
        TRAC_IK::Speed // or TRAC_IK::Distance
    );

    RCLCPP_INFO(get_logger(), "robot_description length = %zu", urdf.size());
    bool ok = ik_->getKDLChain(chain_);
    if (!ok)
    {
      RCLCPP_ERROR(get_logger(), "Failed to build KDL chain from %s to %s",
                   base_link_.c_str(), tip_link_.c_str());
      ik_.reset();
      return;
    }

    ok = ik_->getKDLLimits(lower_, upper_);
    if (!ok)
    {
      RCLCPP_ERROR(get_logger(), "Failed to read joint limits from URDF.");
      ik_.reset();
      return;
    }

    seed_.resize(chain_.getNrOfJoints());
    for (unsigned i = 0; i < seed_.rows(); ++i)
    {
      seed_(i) = 0.0; // simple seed; you can improve by using current joint state
    }

    joint_names_.clear();
    joint_names_.reserve(chain_.getNrOfJoints());

    for (unsigned seg_idx = 0; seg_idx < chain_.getNrOfSegments(); ++seg_idx)
    {
      const auto &seg = chain_.getSegment(seg_idx);
      const auto &jnt = seg.getJoint();
      if (jnt.getType() != KDL::Joint::None)
      {
        joint_names_.push_back(jnt.getName());
      }
    }

    RCLCPP_INFO(get_logger(), "Joint names extracted: %zu", joint_names_.size());
    for (const auto &n : joint_names_)
    {
      RCLCPP_INFO(get_logger(), "  joint: %s", n.c_str());
    }

    if (joint_names_.size() != chain_.getNrOfJoints())
    {
      RCLCPP_ERROR(get_logger(),
                   "Mismatch: extracted %zu joint names but chain reports %u joints",
                   joint_names_.size(), chain_.getNrOfJoints());
      ik_.reset();
      return;
    }

    is_continuous_.assign(chain_.getNrOfJoints(), false);
    for (unsigned i = 0; i < chain_.getNrOfJoints(); ++i)
    {
      const double lo = lower_(i);
      const double hi = upper_(i);
      const double span = hi - lo;
      // Heuristic: treat as continuous if span is ~2π or limits look unbounded
      if (!std::isfinite(lo) || !std::isfinite(hi) || span >= (2.0 * M_PI - 1e-3) || span > 1e6)
      {
        is_continuous_[i] = true;
      }
    }
    have_last_cmd_ = false;
    last_cmd_.clear();

    RCLCPP_INFO(get_logger(),
                "TRAC-IK initialized: joints=%u (base=%s tip=%s)",
                chain_.getNrOfJoints(), base_link_.c_str(), tip_link_.c_str());
  }

  void on_target_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!ik_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "No TRAC-IK solver yet (still waiting for /robot_description).");
      return;
    }

    // Convert PoseStamped -> KDL::Frame
    const auto &p = msg->pose.position;
    const auto &q = msg->pose.orientation;

    KDL::Rotation R = KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w);
    KDL::Vector T(p.x, p.y, p.z);
    KDL::Frame target(R, T);

    KDL::JntArray result(chain_.getNrOfJoints());

    int rc = ik_->CartToJnt(seed_, target, result);

    if (rc < 0)
    {
      RCLCPP_WARN(get_logger(), "IK failed (rc=%d). Try different pose/seed/timeout.", rc);
      return;
    }

    sensor_msgs::msg::JointState out;
    out.header.stamp = now();
    out.name = joint_names_;
    out.position.resize(result.rows());

    if (!have_last_cmd_)
    {
      // First solution: take it as-is
      last_cmd_.resize(result.rows());
      for (unsigned i = 0; i < result.rows(); ++i)
      {
        double v = result(i);
        if (!is_continuous_[i])
          v = clamp_to_limits(i, v);
        out.position[i] = v;
        last_cmd_[i] = v;
        seed_(i) = v;
      }
      have_last_cmd_ = true;
      sol_pub_->publish(out);
      return;
    }

    // Smooth toward new solution
    for (unsigned i = 0; i < result.rows(); ++i)
    {
      const double prev = last_cmd_[i];
      const double tgt = result(i);

      double cmd;
      if (is_continuous_[i])
      {
        // “slerp-like” on a circle: shortest wrap-around interpolation
        const double d = shortest_angular_delta(prev, tgt);
        cmd = prev + std::min(std::max(-max_dq, d), max_dq);
      }
      else
      {
        // plain lerp
        cmd = prev + std::min(std::max(-max_dq, tgt - prev), max_dq);
        cmd = clamp_to_limits(i, cmd);
      }

      out.position[i] = cmd;
      last_cmd_[i] = cmd;
      seed_(i) = cmd; // important: seed next IK with the smoothed command
    }

    sol_pub_->publish(out);
  }

  static double shortest_angular_delta(double from, double to)
  {
    // returns delta in [-pi, +pi]
    double d = std::fmod((to - from) + M_PI, 2.0 * M_PI);
    if (d < 0)
      d += 2.0 * M_PI;
    return d - M_PI;
  }

  double clamp_to_limits(unsigned i, double v) const
  {
    if (i < lower_.rows() && i < upper_.rows())
    {
      const double lo = lower_(i);
      const double hi = upper_(i);
      if (std::isfinite(lo) && std::isfinite(hi) && lo <= hi)
      {
        v = std::min(std::max(v, lo), hi);
      }
    }
    return v;
  }

private:
  std::string base_link_;
  std::string tip_link_;
  double timeout_{0.01};
  double eps_{1e-5};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sol_pub_;

  std::unique_ptr<TRAC_IK::TRAC_IK> ik_;

  KDL::Chain chain_;
  KDL::JntArray lower_, upper_;
  KDL::JntArray seed_;
  std::vector<std::string> joint_names_;

  // constructor utils:
  rclcpp::TimerBase::SharedPtr init_timer_;

  double max_dq{0.1};
  bool have_last_cmd_{false};
  std::vector<double> last_cmd_;
  std::vector<bool> is_continuous_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TracIkNode>());
  rclcpp::shutdown();
  return 0;
}
