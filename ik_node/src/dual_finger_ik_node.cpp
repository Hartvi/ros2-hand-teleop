#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "hand_publisher_interfaces/msg/hand_points.hpp"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <Eigen/Dense>

class DualFingerIkNode : public rclcpp::Node
{
public:
    DualFingerIkNode() : Node("dual_finger_ik_node")
    {
        // Parameters
        base_link_ = declare_parameter<std::string>("base_link", "panda_link0");
        tip_link_l_ = declare_parameter<std::string>("tip_link_left", "panda_leftfinger");
        tip_link_r_ = declare_parameter<std::string>("tip_link_right", "panda_rightfinger");
        declare_parameter<std::string>("robot_description", "");

        lambda_ = declare_parameter<double>("lambda", 0.01);
        max_dq_ = declare_parameter<double>("max_dq", 0.1);

        // Regularisation diagonal: 7 arm joints + 1 gripper
        std::vector<double> default_a(8, 0.001);
        default_a[7] = 0.01;
        a_diag_ = declare_parameter<std::vector<double>>("a_diag", default_a);

        // Left-finger hand-point index (3-element stride):
        //   MediaPipe landmark #4 → points[4*3 .. 4*3+2]   (index finger tip)
        //   MediaPipe landmark #8 → points[8*3 .. 8*3+2]   (middle finger tip)
        hand_idx_l_ = declare_parameter<int>("hand_point_index_left", 4);
        hand_idx_r_ = declare_parameter<int>("hand_point_index_right", 8);

        // Publishers / subscribers
        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/main_joint_states", 10);

        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&DualFingerIkNode::on_joint_state, this, std::placeholders::_1));

        hand_sub_ = create_subscription<hand_publisher_interfaces::msg::HandPoints>(
            "hand_points_corrected", 10,
            std::bind(&DualFingerIkNode::on_hand_points, this, std::placeholders::_1));

        // Deferred init (needs shared_from_this)
        init_timer_ = create_wall_timer(
            std::chrono::milliseconds(0), [this]()
            {
        init_timer_->cancel();
        init_solver(); });
    }

private:
    // ── Initialisation ──────────────────────────────────────────────────────
    void init_solver()
    {
        std::string urdf;
        get_parameter("robot_description", urdf);
        if (urdf.empty())
        {
            RCLCPP_ERROR(get_logger(), "robot_description is empty — pass it via launch.");
            return;
        }

        // Parse URDF → KDL tree
        KDL::Tree tree;
        if (!kdl_parser::treeFromString(urdf, tree))
        {
            RCLCPP_ERROR(get_logger(), "Failed to parse URDF into KDL tree.");
            return;
        }

        // Extract two chains: base → left finger, base → right finger
        if (!tree.getChain(base_link_, tip_link_l_, chain_l_))
        {
            RCLCPP_ERROR(get_logger(), "No chain from %s to %s", base_link_.c_str(), tip_link_l_.c_str());
            return;
        }
        if (!tree.getChain(base_link_, tip_link_r_, chain_r_))
        {
            RCLCPP_ERROR(get_logger(), "No chain from %s to %s", base_link_.c_str(), tip_link_r_.c_str());
            return;
        }

        n_joints_l_ = chain_l_.getNrOfJoints();
        n_joints_r_ = chain_r_.getNrOfJoints();

        // Collect joint names and limits from chain L (the "primary" chain)
        joint_names_.clear();
        lower_.resize(n_joints_l_);
        upper_.resize(n_joints_l_);
        unsigned jidx = 0;
        for (unsigned s = 0; s < chain_l_.getNrOfSegments(); ++s)
        {
            const auto &jnt = chain_l_.getSegment(s).getJoint();
            if (jnt.getType() != KDL::Joint::None)
            {
                joint_names_.push_back(jnt.getName());
                // KDL doesn't carry limits; we parse them from the URDF ourselves.
                // For now, use wide defaults; override below.
                lower_(jidx) = -3.0;
                upper_(jidx) = 3.0;
                ++jidx;
            }
        }

        // Parse joint limits from the URDF XML
        parse_limits_from_urdf(urdf);

        // Build solvers
        fk_l_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_l_);
        fk_r_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_r_);
        jac_solver_l_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_l_);
        jac_solver_r_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_r_);

        // Initialise q to zero
        q_.resize(n_joints_l_);
        for (unsigned i = 0; i < n_joints_l_; ++i)
            q_(i) = 0.0;

        solver_ready_ = true;
        RCLCPP_INFO(get_logger(),
                    "Dual-finger IK ready: %u joints (chain L: %s, chain R: %s)",
                    n_joints_l_, tip_link_l_.c_str(), tip_link_r_.c_str());
        for (const auto &n : joint_names_)
            RCLCPP_INFO(get_logger(), "  joint: %s", n.c_str());
    }

    // ── URDF limit parsing ─────────────────────────────────────────────────
    void parse_limits_from_urdf(const std::string &urdf)
    {
        // Minimal XML parsing: find <joint name="X" ...><limit lower="L" upper="U" .../>
        // We match against joint_names_ already extracted from KDL.
        for (unsigned i = 0; i < joint_names_.size(); ++i)
        {
            const auto &name = joint_names_[i];
            auto jpos = urdf.find("name=\"" + name + "\"");
            if (jpos == std::string::npos)
                continue;

            auto lim_pos = urdf.find("<limit", jpos);
            if (lim_pos == std::string::npos || lim_pos - jpos > 800)
                continue;

            auto extract = [&](const std::string &attr) -> double
            {
                auto ap = urdf.find(attr + "=\"", lim_pos);
                if (ap == std::string::npos || ap - lim_pos > 300)
                    return 0.0;
                ap += attr.size() + 2;
                auto end = urdf.find("\"", ap);
                if (end == std::string::npos)
                    return 0.0;
                return std::stod(urdf.substr(ap, end - ap));
            };
            lower_(i) = extract("lower");
            upper_(i) = extract("upper");
        }
    }

    // ── Callbacks ───────────────────────────────────────────────────────────
    void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(mu_);
        // Map incoming joint states into our q_ vector by name
        for (size_t m = 0; m < msg->name.size(); ++m)
        {
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                if (msg->name[m] == joint_names_[i] && m < msg->position.size())
                {
                    q_(static_cast<unsigned>(i)) = msg->position[m];
                }
            }
        }
    }

    void on_hand_points(const hand_publisher_interfaces::msg::HandPoints::SharedPtr msg)
    {
        if (!solver_ready_)
            return;

        const auto &pts = msg->points;
        const int l3 = hand_idx_l_ * 3;
        const int r3 = hand_idx_r_ * 3;
        if (static_cast<int>(pts.size()) < std::max(l3, r3) + 3)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "HandPoints too short (%zu), need index %d", pts.size(), std::max(l3, r3) + 3);
            return;
        }

        // Target positions from MediaPipe
        Eigen::Vector3d target_a(pts[l3], pts[l3 + 1], pts[l3 + 2]);
        Eigen::Vector3d target_b(pts[r3], pts[r3 + 1], pts[r3 + 2]);

        KDL::JntArray q_snap;
        {
            std::lock_guard<std::mutex> lk(mu_);
            q_snap = q_;
        }

        // ── FK ──
        KDL::Frame fk_frame_l, fk_frame_r;
        fk_l_->JntToCart(q_snap, fk_frame_l);

        // Build q for right chain — same values, but chain_r may have a different
        // mimic mapping.  For Panda: joints 1-7 are shared, finger_joint2 mirrors
        // finger_joint1.  KDL treats them as independent, so copy q and override
        // the last joint with the same finger value.
        KDL::JntArray q_r(n_joints_r_);
        for (unsigned i = 0; i < std::min(n_joints_l_, n_joints_r_); ++i)
            q_r(i) = q_snap(i);
        fk_r_->JntToCart(q_r, fk_frame_r);

        Eigen::Vector3d fk_l_pos(fk_frame_l.p.x(), fk_frame_l.p.y(), fk_frame_l.p.z());
        Eigen::Vector3d fk_r_pos(fk_frame_r.p.x(), fk_frame_r.p.y(), fk_frame_r.p.z());

        // ── Finger assignment heuristic ──
        Eigen::Vector3d p_l, p_r;
        double d_ll = (fk_l_pos - target_a).squaredNorm();
        double d_lr = (fk_l_pos - target_b).squaredNorm();
        if (d_ll <= d_lr)
        {
            p_l = target_a;
            p_r = target_b;
        }
        else
        {
            p_l = target_b;
            p_r = target_a;
        }

        // ── Jacobians (6 × n_joints, keep top 3 rows) ──
        const unsigned n = n_joints_l_; // = 8 for Panda
        KDL::Jacobian jac_kdl_l(n_joints_l_), jac_kdl_r(n_joints_r_);
        jac_solver_l_->JntToJac(q_snap, jac_kdl_l);
        jac_solver_r_->JntToJac(q_r, jac_kdl_r);

        // Position-only Jacobians (3 × n)
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> J_l(3, n), J_r(3, n);
        for (unsigned c = 0; c < n; ++c)
        {
            for (unsigned r = 0; r < 3; ++r)
            {
                J_l(r, c) = jac_kdl_l(r, c);
                J_r(r, c) = (c < n_joints_r_) ? jac_kdl_r(r, c) : 0.0;
            }
        }

        // ── Stacked Jacobian J_bar (6 × n) and residual r (6 × 1) ──
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> J_bar(6, n);
        J_bar.topRows(3) = J_l;
        J_bar.bottomRows(3) = J_r;

        Eigen::Matrix<double, 6, 1> residual;
        residual.head<3>() = fk_l_pos - p_l;
        residual.tail<3>() = fk_r_pos - p_r;

        // ── Solve  (J^T J + A + λI) Δθ = −J^T r ──
        Eigen::MatrixXd JtJ = J_bar.transpose() * J_bar;    // n × n
        Eigen::VectorXd Jtr = J_bar.transpose() * residual; // n × 1

        Eigen::MatrixXd H = JtJ;
        for (unsigned i = 0; i < n; ++i)
        {
            double a_i = (i < a_diag_.size()) ? a_diag_[i] : 0.001;
            H(i, i) += a_i + lambda_;
        }

        Eigen::VectorXd dq = H.ldlt().solve(-Jtr);

        // ── Clamp & apply ──
        sensor_msgs::msg::JointState out;
        out.header.stamp = now();
        out.name = joint_names_;
        out.position.resize(n);

        for (unsigned i = 0; i < n; ++i)
        {
            double step = std::clamp(dq(i), -max_dq_, max_dq_);
            double val = q_snap(i) + step;
            val = std::clamp(val, lower_(i), upper_(i));
            out.position[i] = val;
        }

        joint_pub_->publish(out);
    }

    // ── Members ─────────────────────────────────────────────────────────────
    std::string base_link_, tip_link_l_, tip_link_r_;
    double lambda_{0.01}, max_dq_{0.1};
    std::vector<double> a_diag_;
    int hand_idx_l_{4}, hand_idx_r_{8};

    KDL::Chain chain_l_, chain_r_;
    unsigned n_joints_l_{0}, n_joints_r_{0};
    std::vector<std::string> joint_names_;
    KDL::JntArray lower_, upper_;

    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_l_, fk_r_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_l_, jac_solver_r_;

    KDL::JntArray q_;
    std::mutex mu_;
    bool solver_ready_{false};

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<hand_publisher_interfaces::msg::HandPoints>::SharedPtr hand_sub_;
    rclcpp::TimerBase::SharedPtr init_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualFingerIkNode>());
    rclcpp::shutdown();
    return 0;
}
