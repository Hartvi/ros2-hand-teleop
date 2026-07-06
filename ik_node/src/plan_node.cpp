#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>

#include <hand_publisher_interfaces/msg/obstacle.hpp>
#include <hand_publisher_interfaces/srv/plan_pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

class PlanningService : public rclcpp::Node
{
public:
    PlanningService()
        : Node(
              "plan_ik",
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        init_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PlanningService::initializeMoveIt, this));
    }

    ~PlanningService() override
    {
        move_group_interface_.reset();
        planning_scene_interface_.reset();
    }

private:
    void initializeMoveIt()
    {
        if (move_group_interface_)
        {
            return;
        }

        init_timer_->cancel();

        std::string move_group_name = "panda_arm";
        get_parameter_or("move_group_name", move_group_name, std::string("panda_arm"));

        move_group_interface_ =
            std::make_unique<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), move_group_name);
        move_group_interface_->setPlanningTime(5.0);

        planning_scene_interface_ =
            std::make_unique<moveit::planning_interface::PlanningSceneInterface>();

        get_parameter_or("planning_frame", planning_frame_, move_group_interface_->getPlanningFrame());
        std::string obstacle_topic = "/planning_obstacles";
        get_parameter_or("obstacle_topic", obstacle_topic, std::string("/planning_obstacles"));

        /*
        ros2 service call /plan_and_execute hand_publisher_interfaces/srv/PlanPose "{target: {header: {frame_id: 'world'}, pose: {position: {x: 0.3, y: 0.2, z: 0.5}, orientation: {w: 1.0}}}}"
        */
        service_ = create_service<hand_publisher_interfaces::srv::PlanPose>(
            "plan_and_execute",
            std::bind(
                &PlanningService::planAndExecuteCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        obstacle_sub_ = create_subscription<hand_publisher_interfaces::msg::Obstacle>(
            obstacle_topic,
            10,
            std::bind(&PlanningService::obstacleCallback, this, std::placeholders::_1));

        joint_state_pub_ =
            create_publisher<sensor_msgs::msg::JointState>("/main_joint_states", 10);

        RCLCPP_INFO(get_logger(), "MoveIt ready for planning group '%s'", move_group_name.c_str());
        RCLCPP_INFO(get_logger(), "Service ready: /plan_and_execute");
        RCLCPP_INFO(get_logger(), "Subscribing to obstacle updates on %s", obstacle_topic.c_str());
    }

    void obstacleCallback(const hand_publisher_interfaces::msg::Obstacle::SharedPtr msg)
    {
        if (!planning_scene_interface_)
        {
            RCLCPP_WARN(get_logger(), "Planning scene interface is not initialized yet");
            return;
        }

        if (msg->id.empty())
        {
            RCLCPP_WARN(get_logger(), "Ignoring obstacle update with empty id");
            return;
        }

        if (msg->remove)
        {
            planning_scene_interface_->removeCollisionObjects({msg->id});
            known_obstacles_.erase(msg->id);
            RCLCPP_INFO(get_logger(), "Removed obstacle '%s'", msg->id.c_str());
            return;
        }

        moveit_msgs::msg::CollisionObject collision_object;
        shape_msgs::msg::SolidPrimitive primitive;
        if (!makePrimitive(*msg, primitive))
        {
            return;
        }

        collision_object.header.frame_id = msg->header.frame_id.empty()
                                               ? planning_frame_
                                               : msg->header.frame_id;
        collision_object.header.stamp = msg->header.stamp;
        collision_object.id = msg->id;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(msg->pose);
        collision_object.operation = collision_object.ADD;

        planning_scene_interface_->applyCollisionObject(collision_object);

        const auto summary = summarize(*msg);
        const auto previous = known_obstacles_.find(msg->id);
        if (previous == known_obstacles_.end())
        {
            RCLCPP_INFO(get_logger(), "Registered obstacle %s", summary.c_str());
        }
        else if (previous->second != summary)
        {
            RCLCPP_INFO(get_logger(), "Updated obstacle %s", summary.c_str());
        }
        known_obstacles_[msg->id] = summary;
    }

    bool makePrimitive(
        const hand_publisher_interfaces::msg::Obstacle &msg,
        shape_msgs::msg::SolidPrimitive &primitive) const
    {
        if (msg.type == "box" || msg.type == "cube")
        {
            if (msg.dimensions.size() < 3)
            {
                RCLCPP_WARN(get_logger(), "Box obstacle '%s' requires [x, y, z] dimensions", msg.id.c_str());
                return false;
            }
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[primitive.BOX_X] = msg.dimensions[0];
            primitive.dimensions[primitive.BOX_Y] = msg.dimensions[1];
            primitive.dimensions[primitive.BOX_Z] = msg.dimensions[2];
            return true;
        }

        if (msg.type == "cylinder")
        {
            if (msg.dimensions.size() < 2)
            {
                RCLCPP_WARN(get_logger(), "Cylinder obstacle '%s' requires [radius, height] dimensions", msg.id.c_str());
                return false;
            }
            primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(2);
            primitive.dimensions[primitive.CYLINDER_RADIUS] = msg.dimensions[0];
            primitive.dimensions[primitive.CYLINDER_HEIGHT] = msg.dimensions[1];
            return true;
        }

        if (msg.type == "sphere")
        {
            if (msg.dimensions.empty())
            {
                RCLCPP_WARN(get_logger(), "Sphere obstacle '%s' requires [radius] dimensions", msg.id.c_str());
                return false;
            }
            primitive.type = primitive.SPHERE;
            primitive.dimensions.resize(1);
            primitive.dimensions[primitive.SPHERE_RADIUS] = msg.dimensions[0];
            return true;
        }

        RCLCPP_WARN(get_logger(), "Unknown obstacle type '%s' for '%s'", msg.type.c_str(), msg.id.c_str());
        return false;
    }

    std::string summarize(const hand_publisher_interfaces::msg::Obstacle &msg) const
    {
        std::ostringstream out;
        out << "'" << msg.id << "' type=" << msg.type << " dims=[";
        for (size_t i = 0; i < msg.dimensions.size(); ++i)
        {
            if (i > 0)
            {
                out << ", ";
            }
            out << msg.dimensions[i];
        }
        out << "] pose=(" << msg.pose.position.x << ", " << msg.pose.position.y << ", "
            << msg.pose.position.z << ") frame="
            << (msg.header.frame_id.empty() ? planning_frame_ : msg.header.frame_id);
        return out.str();
    }

    void planAndExecuteCallback(
        const std::shared_ptr<hand_publisher_interfaces::srv::PlanPose::Request> request,
        std::shared_ptr<hand_publisher_interfaces::srv::PlanPose::Response> response)
    {
        if (!move_group_interface_)
        {
            response->success = false;
            response->message = "MoveIt interface is not initialized yet";
            return;
        }

        const std::string frame_id = request->target.header.frame_id.empty()
                                         ? move_group_interface_->getPlanningFrame()
                                         : request->target.header.frame_id;

        move_group_interface_->setPoseReferenceFrame(frame_id);
        move_group_interface_->setStartStateToCurrentState();
        move_group_interface_->setPoseTarget(request->target.pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        const auto success = static_cast<bool>(move_group_interface_->plan(plan));

        if (!success)
        {
            response->success = false;
            response->message = "Planning failed";
            move_group_interface_->clearPoseTargets();
            RCLCPP_ERROR(get_logger(), "Planning failed");
            return;
        }

        const auto result = move_group_interface_->execute(plan);
        move_group_interface_->clearPoseTargets();

        if (static_cast<bool>(result))
        {
            response->success = true;
            response->message = "Plan executed successfully";
            RCLCPP_INFO(get_logger(), "Plan executed successfully");

            if (!plan.trajectory.joint_trajectory.points.empty())
            {
                sensor_msgs::msg::JointState joint_state;
                joint_state.header.stamp = now();
                joint_state.name = plan.trajectory.joint_trajectory.joint_names;
                joint_state.position = plan.trajectory.joint_trajectory.points.back().positions;
                joint_state_pub_->publish(joint_state);
            }
        }
        else
        {
            response->success = false;
            response->message = "Execution failed";
            RCLCPP_ERROR(get_logger(), "Execution failed");
        }
    }

private:
    std::string planning_frame_ = "world";
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<hand_publisher_interfaces::msg::Obstacle>::SharedPtr obstacle_sub_;
    rclcpp::Service<hand_publisher_interfaces::srv::PlanPose>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    std::unordered_map<std::string, std::string> known_obstacles_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PlanningService>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    executor.remove_node(node);
    node.reset();

    rclcpp::shutdown();
    return 0;
}