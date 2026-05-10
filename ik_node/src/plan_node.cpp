#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <hand_publisher_interfaces/srv/add_object.hpp>
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
              "planning_service",
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

        add_object_service_ = create_service<hand_publisher_interfaces::srv::AddObject>(
            "add_object",
            std::bind(
                &PlanningService::addObjectCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        joint_state_pub_ =
            create_publisher<sensor_msgs::msg::JointState>("/main_joint_states", 10);

        RCLCPP_INFO(get_logger(), "MoveIt ready for planning group '%s'", move_group_name.c_str());
        RCLCPP_INFO(get_logger(), "Service ready: /plan_and_execute");
        RCLCPP_INFO(get_logger(), "Service ready: /add_object");
    }

    void addObjectCallback(
        const std::shared_ptr<hand_publisher_interfaces::srv::AddObject::Request> request,
        std::shared_ptr<hand_publisher_interfaces::srv::AddObject::Response> response)
    {
        if (!move_group_interface_ || !planning_scene_interface_)
        {
            response->success = false;
            response->message = "MoveIt interface is not initialized yet";
            return;
        }

        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = move_group_interface_->getPlanningFrame();
        collision_object.id = request->name;

        shape_msgs::msg::SolidPrimitive primitive;
        if (request->type == "cube")
        {
            primitive.type = primitive.BOX;
            if (request->dimensions.size() >= 3)
            {
                primitive.dimensions.resize(3);
                primitive.dimensions[primitive.BOX_X] = request->dimensions[0];
                primitive.dimensions[primitive.BOX_Y] = request->dimensions[1];
                primitive.dimensions[primitive.BOX_Z] = request->dimensions[2];
            }
            else
            {
                response->success = false;
                response->message = "Cube requires 3 dimensions [x, y, z]";
                return;
            }
        }
        else if (request->type == "cylinder")
        {
            primitive.type = primitive.CYLINDER;
            if (request->dimensions.size() >= 2)
            {
                primitive.dimensions.resize(2);
                primitive.dimensions[primitive.CYLINDER_RADIUS] = request->dimensions[0];
                primitive.dimensions[primitive.CYLINDER_HEIGHT] = request->dimensions[1];
            }
            else
            {
                response->success = false;
                response->message = "Cylinder requires 2 dimensions [radius, height]";
                return;
            }
        }
        else
        {
            response->success = false;
            response->message = "Unknown type: " + request->type;
            return;
        }

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(request->pose);
        collision_object.operation = collision_object.ADD;

        planning_scene_interface_->applyCollisionObject(collision_object);

        response->success = true;
        response->message = "Object added successfully";
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
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Service<hand_publisher_interfaces::srv::PlanPose>::SharedPtr service_;
    rclcpp::Service<hand_publisher_interfaces::srv::AddObject>::SharedPtr add_object_service_;
    rclcpp::TimerBase::SharedPtr init_timer_;
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