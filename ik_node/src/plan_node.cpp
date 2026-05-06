#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_srvs/srv/pose.hpp>
#include <hand_publisher_interfaces/srv/add_object.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

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
        move_group_interface_ =
            std::make_unique<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), "manipulator");

        planning_scene_interface_ =
            std::make_unique<moveit::planning_interface::PlanningSceneInterface>();

        service_ = create_service<geometry_srvs::srv::Pose>(
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

        RCLCPP_INFO(get_logger(), "Service ready: /plan_and_execute");
        RCLCPP_INFO(get_logger(), "Service ready: /add_object");

        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/main_joint_states", 10);
    }

private:
    void addObjectCallback(
        const std::shared_ptr<hand_publisher_interfaces::srv::AddObject::Request> request,
        std::shared_ptr<hand_publisher_interfaces::srv::AddObject::Response> response)
    {
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
        const std::shared_ptr<geometry_srvs::srv::Pose::Request> request,
        std::shared_ptr<geometry_srvs::srv::Pose::Response> response)
    {
        move_group_interface_->setPoseTarget(request->pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        auto success =
            static_cast<bool>(move_group_interface_->plan(plan));

        if (!success)
        {
            response->success = false;
            RCLCPP_ERROR(get_logger(), "Planning failed");
            return;
        }

        auto result = move_group_interface_->execute(plan);

        if (static_cast<bool>(result))
        {
            response->success = true;
            RCLCPP_INFO(get_logger(), "Plan executed successfully");

            // Publish the final joint states to /main_joint_states
            sensor_msgs::msg::JointState joint_state;
            joint_state.header.stamp = now();
            joint_state.name = plan.trajectory.joint_trajectory.joint_names;
            joint_state.position = plan.trajectory.joint_trajectory.points.back().positions;
            joint_state_pub_->publish(joint_state);
        }
        else
        {
            response->success = false;
            RCLCPP_ERROR(get_logger(), "Execution failed");
        }
    }

private:
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface>
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    move_group_interface_;

    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface>
        planning_scene_interface_;

    rclcpp::Service<geometry_srvs::srv::Pose>::SharedPtr service_;
    rclcpp::Service<hand_publisher_interfaces::srv::AddObject>::SharedPtr add_object_service_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PlanningService>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}