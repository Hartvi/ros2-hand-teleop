#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_srvs/srv/pose.hpp>

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

        RCLCPP_INFO(get_logger(), "Service ready: /plan_and_execute");
    }

private:
    void planAndExecuteCallback(
        const std::shared_ptr<geometry_srvs::srv::Pose::Request> request,
        std::shared_ptr<geometry_srvs::srv::Pose::Response> response)
    {
        addCollisionBox();

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
        }
        else
        {
            response->success = false;
            RCLCPP_ERROR(get_logger(), "Execution failed");
        }
    }

    void addCollisionBox()
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id =
            move_group_interface_->getPlanningFrame();

        collision_object.id = "box1";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.5;
        primitive.dimensions[primitive.BOX_Y] = 0.1;
        primitive.dimensions[primitive.BOX_Z] = 0.5;

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.2;
        box_pose.position.y = 0.2;
        box_pose.position.z = 0.25;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        planning_scene_interface_->applyCollisionObject(collision_object);
        planning_scene_interface_->getAttachedObjects
    }

private:
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface>
        move_group_interface_;

    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface>
        planning_scene_interface_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, cgeometry_srvs::srv::Pose
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PlanningService>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}