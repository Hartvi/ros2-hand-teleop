#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>

#include <hand_publisher_interfaces/msg/obstacle.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

class ObstacleSceneNode : public rclcpp::Node
{
public:
    ObstacleSceneNode()
        : Node(
              "obstacle_scene_node",
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        init_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ObstacleSceneNode::initializeMoveIt, this));
    }

    ~ObstacleSceneNode() override
    {
        planning_scene_interface_.reset();
    }

private:
    void initializeMoveIt()
    {
        if (planning_scene_interface_)
        {
            return;
        }

        init_timer_->cancel();

        get_parameter_or("planning_frame", planning_frame_, std::string("world"));
        std::string obstacle_topic = "/planning_obstacles";
        get_parameter_or("obstacle_topic", obstacle_topic, std::string("/planning_obstacles"));
        std::string planning_scene_topic = "/monitored_planning_scene";
        get_parameter_or(
            "planning_scene_topic",
            planning_scene_topic,
            std::string("/monitored_planning_scene"));

        planning_scene_interface_ =
            std::make_unique<moveit::planning_interface::PlanningSceneInterface>();

        obstacle_sub_ = create_subscription<hand_publisher_interfaces::msg::Obstacle>(
            obstacle_topic,
            10,
            std::bind(&ObstacleSceneNode::obstacleCallback, this, std::placeholders::_1));

        planning_scene_sub_ = create_subscription<moveit_msgs::msg::PlanningScene>(
            planning_scene_topic,
            10,
            std::bind(&ObstacleSceneNode::planningSceneCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Obstacle scene node ready");
        RCLCPP_INFO(get_logger(), "Subscribing to obstacle updates on %s", obstacle_topic.c_str());
        RCLCPP_INFO(get_logger(), "Logging planning scene updates from %s", planning_scene_topic.c_str());
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

        shape_msgs::msg::SolidPrimitive primitive;
        if (!makePrimitive(*msg, primitive))
        {
            return;
        }

        moveit_msgs::msg::CollisionObject collision_object;
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

    void planningSceneCallback(const moveit_msgs::msg::PlanningScene::SharedPtr msg) const
    {
        if (msg->world.collision_objects.empty() && msg->robot_state.attached_collision_objects.empty())
        {
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "Planning scene update: is_diff=%s collision_objects=%zu attached_objects=%zu",
            msg->is_diff ? "true" : "false",
            msg->world.collision_objects.size(),
            msg->robot_state.attached_collision_objects.size());

        for (const auto &object : msg->world.collision_objects)
        {
            RCLCPP_INFO(
                get_logger(),
                "Planning scene object '%s' operation=%d primitives=%zu meshes=%zu",
                object.id.c_str(),
                object.operation,
                object.primitives.size(),
                object.meshes.size());
        }
    }

    std::string planning_frame_ = "world";
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    rclcpp::Subscription<hand_publisher_interfaces::msg::Obstacle>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_sub_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    std::unordered_map<std::string, std::string> known_obstacles_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ObstacleSceneNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    executor.remove_node(node);
    node.reset();

    rclcpp::shutdown();
    return 0;
}
