#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

std::mutex execution_mutex;  // 保证执行顺序的互斥锁

class MoveTargetNode : public rclcpp::Node {
public:
    MoveTargetNode() : Node("move_target"), move_group_(std::shared_ptr<rclcpp::Node>(this, [](auto){}), "arm") {
        // 创建订阅器，监听目标位姿话题
        subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "target_pose", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
                this->poseCallback(msg);
            });
        
        RCLCPP_INFO(this->get_logger(), "节点已启动，等待目标位姿...");
    }

private:
    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(execution_mutex);
        
        // 创建目标位姿
        geometry_msgs::msg::Pose target_pose = *msg;
        
        // 设置并规划路径
        move_group_.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        
        if (move_group_.plan(plan)) {
            RCLCPP_INFO(this->get_logger(), "规划成功，开始执行");
            move_group_.execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "规划失败，请检查目标位姿");
        }
    }

    moveit::planning_interface::MoveGroupInterface move_group_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
};

int main(int argc,char *argv[]) {
    rclcpp::init(argc, argv);   
    auto node = std::make_shared<MoveTargetNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}