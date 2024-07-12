#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

std::shared_ptr<rclcpp::Node> node;
std::shared_ptr<rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>> action_client;
bool common_goal_accepted = false;
rclcpp_action::ResultCode common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
int common_action_result_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;

void common_goal_response(rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr goal_handle) {
    if (!goal_handle) {
        common_goal_accepted = false;
        RCLCPP_INFO(node->get_logger(), "Goal rejected");
    } else {
        common_goal_accepted = true;
        RCLCPP_INFO(node->get_logger(), "Goal accepted");
    }
}

void common_result_response(const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result) {
    common_resultcode = result.code;
    common_action_result_code = result.result->error_code;
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node->get_logger(), "SUCCEEDED result code");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(node->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(node->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_INFO(node->get_logger(), "Unknown result code");
            return;
    }
}

void common_feedback(
    rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
    const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback) {
    std::cout << "Feedback received. Current Positions: ";
    for (auto & position : feedback->desired.positions) {
        std::cout << position << " ";
    }
    std::cout << std::endl;
}

void keyLoop() {
    char c;
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use keys 'a' and 'b' to control the joint.");
    while (true) {
        std::cin.get(c);
        double position = 0;
        if (c == 'a') position = 1.0;
        else if (c == 'b') position = -1.0;
        else continue;

        if (!action_client->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
            continue;
        }

        auto goal_msg = control_msgs::action::FollowJointTrajectory_Goal();
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.time_from_start = rclcpp::Duration(1, 0); // Reach the target position after 1 second
        point.positions.push_back(position);
        // Set the correct joint names according to your actual robot setup
        goal_msg.trajectory.joint_names = {"slider_to_cart"};
        goal_msg.trajectory.points.push_back(point);

        rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions opt;
        opt.goal_response_callback = common_goal_response;
        opt.result_callback = common_result_response;
        opt.feedback_callback = common_feedback;

        action_client->async_send_goal(goal_msg, opt);
    }
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("trajectory_test_node");
    action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        node, "/joint_trajectory_controller/follow_joint_trajectory");

    std::thread key_thread(keyLoop);
    key_thread.detach();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
