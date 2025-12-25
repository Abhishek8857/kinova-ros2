#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "agent_action_interface/action/execute_motion.hpp"

class MotionClient : public rclcpp::Node
{
public:
    using ExecuteMotion = agent_action_interface::action::ExecuteMotion;
    using GoalHandleMotion = rclcpp_action::ClientGoalHandle<ExecuteMotion>;

    MotionClient()
        : Node("motion_action_client")
    {
        client_ = rclcpp_action::create_client<ExecuteMotion>(this, "execute_motion");
    }

    void send_goal(const std::vector<double>& data)
    {
        if (!client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        auto goal_msg = ExecuteMotion::Goal();
        goal_msg.data = data;

        auto send_goal_options =
            rclcpp_action::Client<ExecuteMotion>::SendGoalOptions();

        send_goal_options.feedback_callback =
            std::bind(&MotionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

        send_goal_options.result_callback =
            std::bind(&MotionClient::result_callback, this, std::placeholders::_1);

        client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    void feedback_callback(
        GoalHandleMotion::SharedPtr,
        const std::shared_ptr<const ExecuteMotion::Feedback> feedback)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Feedback: %s (%.2f)",
            feedback->state.c_str(),
            feedback->progress
        );
    }

    void result_callback(const GoalHandleMotion::WrappedResult &result)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Result: success=%d, code=%s, desc=%s",
            result.result->success,
            result.result->error_code.c_str(),
            result.result->error_description.c_str()
        );
    }

    rclcpp_action::Client<ExecuteMotion>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionClient>();

    node->send_goal({1, 0.4, 0.1, 0.3, 0.707, 0, -0.7070, 1});

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
