#include "rclcpp/rclcpp.hpp"
#include "pedsim_msgs/msg/agent_states.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <experimental/optional>

class AgentPosePublisher : public rclcpp::Node
{
public:
  AgentPosePublisher() : Node("agent_pose_publisher")
  {
    agent_states_subscription_ = this->create_subscription<pedsim_msgs::msg::AgentStates>(
      "pedsim_simulator/simulated_agents",
      10,
      std::bind(&AgentPosePublisher::agentStatesCallback, this, std::placeholders::_1));
    
      // Create tf buffer and transform listener   
      m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      m_transform_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
  }

private:
  void agentStatesCallback(const pedsim_msgs::msg::AgentStates::SharedPtr msg)
  {
    for (const auto& agent_state : msg->agent_states)
    {
      if (odometry_publishers_.count(agent_state.id) == 0)
      {
        // Create a new publisher for this agent
        std::string topic_name = "pedsim_simulator/agent_" + std::to_string(agent_state.id) + "_pose";
        odometry_publishers_[agent_state.id] = this->create_publisher<nav_msgs::msg::Odometry>(topic_name, 10);
      }

      geometry_msgs::msg::Pose agent_pose;
      agent_pose.position.x = agent_state.pose.position.x;
      agent_pose.position.y = agent_state.pose.position.y;
      agent_pose.orientation.w = 1.0;

      // Transform to map frame
      if (auto pose_out = pose_transform(agent_pose, "map", "agent_" + std::to_string(agent_state.id))) {
        geometry_msgs::msg::Pose map_pose = pose_out.value();
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.pose.pose = map_pose;
        odometry_publishers_[agent_state.id]->publish(odom_msg);
      }
    }
  }
  std::experimental::optional<geometry_msgs::msg::Pose> pose_transform(
        const geometry_msgs::msg::Pose & obj, 
        const std::string & output_frame, 
        const std::string & input_frame) {   
    geometry_msgs::msg::Pose transformed_obj;
    try {
      // rclcpp::Time now = this->get_clock()->now();
      auto tf = m_tf_buffer->lookupTransform(output_frame, input_frame, tf2::TimePointZero);
      tf2::doTransform(obj, transformed_obj, tf);
    } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());  
    }
    return transformed_obj;
  }

private:
  // tf buffer and transform listener
  std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_transform_listener {nullptr};
  rclcpp::Subscription<pedsim_msgs::msg::AgentStates>::SharedPtr agent_states_subscription_;
  std::unordered_map<int64_t, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odometry_publishers_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgentPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
