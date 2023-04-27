#ifndef PEDSIM_DATA_RECORDER_HPP_
#define PEDSIM_DATA_RECORDER_HPP_

#include <fstream>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>

#include <pedsim_msgs/msg/agent_state.hpp>
#include <pedsim_msgs/msg/agent_states.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_transport/record_options.hpp"
#include "rosbag2_transport/recorder.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class PedsimDataRecorder : public rclcpp::Node
{
public:
  PedsimDataRecorder();

private:
  bool is_subscribed_;
  int frame_time; // milliseconds
  int ego_agent_id; // Agent types 0, 1 -> ordinary agents 2 -> Robot 3 -> standing/elderly agents
  
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<std_msgs::msg::String, std_msgs::msg::String>>> subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  void common_callback(const pedsim_msgs::msg::AgentStates &agents_msg,
                const nav_msgs::msg::OccupancyGrid &costmap_msg);
  
  void write_data_to_txt_file(const pedsim_msgs::msg::AgentStates &agents_msg,
                                    const nav_msgs::msg::OccupancyGrid &costmap_msg);
  
  void record_bag(const std::vector<std::string> &topics);
  
  bool check_agent_in_local_costmap(const pedsim_msgs::msg::AgentState &agent_msg,
                                    const nav_msgs::msg::OccupancyGrid &costmap_msg);
  
  void stop_subscription();
};

#endif  // PEDSIM_DATA_RECORDER_HPP_