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
  int frame_time; // milliseconds
  int ego_agent_id; // Agent types 0, 1 -> ordinary agents 2 -> Robot 3 -> standing/elderly agents
  void callback_1(const pedsim_msgs::msg::AgentStates &agents_msg,
                const nav_msgs::msg::OccupancyGrid &local_costmap_msg);
  
  void callback_2(const pedsim_msgs::msg::AgentStates &agents_msg,
              const nav_msgs::msg::OccupancyGrid &global_costmap_msg);

  std::shared_ptr<message_filters::Subscriber<pedsim_msgs::msg::AgentStates>> agents_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::OccupancyGrid>> local_costmap_sub_;
  //std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::OccupancyGrid>> global_costmap_sub_;

  std::shared_ptr<message_filters::TimeSynchronizer<pedsim_msgs::msg::AgentStates, nav_msgs::msg::OccupancyGrid>> sync_local_;
  //std::shared_ptr<message_filters::TimeSynchronizer<pedsim_msgs::msg::AgentStates, nav_msgs::msg::OccupancyGrid>> sync_global_;

  void write_data_to_txt_file(const pedsim_msgs::msg::AgentStates &agents_msg,
                                    const nav_msgs::msg::OccupancyGrid &local_costmap_msg);
  void record_bag(const std::vector<std::string> &topics);
  bool check_agent_in_local_costmap(const pedsim_msgs::msg::AgentState &agent_msg,
                                    const nav_msgs::msg::OccupancyGrid &local_costmap_msg);
};

#endif  // PEDSIM_DATA_RECORDER_HPP_