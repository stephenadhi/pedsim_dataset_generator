#ifndef PEDSIM_DATA_RECORDER_HPP_
#define PEDSIM_DATA_RECORDER_HPP_

#include <fstream>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

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
  std::string agents_topic, costmap_topic;
  double max_delay;
  int record_duration;

private:
  bool is_subscribed_;
  double frame_time = 0;
  double time_interval;
  int map_width;
  int map_height;
  double map_origin_x;
  double map_origin_y;
  double map_resolution;
  long unsigned int ego_agent_id; // Agent types 0, 1 -> ordinary agents 2 -> Robot 3 -> standing/elderly agents
  std::ofstream data_file; //Txt file to store data

  std::unique_ptr<rosbag2_cpp::Writer> rosbag_riter_;

  message_filters::Subscriber<pedsim_msgs::msg::AgentStates> agents_sub_;
  message_filters::Subscriber<nav_msgs::msg::OccupancyGrid> costmap_sub_;

  typedef message_filters::sync_policies::ApproximateTime<pedsim_msgs::msg::AgentStates, nav_msgs::msg::OccupancyGrid>  MyApproxSyncPolicy;
  message_filters::Synchronizer<MyApproxSyncPolicy> * approxSync_;
  
  rclcpp::TimerBase::SharedPtr timer_;

  void common_callback(const pedsim_msgs::msg::AgentStates::ConstSharedPtr& agents_msg,
                const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& costmap_msg);
  
  void write_data_to_txt_file(const pedsim_msgs::msg::AgentStates::ConstSharedPtr& agents_msg,
                                    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& costmap_msg);
  
  void record_bag(const std::vector<std::string> &topics);
  
  bool check_agent_in_local_costmap(double agent_x, double agent_y);
  
  void stop_subscription();
};

#endif  // PEDSIM_DATA_RECORDER_HPP_