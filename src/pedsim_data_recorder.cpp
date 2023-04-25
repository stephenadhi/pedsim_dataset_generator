#include "pedsim_data_recorder.hpp"

PedsimDataRecorder::PedsimDataRecorder()
: Node("pedsim_data_recorder")
{
  // Set up subscribers
  // TODO: implement subscribers

  std::ofstream data_file;
  data_file.open("dataset.txt");
  record_bag({"/pedsim_simulator/simulated_agents", "local_costmap/costmap", "global_costmap/costmap"});
}

void PedsimDataRecorder::agents_callback(const pedsim_msgs::msg::AgentStates &agents_msg)
{
  write_data_to_txt_file(agents_msg, local_costmap_msg);
}

void PedsimDataRecorder::write_data_to_txt_file(const pedsim_msgs::msg::AgentStates &agents_msg,
                                        const nav_msgs::msg::OccupancyGrid &local_costmap_msg)
{
  for (const auto &agent : agents_msg.agent_states) { 
    bool occ_flag = check_agent_in_local_costmap(agent, local_costmap_msg);
    std::vector<int8_t> costmap_data = {};
    if (agent.id == ego_agent_id) { 
      costmap_data = local_costmap_msg.data;
      continue; 
      }
    int agent_id = agent.id;
    int type = agent.type;
    std::string social_state = agent.social_state;
    double pos_x = agent.pose.position.x;
    double pos_y = agent.pose.position.y;
    double lin_vel_x = agent.twist.linear.x;
    double lin_vel_y = agent.twist.linear.y;
    double ang_vel_z = agent.twist.angular.z;
    

  }
}

bool PedsimDataRecorder::check_agent_in_local_costmap(const pedsim_msgs::msg::AgentState &agent,
                                              const nav_msgs::msg::OccupancyGrid &local_costmap_msg)
{
  double agent_x = agent.pose.position.x;
  double agent_y = agent.pose.position.y;

  double map_origin_x = local_costmap_msg.info.origin.position.x;
  double map_origin_y = local_costmap_msg.info.origin.position.y;
  double map_resolution = local_costmap_msg.info.resolution;

  int map_width = local_costmap_msg.info.width;
  int map_height = local_costmap_msg.info.height;

  int grid_x = (int)((agent_x - map_origin_x) / map_resolution);
  int grid_y = (int)((agent_y - map_origin_y) / map_resolution);

  if (grid_x >= 0 && grid_x < map_width && grid_y >= 0 && grid_y < map_height)
  {
    RCLCPP_INFO(this->get_logger(), "Agent %li is inside the local costmap", agent.id);
    return true;
  }
  else { return false; }
}

void PedsimDataRecorder::record_bag(const std::vector<std::string> &topics)
{
  // Set up storage options
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = "ros2_bag";
  storage_options.storage_id = "sqlite3";

  // Set up record options
  rosbag2_transport::RecordOptions record_options;
  record_options.all = false;
  record_options.topics = topics;
  record_options.rmw_serialization_format = "cdr";
  record_options.node_prefix = "pedsim";

  // Create recorder
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
      std::make_shared<rosbag2_cpp::writers::SequentialWriter>(),
      shared_from_this(),
      storage_options,
      record_options);

  // Start recording
  recorder->record();

  // Stop recording after an hour
  rclcpp::sleep_for(std::chrono::hours(1));
  recorder->cancel();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PedsimDataRecorder>());
  rclcpp::shutdown();
  return 0;
}