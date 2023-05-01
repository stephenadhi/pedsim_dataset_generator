#include "pedsim_data_recorder.hpp"

PedsimDataRecorder::PedsimDataRecorder()
: Node("pedsim_data_recorder"), is_subscribed_(true)
{
  // Declare parameters
  this->declare_parameter<std::string>("agents_topic", "/pedsim_simulator/simulated_agents");
  this->declare_parameter<std::string>("costmap_topic", "/global_costmap/costmap");
  this->declare_parameter<double>("max_delay", 0.05);
  this->declare_parameter<double>("time_interval", 0.2);
  this->declare_parameter<int>("record_duration", 600);
  this->declare_parameter<int>("ego_agent_id", 2);
  this->get_parameter("agents_topic", agents_topic);
  this->get_parameter("costmap_topic", costmap_topic);
  this->get_parameter("time_interval", time_interval);
  this->get_parameter("max_delay", max_delay);
  this->get_parameter("record_duration", record_duration);
  this->get_parameter("ego_agent_id", ego_agent_id);

  // Set up subscribers
  agents_sub_.subscribe(this, agents_topic);
  costmap_sub_.subscribe(this, costmap_topic);

  // create an approximate time synchronizer to synchronize messages from the two topics
  approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(10), agents_sub_, costmap_sub_);
  // approxSync_->setSlop(max_delay); // set the maximum time difference between messages to 0.05 seconds
  approxSync_->registerCallback(std::bind(&PedsimDataRecorder::common_callback, this, std::placeholders::_1, std::placeholders::_2));

  // create a timer to stop the subscription after 600 seconds
  timer_ = create_wall_timer(std::chrono::seconds(record_duration), std::bind(&PedsimDataRecorder::stop_subscription, this));

  data_file.open("dataset.txt");
  data_file << "frame_time" << '\t' << "agent_id" << '\t' << "pos_x" << '\t' << "pos_y" << '\t'  << "lin_vel_x" << '\t' << "lin_vel_y" 
    << '\t' << "ang_vel_z" << '\t' << "agent_type" << '\t'  << "social_state" << '\t' << "occ_flag" << '\t' << "costmap_data" << '\n';
  // record_bag({"/pedsim_simulator/simulated_agents", "local_costmap/costmap"});
}

void PedsimDataRecorder::common_callback(const pedsim_msgs::msg::AgentStates::ConstSharedPtr& agents_msg, 
                                        const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& costmap_msg)
{
  // RCLCPP_INFO(this->get_logger(), "Received %zu agents", agents_msg->agent_states.size());
  // RCLCPP_INFO(this->get_logger(), "Received grid of size %u x %u", costmap_msg->info.width, costmap_msg->info.height);
  if (is_subscribed_) {
    map_width = costmap_msg->info.width;
    map_height = costmap_msg->info.height;
    map_origin_x = costmap_msg->info.origin.position.x;
    map_origin_y = costmap_msg->info.origin.position.y;
    map_resolution = costmap_msg->info.resolution;
    write_data_to_txt_file(agents_msg, costmap_msg);
  }
}

void PedsimDataRecorder::write_data_to_txt_file(const pedsim_msgs::msg::AgentStates::ConstSharedPtr& agents_msg,
                                        const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& costmap_msg)
{
  frame_time += time_interval;  
  for (const auto &agent : agents_msg->agent_states) {
    double pos_x = agent.pose.position.x;
    double pos_y = agent.pose.position.y;
    bool occ_flag = check_agent_in_local_costmap(pos_x, pos_y);
    std::vector<int8_t> costmap_data = {};
    if (agent.id == ego_agent_id) { 
      costmap_data = costmap_msg->data;
    }
    long unsigned int agent_id = agent.id;
    double lin_vel_x = agent.twist.linear.x;
    double lin_vel_y = agent.twist.linear.y;
    double ang_vel_z = agent.twist.angular.z;
    int agent_type = agent.type;
    std::string social_state = agent.social_state;
    // write the new data to the file
    try {
      // RCLCPP_INFO(this->get_logger(), "Writing data to txt file");
      if (data_file.is_open()) {
        data_file << frame_time << '\t' << agent_id << '\t' << pos_x << '\t' << pos_y << '\t'  << lin_vel_x << '\t' 
          << lin_vel_y << '\t' << ang_vel_z << '\t' << agent_type << '\t'  << social_state << '\t' << occ_flag << '\t'; 
        for (int8_t value : costmap_data) {
            data_file << static_cast<int>(value) << ' ';
        }
        data_file << '\n';
      }
    } catch (std::ofstream::failure& e) {
      // if an exception occurs, close the file and rethrow the exception
      data_file.close();
      throw;
    }
    // flush the buffer to ensure that the data is written to the file immediately
    data_file.flush();
    // sleep for a short time to avoid overloading the system
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

bool PedsimDataRecorder::check_agent_in_local_costmap(double agent_x, double agent_y)
{
  int grid_x = (int)((agent_x - map_origin_x) / map_resolution);
  int grid_y = (int)((agent_y - map_origin_y) / map_resolution);
  if (grid_x >= 0 && grid_x < map_width && grid_y >= 0 && grid_y < map_height) {
    return true;
  }
  else { return false; }
}

// void PedsimDataRecorder::record_bag(const std::vector<std::string> &topics)
// {
//   // Set up storage options
//   rosbag2_storage::StorageOptions storage_options;
//   storage_options.uri = "ros2_bag";
//   storage_options.storage_id = "sqlite3";
//   // Set up record options
//   rosbag2_transport::RecordOptions record_options;
//   record_options.all = false;
//   record_options.topics = topics;
//   record_options.rmw_serialization_format = "cdr";
//   record_options.node_prefix = "pedsim";
//   // Create recorder
//   auto recorder = std::make_shared<rosbag2_transport::Recorder>(
//       std::make_shared<rosbag2_cpp::writers::SequentialWriter>(),
//       shared_from_this(),
//       storage_options,
//       record_options);
//   // Start recording
//   recorder->record();
// }

void PedsimDataRecorder::stop_subscription() {
  // stop the subscription and destroy the node
  is_subscribed_ = false;
  data_file.close();
  rclcpp::shutdown();
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PedsimDataRecorder>());
  rclcpp::shutdown();
  return 0;
}