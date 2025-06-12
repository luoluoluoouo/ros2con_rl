#include "rl_topic/rl_topic.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <yaml-cpp/yaml.h>

namespace rl_topic
{

RLQuadrupedController::RLQuadrupedController() = default;
using config_type = controller_interface::interface_configuration_type;

controller_interface::InterfaceConfiguration RLQuadrupedController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto& joint_name : joint_names_)
  {
      for (const auto& interface_type : command_interface_types_)
      {
          if (!command_prefix_.empty())
          {
              conf.names.push_back(command_prefix_ + "/" + joint_name + "/" + interface_type);
          }
          else
          {
              conf.names.push_back(joint_name + "/" + interface_type);
          }
      }
  }

  return conf;
}

controller_interface::InterfaceConfiguration RLQuadrupedController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto& joint_name : joint_names_)
  {
      for (const auto& interface_type : state_interface_types_)
      {
          conf.names.push_back(joint_name + "/" + interface_type);
      }
  }

  for (const auto& interface_type : imu_interface_types_)
  {
      conf.names.push_back(imu_name_ + "/" + interface_type);
  }

  for (const auto& interface_type : foot_force_interface_types_)
  {
      conf.names.push_back(foot_force_name_ + "/" + interface_type);
  }

  return conf;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RLQuadrupedController::on_init()
{
  try
  {
    auto node = get_node();

    policy_path_ = auto_declare<std::string>("policy_path", "");
    config_path_ = auto_declare<std::string>("config_path", "");

    cmd_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg)
      {
        cmd_[0] = msg->linear.x;
        cmd_[1] = msg->linear.y;
        cmd_[2] = msg->angular.z;
      });

    // string: sit, stand, move  
    mode_sub_ = node->create_subscription<std_msgs::msg::String>(
      "/mode", 10, [this](const std_msgs::msg::String::SharedPtr msg)
      {
        mode_ = msg->data;
      });

    joint_pub_ = node->create_publisher<sensor_msgs::msg::JointState>(
      "/cmd/joint_states", 10);

    pidgain_pub_ = node->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/pid_gain", 10);    

    joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
    feet_names_ = auto_declare<std::vector<std::string>>("feet_names", feet_names_);
    command_interface_types_ =
        auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
    state_interface_types_ =
        auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

    command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
    base_name_ = auto_declare<std::string>("base_name", base_name_);

    // imu sensor
    imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
    imu_interface_types_ = auto_declare<std::vector<std::string>>("imu_interfaces", state_interface_types_);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init() failed: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RLQuadrupedController::on_configure(const rclcpp_lifecycle::State &)
{
  try
  {
    auto node = get_node();

    policy_ = torch::jit::load(policy_path_);
    YAML::Node config = YAML::LoadFile(config_path_);

    initial_angles_ = config["initial_angles"].as<std::vector<float>>();
    sit_angles_ = config["sit_angles"].as<std::vector<float>>();
    default_angles_ = config["default_angles"].as<std::vector<float>>();

    action_scale_ = config["action_scale"].as<float>();
    cmd_scale_ = config["cmd_scale"].as<std::vector<float>>();
    ang_vel_scale_ = config["ang_vel_scale"].as<float>();
    dof_pos_scale_ = config["dof_pos_scale"].as<float>();
    dof_vel_scale_ = config["dof_vel_scale"].as<float>();

    cmd_ = config["cmd_init"].as<std::vector<float>>();

    obs_buf_size_ = config["obs_buffer_size"].as<int>();
    one_step_obs_size_ = config["one_step_obs_size"].as<int>();

    obs_buffer_ = torch::zeros({1, obs_buf_size_ * one_step_obs_size_});
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "on_configure() failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RLQuadrupedController::on_activate(const rclcpp_lifecycle::State &)
{
  try
  { 
    // clear out vectors in case of restart
    ctrl_interfaces_.clear();

    // assign command interfaces
    for (auto& interface : command_interfaces_)
    {
        std::string interface_name = interface.get_interface_name();
        if (const size_t pos = interface_name.find('/'); pos != std::string::npos)
        {
            command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
        }
        else
        {
            command_interface_map_[interface_name]->push_back(interface);
        }
    }

    // assign state interfaces
    for (auto& interface : state_interfaces_)
    {
        if (interface.get_prefix_name() == imu_name_)
        {
            ctrl_interfaces_.imu_state_interface_.emplace_back(interface);
        }
        else if (interface.get_prefix_name() == foot_force_name_)
        {
            ctrl_interfaces_.foot_force_state_interface_.emplace_back(interface);
        }
        else
        {
            state_interface_map_[interface.get_interface_name()]->push_back(interface);
        }
    }

    current_pos_ = get_current_pos();
    // for (int i = 0; i < 12; ++i)
    // {
    //   ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(current_pos_[i]);
    //   ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0.0);
    //   ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0.0);
    //   ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(13.0);
    //   ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(0.4);
    // }

    start_time_ = std::chrono::high_resolution_clock::now();
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "on_activate() failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}
std::chrono::steady_clock::time_point last_time_;

controller_interface::return_type RLQuadrupedController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  // running_time_ = std::chrono::duration<float>(std::chrono::high_resolution_clock::now() - start_time_).count();

  // if (running_time_ < 5.0) {
  //   sensor_msgs::msg::JointState joint_state;
  //   joint_state.header.stamp = get_node()->now();
  //   joint_state.name = topic_joint_names_;
  //   joint_state.position = std::vector<double>(initial_angles_.begin(), initial_angles_.end());
  //   joint_pub_->publish(joint_state);

  //   std_msgs::msg::Float32MultiArray pid_gain;
  //   pid_gain.data =  {13.0, 0.4}; // Kp, Kd
  //   pidgain_pub_->publish(pid_gain);
  //   return controller_interface::return_type::OK;
  // }

  // auto now = std::chrono::steady_clock::now();
  // if (last_time_.time_since_epoch().count() != 0) {
  //   auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_time_);
  //   double frequency = 1.0 / duration.count();  // 計算頻率（Hz）
  //   std::cout << "Update frequency: " << frequency << " Hz" << std::endl;
  // }
  // last_time_ = now;

  if (mode_ != prev_mode_) {
    is_mode_change_ = true;
    step_ = 0;
    current_pos_ = get_current_pos();
    prev_mode_ = mode_;
  } else {
    is_mode_change_ = false;
  }

  if (mode_ == "sit") {
    sit(step_, current_pos_);
  } else if (mode_ == "stand") {
    stand(step_, current_pos_);
  } else if (mode_ == "move") {
    move();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid mode: %s", mode_.c_str());
  }

  if (step_ < steps_) {
    step_++;
  }

  return controller_interface::return_type::OK;
}

std::vector<float> RLQuadrupedController::get_current_pos()
{
  std::vector<float> pos(12);
  for (int i = 0; i < 12; ++i)
  {
    pos[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
  }
  return pos;
}

void RLQuadrupedController::sit(int step, std::vector<float> current_pos) 
{
  if (step < steps_) {
    std::vector<float> target_pos(12);

    double phase = float(step)/float(steps_);
    for (int i = 0; i < 12; ++i)
    {
      target_pos[i] = current_pos[i] * float(1 - phase) + sit_angles_[i] * phase;
    }
    target_pos = convert_joint_angles(target_pos);

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = get_node()->now();
    joint_state.name = topic_joint_names_;
    joint_state.position = std::vector<double>(target_pos.begin(), target_pos.end());
    joint_pub_->publish(joint_state);

    std_msgs::msg::Float32MultiArray pid_gain;
    pid_gain.data =  {5.0, 0.4}; // Kp, Kd
    pidgain_pub_->publish(pid_gain);

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(step_time_ * 1000)));
  } else {
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = get_node()->now();
    joint_state.name = topic_joint_names_;
    std::vector<float> sit_angles = convert_joint_angles(sit_angles_);
    joint_state.position = std::vector<double>(sit_angles.begin(), sit_angles.end());
    joint_pub_->publish(joint_state);

    std_msgs::msg::Float32MultiArray pid_gain;
    pid_gain.data =  {13.0, 0.4}; // Kp, Kd
    pidgain_pub_->publish(pid_gain);
  }
}

void RLQuadrupedController::stand(int step, std::vector<float> current_pos) 
{
  if (step < steps_) {
    std::vector<float> target_pos(12);

    double phase = float(step)/float(steps_);
    for (int i = 0; i < 12; ++i)
    {
      target_pos[i] = current_pos[i] * float(1 - phase) + default_angles_[i] * phase;
    }
    target_pos = convert_joint_angles(target_pos);

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = get_node()->now();
    joint_state.name = topic_joint_names_;
    joint_state.position = std::vector<double>(target_pos.begin(), target_pos.end());
    joint_pub_->publish(joint_state);

    std_msgs::msg::Float32MultiArray pid_gain;
    pid_gain.data =  {13.0, 0.4}; // Kp, Kd
    pidgain_pub_->publish(pid_gain);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(step_time_ * 1000)));
  } else {
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = get_node()->now();
    joint_state.name = topic_joint_names_;
    std::vector<float> default_angles = convert_joint_angles(default_angles_);
    joint_state.position = std::vector<double>(default_angles.begin(), default_angles.end());
    joint_pub_->publish(joint_state);

    std_msgs::msg::Float32MultiArray pid_gain;
    pid_gain.data =  {13.0, 0.4}; // Kp, Kd
    pidgain_pub_->publish(pid_gain);
  }
}

void RLQuadrupedController::move()
{
  std::vector<float> pos(12), vel(12), ang_vel(3), quat(4);

  for (int i = 0; i < 12; ++i)
  {
    pos[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
  }

  quat[0] = ctrl_interfaces_.imu_state_interface_[0].get().get_value();
  quat[1] = ctrl_interfaces_.imu_state_interface_[1].get().get_value();
  quat[2] = ctrl_interfaces_.imu_state_interface_[2].get().get_value();
  quat[3] = ctrl_interfaces_.imu_state_interface_[3].get().get_value();
  ang_vel[0] = ctrl_interfaces_.imu_state_interface_[4].get().get_value();
  ang_vel[1] = ctrl_interfaces_.imu_state_interface_[5].get().get_value();
  ang_vel[2] = ctrl_interfaces_.imu_state_interface_[6].get().get_value();

  std::vector<float> gravity(3);
  gravity[0] = 2 * (quat[1] * quat[3] - quat[0] * quat[2]);
  gravity[1] = -2 * (quat[2] * quat[3] + quat[0] * quat[1]);
  gravity[2] = -1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2]);

  std::vector<torch::Tensor> obs_parts = {
    torch::tensor(latest_cmd_) * torch::tensor(cmd_scale_),
    torch::tensor(ang_vel) * ang_vel_scale_,
    torch::tensor(gravity),
    (torch::tensor(pos) - torch::tensor(default_angles_)) * dof_pos_scale_,
    torch::tensor(vel) * dof_vel_scale_,
    prev_action_
  };

  auto obs = torch::cat(obs_parts).unsqueeze(0);
  obs = torch::clamp(obs, -100, 100);
  obs_buffer_ = torch::cat({obs, obs_buffer_.slice(1, 0, obs_buf_size_ * one_step_obs_size_ - one_step_obs_size_)}, 1);

  auto action = policy_.forward({obs_buffer_}).toTensor().squeeze();
  prev_action_ = action;

  /// 假設 action 是 float tensor
  std::vector<float> action_vec(action.data_ptr<float>(), action.data_ptr<float>() + action.numel());

  // 轉換順序與符號
  std::vector<float> converted_action = convert_joint_angles(action_vec);

  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = get_node()->now();
  joint_state.name = topic_joint_names_;
  joint_state.position = {};
  for (int i = 0; i < 12; ++i)
  {
    joint_state.position .push_back(default_angles_[i] + converted_action[i] * action_scale_);
  }
  joint_pub_->publish(joint_state);

  std_msgs::msg::Float32MultiArray pid_gain;
  pid_gain.data =  {13.0, 0.4}; // Kp, Kd
  pidgain_pub_->publish(pid_gain);

  for (int i = 0; i < 3; ++i)
  {
    latest_cmd_[i] = cmd_[i];
  }
}

std::vector<float> RLQuadrupedController::convert_joint_angles(const std::vector<float>& dof_pos) {
    if (dof_pos.size() != 12) {
        throw std::invalid_argument("Expected 12 joint angles in dof_pos.");
    }

    // 定義格式順序
    std::vector<std::string> mapping = {
        "flh", "frh", "rlh", "rrh",  // Hips
        "flu", "fru", "rlu", "rru",  // Upper legs
        "fld", "frd", "rld", "rrd"   // Lower legs
    };

    std::vector<std::string> format1_order = {
        "flh", "flu", "fld",
        "frh", "fru", "frd",
        "rlh", "rlu", "rld",
        "rrh", "rru", "rrd"
    };

    // 建立 mapping 到 index 的對應
    std::unordered_map<std::string, int> name_to_index;
    for (size_t i = 0; i < format1_order.size(); ++i) {
        name_to_index[format1_order[i]] = static_cast<int>(i);
    }

    // 依照 mapping 順序取值
    std::vector<float> reordered_dof_pos;
    for (const auto& name : mapping) {
        reordered_dof_pos.push_back(dof_pos[name_to_index[name]]);
    }

    // 對應的符號變換
    std::vector<float> result = {
        reordered_dof_pos[0],  reordered_dof_pos[1], -reordered_dof_pos[2], -reordered_dof_pos[3],
        reordered_dof_pos[4], -reordered_dof_pos[5],  reordered_dof_pos[6], -reordered_dof_pos[7],
        reordered_dof_pos[8], -reordered_dof_pos[9],  reordered_dof_pos[10], -reordered_dof_pos[11]
    };

    return result;
}

} // namespace rl_topic

PLUGINLIB_EXPORT_CLASS(rl_topic::RLQuadrupedController, controller_interface::ControllerInterface)
