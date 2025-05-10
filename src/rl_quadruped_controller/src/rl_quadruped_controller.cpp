#include "rl_quadruped_controller/rl_quadruped_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <yaml-cpp/yaml.h>

namespace rl_quadruped_controller
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
              conf.names.push_back(command_prefix_ + "/" + joint_name + "/" += interface_type);
          }
          else
          {
              conf.names.push_back(joint_name + "/" += interface_type);
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
          conf.names.push_back(joint_name + "/" += interface_type);
      }
  }

  for (const auto& interface_type : imu_interface_types_)
  {
      conf.names.push_back(imu_name_ + "/" += interface_type);
  }

  for (const auto& interface_type : foot_force_interface_types_)
  {
      conf.names.push_back(foot_force_name_ + "/" += interface_type);
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
        latest_cmd_[0] = msg->linear.x;
        latest_cmd_[1] = msg->linear.y;
        latest_cmd_[2] = msg->angular.z;
      });

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

    action_scale_ = config["action_scale"].as<float>();
    default_angles_ = config["default_angles"].as<std::vector<float>>();
    cmd_scale_ = config["cmd_scale"].as<std::vector<float>>();
    ang_vel_scale_ = config["ang_vel_scale"].as<float>();
    dof_pos_scale_ = config["dof_pos_scale"].as<float>();
    dof_vel_scale_ = config["dof_vel_scale"].as<float>();

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
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "on_activate() failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type RLQuadrupedController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::vector<float> pos(12), vel(12), ang_vel(3), quat(4);

  for (int i = 0; i < 12; ++i)
  {
    pos[i] = state_interfaces_[i].get_value();
    vel[i] = state_interfaces_[i + 12].get_value();
  }
  for (int i = 0; i < 4; ++i)
    quat[i] = state_interfaces_[24 + i].get_value();
  for (int i = 0; i < 3; ++i)
    ang_vel[i] = state_interfaces_[28 + i].get_value();

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

  for (int i = 0; i < 12; ++i)
  {
    double cmd = action[i].item<float>() * action_scale_ + default_angles_[i];
    command_interfaces_[i].set_value(cmd);
  }

  RCLCPP_INFO(get_node()->get_logger(), "Action[0]=%.3f", action[0].item<float>());

  return controller_interface::return_type::OK;
}

} // namespace rl_quadruped_controller

PLUGINLIB_EXPORT_CLASS(rl_quadruped_controller::RLQuadrupedController, controller_interface::ControllerInterface)
