#include "rl_quadruped_controller/rl_quadruped_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <yaml-cpp/yaml.h>

namespace rl_quadruped_controller
{

RLQuadrupedController::RLQuadrupedController() = default;

controller_interface::InterfaceConfiguration RLQuadrupedController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, {}};
}

controller_interface::InterfaceConfiguration RLQuadrupedController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, {}};
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RLQuadrupedController::on_init()
{
  try
  {
    auto node = get_node();
    
    std::string policy_path;
    std::string config_path;

    node->get_parameter_or<std::string>("policy_path", policy_path, "");
    node->get_parameter_or<std::string>("config_path", config_path, "");


    cmd_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg)
      {
        latest_cmd_[0] = msg->linear.x;
        latest_cmd_[1] = msg->linear.y;
        latest_cmd_[2] = msg->angular.z;
      });
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

    std::string policy_path;
    if (!node->get_parameter("policy_path", policy_path) || policy_path.empty()) {
      RCLCPP_ERROR(node->get_logger(), "policy_path parameter not set or empty!");
      return CallbackReturn::FAILURE;
    }

    std::string config_path;
    if (!node->get_parameter("config_path", config_path) || config_path.empty()) {
      RCLCPP_ERROR(node->get_logger(), "config_path parameter not set or empty!");
      return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(node->get_logger(), "Loaded policy_path: %s", policy_path.c_str());
    RCLCPP_INFO(node->get_logger(), "Loaded config_path: %s", config_path.c_str());

    policy_ = torch::jit::load(policy_path);
    YAML::Node config = YAML::LoadFile(config_path);

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
