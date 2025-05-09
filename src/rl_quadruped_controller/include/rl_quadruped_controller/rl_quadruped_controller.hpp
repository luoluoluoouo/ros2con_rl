#pragma once

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <torch/script.h>
#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

namespace rl_quadruped_controller
{

class RLQuadrupedController : public controller_interface::ControllerInterface
{
public:
  RLQuadrupedController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces_;
  std::vector<hardware_interface::LoanedStateInterface> state_interfaces_;

  std::vector<float> latest_cmd_{0.0, 0.0, 0.0};
  std::vector<float> default_angles_, cmd_scale_;
  float action_scale_{1.0}, ang_vel_scale_{1.0}, dof_pos_scale_{1.0}, dof_vel_scale_{1.0};

  torch::jit::script::Module policy_;
  torch::Tensor obs_buffer_;
  torch::Tensor prev_action_ = torch::zeros({12});

  int one_step_obs_size_{45}, obs_buf_size_{6};
};

}  // namespace rl_quadruped_controller
