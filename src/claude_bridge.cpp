#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MyArmHardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    RCLCPP_INFO(rclcpp::get_logger("MyArmHardware"), "Initializing hardware interface");

    // Initialize joint vectors
    size_t n_joints = info_.joints.size();
    positions_.resize(n_joints, 0.0);
    velocities_.resize(n_joints, 0.0);
    commands_.resize(n_joints, 0.0);

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      state_interfaces.emplace_back(info_.joints[i].name,
                                    hardware_interface::HW_IF_POSITION,
                                    &positions_[i]);
      state_interfaces.emplace_back(info_.joints[i].name,
                                    hardware_interface::HW_IF_VELOCITY,
                                    &velocities_[i]);
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      command_interfaces.emplace_back(info_.joints[i].name,
                                      hardware_interface::HW_IF_POSITION,
                                      &commands_[i]);
    }
    return command_interfaces;
  }

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    // Read joint states from hardware or sensors here
    // For demo, we just mirror commands
    for (size_t i = 0; i < positions_.size(); ++i)
      positions_[i] = commands_[i];
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    // Send commands_ to hardware here (e.g., via Dynamixel SDK)
    // Also publish for monitoring
    if (!traj_pub_)
      traj_pub_ = rclcpp::Node::make_shared("hardware_pub_node")->create_publisher<std_msgs::msg::Float64MultiArray>("trajectory_points", 10);

    std_msgs::msg::Float64MultiArray msg;
    msg.data = commands_;
    traj_pub_->publish(msg);

    // Print debug info
    RCLCPP_INFO(rclcpp::get_logger("MyArmHardware"), "Publishing command point:");
    for (size_t i = 0; i < commands_.size(); ++i)
      RCLCPP_INFO(rclcpp::get_logger("MyArmHardware"), "  Joint[%zu] = %.3f", i, commands_[i]);

    return hardware_interface::return_type::OK;
  }

private:
  std::vector<double> positions_, velocities_, commands_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr traj_pub_;
};

PLUGINLIB_EXPORT_CLASS(MyArmHardware, hardware_interface::SystemInterface)