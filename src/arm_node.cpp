#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "tf2_ros/transform_broadcaster.h"

#define MX_PRESENT_POSITION 36
#define MX_GOAL_POSITION    30
#define MX_MOVING_SPEED     32
#define MX_TORQUE_ENABLE    24
#define MX_PRESENT_VOLTAGE  42
#define MX_MAX_TORQUE       14
#define MX_TORQUE_LIMIT     34

#define TORQUE_LIMIT_BITS 1000

enum State {
    INIT = 0,
    WRITE_ARM_POSITION,
    WRITE_GRIPPER_POSITION,
    WAIT_FOR_GOAL_REACHED,
    FINISH_TASK
};

class ArmNode : public rclcpp::Node {
public:

    ArmNode() : Node("arm_node")
    {
        RCLCPP_INFO(this->get_logger(), "INITIALIZING arm_node BY RAFACUELLARR...");
        
        if(!declare_and_get_parameters() || !initialize_dynamixel())
        {
            RCLCPP_ERROR(this->get_logger(), "Initialization failed");
            rclcpp::shutdown();
            return;
        }

        setup_interfaces();
        initialize_messages();

        // Create timer for main loop (50Hz = 20ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ArmNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Arm node initialized successfully");
    }

    ~ArmNode()
    {
        // Return to zero positions on shutdown with a profile

        port_handler_->closePort();
    }

private:
    // ROS2 communication
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_goal_pose_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_goal_gripper_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_torque_gripper_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_q_trajectory_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_current_pose_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_current_gripper_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_goal_reached_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_current_voltage_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Dynamixel hardware interface
    dynamixel::PortHandler* port_handler_;
    dynamixel::PacketHandler* packet_handler_;

    // Configuration parameters (using int directly instead of int64_t conversion)
    std::string port_name_;
    int baudrate_;
    bool torque_enable_;
    int max_speed_;
    std::vector<std::string> joint_names_;
    
    // Servo configuration (unified - no separate arm/gripper in memory)
    std::vector<int> servo_ids_;
    std::vector<double> servo_bits_per_radian_;
    std::vector<int> servo_zeros_;
    std::vector<int> servo_directions_;
    size_t num_arm_servos_;
    size_t num_gripper_servos_;
    
    // Goal tracking
    std::vector<int> goal_pose_bits_;
    std::vector<int> goal_torque_gripper_bits_;
    std::vector<std::vector<int>> goal_trajectory_bits_;
    trajectory_msgs::msg::JointTrajectory goal_trajectory_;
    
    // State flags
    bool new_arm_pose_ = false;
    bool new_trajectory_ = false;
    bool new_gripper_pose_ = false;
    bool new_gripper_torque_ = false;

    // State machine
    State state_ = INIT;
    size_t trajectory_idx_ = 0;
    std::vector<int> current_position_bits_;

    // Messages (reused)
    sensor_msgs::msg::JointState joint_states_;
    std_msgs::msg::Float64MultiArray msg_current_pose_;
    std_msgs::msg::Float64 msg_current_gripper_;
    std_msgs::msg::Float64 msg_voltage_;
    std_msgs::msg::Bool msg_goal_reached_;

    // Dynamixel SDK messages
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    bool declare_and_get_parameters()
    {
        // Declare all parameters with defaults
        this->declare_parameter<std::string>("port", "");
        this->declare_parameter<int>("baudrate", 1000000);
        this->declare_parameter<std::vector<int64_t>>("servo_arm_ids", std::vector<int64_t>{});
        this->declare_parameter<std::vector<double>>("servo_arm_bits_per_radian", std::vector<double>{});
        this->declare_parameter<std::vector<int64_t>>("servo_arm_zeros", std::vector<int64_t>{});
        this->declare_parameter<std::vector<int64_t>>("servo_arm_directions", std::vector<int64_t>{});
        this->declare_parameter<std::vector<int64_t>>("servo_gripper_ids", std::vector<int64_t>{});
        this->declare_parameter<std::vector<double>>("servo_gripper_bits_per_radian", std::vector<double>{});
        this->declare_parameter<std::vector<int64_t>>("servo_gripper_zeros", std::vector<int64_t>{});
        this->declare_parameter<std::vector<int64_t>>("servo_gripper_directions", std::vector<int64_t>{});
        this->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
        this->declare_parameter<bool>("torque_enable", true);
        this->declare_parameter<int>("max_speed", 150);

        // Get parameters
        port_name_ = this->get_parameter("port").as_string();
        if(port_name_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Missing port name parameter");
            return false;
        }

        baudrate_ = this->get_parameter("baudrate").as_int();
        torque_enable_ = this->get_parameter("torque_enable").as_bool();
        max_speed_ = this->get_parameter("max_speed").as_int();
        joint_names_ = this->get_parameter("joint_names").as_string_array();

        // Get arm servo parameters and convert int64_t to int
        auto arm_ids = this->get_parameter("servo_arm_ids").as_integer_array();
        auto arm_zeros = this->get_parameter("servo_arm_zeros").as_integer_array();
        auto arm_directions = this->get_parameter("servo_arm_directions").as_integer_array();
        servo_bits_per_radian_ = this->get_parameter("servo_arm_bits_per_radian").as_double_array();

        // Get gripper servo parameters
        auto gripper_ids = this->get_parameter("servo_gripper_ids").as_integer_array();
        auto gripper_zeros = this->get_parameter("servo_gripper_zeros").as_integer_array();
        auto gripper_directions = this->get_parameter("servo_gripper_directions").as_integer_array();
        auto gripper_bits = this->get_parameter("servo_gripper_bits_per_radian").as_double_array();

        // Store counts for later indexing
        num_arm_servos_ = arm_ids.size();
        num_gripper_servos_ = gripper_ids.size();

        // Combine into unified vectors (arm first, then gripper)
        servo_ids_.reserve(num_arm_servos_ + num_gripper_servos_);
        servo_zeros_.reserve(num_arm_servos_ + num_gripper_servos_);
        servo_directions_.reserve(num_arm_servos_ + num_gripper_servos_);
        
        for(auto id : arm_ids) servo_ids_.push_back(static_cast<int>(id));
        for(auto z : arm_zeros) servo_zeros_.push_back(static_cast<int>(z));
        for(auto d : arm_directions) servo_directions_.push_back(static_cast<int>(d));
        
        for(auto id : gripper_ids) servo_ids_.push_back(static_cast<int>(id));
        for(auto z : gripper_zeros) servo_zeros_.push_back(static_cast<int>(z));
        for(auto d : gripper_directions) servo_directions_.push_back(static_cast<int>(d));
        
        servo_bits_per_radian_.insert(servo_bits_per_radian_.end(), gripper_bits.begin(), gripper_bits.end());

        if(servo_ids_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No servo IDs configured");
            return false;
        }

        // Log configuration
        RCLCPP_INFO(this->get_logger(), "Port: %s @ %d baud", port_name_.c_str(), baudrate_);
        RCLCPP_INFO(this->get_logger(), "Arm servos: %zu, Gripper servos: %zu", num_arm_servos_, num_gripper_servos_);
        RCLCPP_INFO(this->get_logger(), "Max speed: %d, Torque enabled: %s", max_speed_, torque_enable_ ? "Yes" : "No");

        return true;
    }

    bool initialize_dynamixel()
    {
        // Initialize hardware
        port_handler_ = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
        packet_handler_ = dynamixel::PacketHandler::getPacketHandler(1.0);
        
        if (!port_handler_->openPort())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open port %s", port_name_.c_str());
            return false;
        }

        if (!port_handler_->setBaudRate(baudrate_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate");
            return false;
        }

        for (int id : servo_ids_) {
            uint8_t srl = 2;  // respond to all packets
            packet_handler_->write1ByteTxRx(port_handler_, id, 16, srl, &dxl_error);
        }

        // Read initial positions
        current_position_bits_.resize(servo_ids_.size());
        if(!read_current_positions())
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot read initial servo positions");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Initial positions read successfully");

        // Initialize goal positions to current positions
        goal_pose_bits_ = current_position_bits_;
        goal_torque_gripper_bits_.resize(num_gripper_servos_);

        // set torque limits
        std::vector<int> torque_limits(servo_ids_.size());
        for(size_t i = 0; i < servo_ids_.size(); i++)
            torque_limits[i] = TORQUE_LIMIT_BITS;

        if(!write_torque_limits(torque_limits))
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot set torque limits");
            return false;
        }

        // Set moving speed for arm servos
        for(size_t i = 0; i < num_arm_servos_; i++)
        {
            if(packet_handler_->write2ByteTxOnly(port_handler_, servo_ids_[i], MX_MOVING_SPEED, max_speed_) != COMM_SUCCESS)
            {
                RCLCPP_ERROR(this->get_logger(), "Cannot set speed for servo %d", servo_ids_[i]);
                return false;
            }
        }

        // Enable torque
        for (int id : servo_ids_) {
            dxl_comm_result = packet_handler_->write1ByteTxRx(
                port_handler_, id, MX_TORQUE_ENABLE, 1, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
                RCLCPP_ERROR(this->get_logger(), "Failed to enable torque on ID %d", id);
        }

        return true;
    }

    void setup_interfaces()
    {
        // Subscribers
        sub_goal_pose_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/hardware/arm/goal_pose", 10, 
            std::bind(&ArmNode::callback_goal_pose, this, std::placeholders::_1));
        
        sub_goal_gripper_ = this->create_subscription<std_msgs::msg::Float64>(
            "/hardware/arm/goal_gripper", 10,
            std::bind(&ArmNode::callback_goal_gripper, this, std::placeholders::_1));

        // Publishers
        pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        pub_current_pose_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/hardware/arm/current_pose", 10);
        pub_current_gripper_ = this->create_publisher<std_msgs::msg::Float64>("/hardware/arm/current_gripper", 10);
        pub_goal_reached_ = this->create_publisher<std_msgs::msg::Bool>("/hardware/arm/goal_reached", 10);
        pub_current_voltage_ = this->create_publisher<std_msgs::msg::Float64>("/hardware/arm/voltage", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    void initialize_messages()
    {
        msg_current_pose_.data.resize(num_arm_servos_);
        joint_states_.name = joint_names_;
        joint_states_.position = bits_to_radians(current_position_bits_);
    }

    // Callbacks
    void callback_goal_pose(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Only convert arm positions
        for(size_t i = 0; i < num_arm_servos_ && i < msg->data.size(); i++)
        {
            goal_pose_bits_[i] = servo_directions_[i] * msg->data[i] * 
                                servo_bits_per_radian_[i] + servo_zeros_[i];
            // RCLCPP_INFO(this->get_logger(), "Goal pose received. Servo number: %ld, Goal pose bits: %u", i, goal_pose_bits_[i]);
        }
        new_arm_pose_ = true;
    }

    void callback_goal_gripper(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // Set both gripper servos to same position
        for(size_t i = 0; i < num_gripper_servos_; i++)
        {
            size_t idx = num_arm_servos_ + i;
            goal_pose_bits_[idx] = servo_directions_[idx] * msg->data * 
                                  servo_bits_per_radian_[idx] + servo_zeros_[idx];
        }
        new_gripper_pose_ = true;
    }

    void timer_callback()
    {
        // State machine
        switch(state_)
        {
        case INIT:
            if(new_arm_pose_)
                state_ = WRITE_ARM_POSITION;
            else if(new_gripper_pose_)
                state_ = WRITE_GRIPPER_POSITION;
            break;

        case WRITE_ARM_POSITION:
            new_arm_pose_ = false;
            if(!write_goal_positions(0, num_arm_servos_))
                RCLCPP_ERROR(this->get_logger(), "Cannot write arm goal positions");
            state_ = WAIT_FOR_GOAL_REACHED;
            break;

        case WRITE_GRIPPER_POSITION:
            new_gripper_pose_ = false;
            if(!write_goal_positions(num_arm_servos_, num_gripper_servos_))
                RCLCPP_ERROR(this->get_logger(), "Cannot write gripper goal positions");
            state_ = WAIT_FOR_GOAL_REACHED;
            break;

        case WAIT_FOR_GOAL_REACHED:
            state_ = FINISH_TASK;
            break;

        case FINISH_TASK:
            RCLCPP_INFO(this->get_logger(), "Task finished");
            msg_goal_reached_.data = true;
            pub_goal_reached_->publish(msg_goal_reached_);
            state_ = INIT;
            break;
        }

        // Read and publish current state
        if(read_current_positions())
        {
            joint_states_.header.stamp = this->now();
            joint_states_.position = bits_to_radians(current_position_bits_);
            
            // Publish joint states
            pub_joint_state_->publish(joint_states_);
            
            // Publish arm pose
            for(size_t i = 0; i < num_arm_servos_; i++)
                msg_current_pose_.data[i] = joint_states_.position[i];
            pub_current_pose_->publish(msg_current_pose_);
            
            // Publish gripper position (average of gripper servos)
            if(num_gripper_servos_ > 0)
            {
                msg_current_gripper_.data = joint_states_.position[num_arm_servos_] * 
                                           servo_directions_[num_arm_servos_];
                pub_current_gripper_->publish(msg_current_gripper_);
            }
        }
        
        pub_current_voltage_->publish(msg_voltage_);
    }

    // Hardware communication helpers
    bool read_current_positions() {
        for(size_t i = 0; i < servo_ids_.size(); i++)
        {
            dxl_comm_result = packet_handler_->read2ByteTxRx(
                port_handler_,
                (uint8_t) servo_ids_[i],
                MX_PRESENT_POSITION,
                reinterpret_cast<uint16_t *>(&current_position_bits_[i])
            );

            // RCLCPP_INFO(
            //     this->get_logger(),
            //     "Get [ID: %d] [Present Position: %d]",
            //     servo_ids_[i],
            //     current_position_bits_[i]
            // );
        }
        return true;
    }

    bool write_goal_positions(size_t start_idx, size_t count)
    {
        for(size_t i = start_idx; i < start_idx + count && i < servo_ids_.size(); i++)
        {
            dxl_comm_result = packet_handler_->write2ByteTxRx(port_handler_, (uint8_t)servo_ids_[i], MX_GOAL_POSITION, goal_pose_bits_[i], &dxl_error);
            if(dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "%s", packet_handler_->getTxRxResult(dxl_comm_result));
                return false;
            } else if (dxl_error != 0) {
                RCLCPP_INFO(this->get_logger(), "%s", packet_handler_->getRxPacketError(dxl_error));
                return false;
            } else {
                RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", servo_ids_[i], goal_pose_bits_[i]);
            }
        }
        return true;
    }

    bool write_torque_limits(std::vector<int> max_torque) {
        for(size_t i = 0; i < servo_ids_.size(); i++) {
            dxl_comm_result = packet_handler_->write2ByteTxRx(port_handler_, (uint8_t)servo_ids_[i], MX_TORQUE_LIMIT, max_torque[i], &dxl_error);
            if(dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "%s", packet_handler_->getTxRxResult(dxl_comm_result));
                return false;
            } else if (dxl_error != 0) {
                RCLCPP_INFO(this->get_logger(), "%s", packet_handler_->getRxPacketError(dxl_error));
                return false;
            } else {
                RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Max torque: %d]", servo_ids_[i], max_torque[i]);
            }
        }
        return true;
    }

    // Conversion helpers
    std::vector<double> bits_to_radians(const std::vector<int>& bits) {
        std::vector<double> radians(bits.size());
        for(size_t i = 0; i < bits.size(); i++)
            radians[i] = servo_directions_[i] * (bits[i] - servo_zeros_[i]) / servo_bits_per_radian_[i];
        return radians;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}