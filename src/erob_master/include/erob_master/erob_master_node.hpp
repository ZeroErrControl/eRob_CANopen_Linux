#ifndef EROB_MASTER_NODE_HPP
#define EROB_MASTER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>

// FSM States
enum class FSMState {
    SIBAL,
    INIT,
    READY, // Pre-operational
    OP,    // Operational
    STOP,
    EXIT
};

// COB Types
enum class COBType 
{
    NMT,
    SYNC,
    TS,     // Time Stamp
    EMCY,   // Emergency
    TPDO1,
    RPDO1,
    TPDO2,
    RPDO2,
    TPDO3,
    RPDO3,
    TPDO4,
    RPDO4,
    TSDO,
    RSDO,
    NG,    // Node Guarding
    UNKNOWN
};

enum class SYNCAction
{
    DISABLE_SYNC,
    ENABLE_SYNC,
    RESET_AND_ENABLE_DEVICE,
    STOP_DEVICE,
    PDO_CONTROL,
    TERMINATE
};

enum class DeviceStatus
{
    FAULT,
    NOT_READY_TO_SWITCH_ON,
    QUICK_STOP_ACTIVE,
    SWITCH_ON_DISABLED,
    READY_TO_SWITCH_ON,
    SWITCHED_ON,
    OPERATION_ENABLED
};

// CAN message structure
struct CANMessage
{
    uint32_t    can_id;
    uint8_t     can_dlc;
    uint8_t     data[8];
    COBType     cob_type;
    std::string description;
};

union VelocityData // BE
{
    int32_t velocity_uu;
    uint8_t bytes[4]; 
};

union PositionData // BE
{
    int32_t position_uu;
    uint8_t bytes[4]; 
};

// CAN device
struct CANDevice
{
    int id;
    std::string name;
    std::string type;
    DeviceStatus status;
    bool set_point;
    VelocityData target_velocity;
    VelocityData actual_velocity;
    PositionData target_position;
    PositionData actual_position;
};



class ERobMasterNode : public rclcpp::Node 
{
public:
    using SharedPtr = std::shared_ptr<ERobMasterNode>;
    
    ERobMasterNode();
    ~ERobMasterNode();
    void siginit_exit();
private:
    // methods
    void init_params();
    void init_can_devices(const std::string &config_file);
    void init_can_master_socket();
    void init_sync_timer();
    bool transit_to_ready();
    bool transit_to_op();
    bool transit_to_stop();
    bool transit_to_init(bool first_init = true);
    bool transit_to_exit(bool transition_error = false);
    bool send_can_message(const CANMessage &message, uint8_t node_id);
    bool send_and_check_can_message(const CANMessage &message, uint8_t node_id);
    bool check_pdo_message();
    void publish_actual_values();
    COBType hex_to_cobtype(uint32_t cob_id) const;
    std::string state_to_string(FSMState state) const;
    std::string cobtype_to_string(COBType cob_type) const;
    std::string format_emergency_message(const struct can_frame &response) const;
    uint32_t get_expected_response_id(const CANMessage &message, uint8_t node_id) const;
    float mps_to_uu(float velocity);
    float uu_to_mps(int32_t velocity_uu);
    float rad_to_uu(float position);
    float uu_to_rad(int32_t position_uu);
    float rad_to_deg(float radian);
    float deg_to_rad(float degree);
    // service callbacks
    void handle_start(const std_srvs::srv::Trigger::Request::SharedPtr request,
                      std_srvs::srv::Trigger::Response::SharedPtr response);
    void handle_exit(const std_srvs::srv::Trigger::Request::SharedPtr request,
                     std_srvs::srv::Trigger::Response::SharedPtr response);
    void handle_stop(const std_srvs::srv::Trigger::Request::SharedPtr request,
                     std_srvs::srv::Trigger::Response::SharedPtr response);
    void handle_recover(const std_srvs::srv::Trigger::Request::SharedPtr request,
                        std_srvs::srv::Trigger::Response::SharedPtr response);
    void handle_reset(const std_srvs::srv::Trigger::Request::SharedPtr request,
                      std_srvs::srv::Trigger::Response::SharedPtr response);
    // subscriber callback
    void target_velocity_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void target_position_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    // timer callback
    void sync_callback();
    void call_exit_service();
    // publishers
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr actual_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr actual_position_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr erob_status_pub_;
    // subscribers
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_position_sub_;
    // services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr exit_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recover_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    // callback groups
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
    rclcpp::CallbackGroup::SharedPtr sync_cb_group_;
    rclcpp::CallbackGroup::SharedPtr subscribe_position_cb_group_;
    rclcpp::CallbackGroup::SharedPtr subscribe_velocity_cb_group_;
    // parameters
    std::string type_;
    std::string can_interface_;
    std::string can_config_file_;
    int32_t resolution_position_;
    int32_t resolution_velocity_;
    float max_position_;
    float min_position_;
    float wheel_radius_;
    float max_velocity_;
    // variables
    FSMState current_state_;
    std::mutex sync_mutex_;
    std::condition_variable sync_cv_;
    std::vector<CANDevice> can_devices_;
    rclcpp::TimerBase::SharedPtr sync_timer_;
    rclcpp::Time sync_start_time_;
    SYNCAction sync_action_;
    int sync_interval_ms_;
    int sync_count_;
    int can_socket_;
    bool erob_state_;
    // 添加测试函数声明
    void test_motor_motion();
    // 添加测试定时器声明
    rclcpp::TimerBase::SharedPtr test_timer_;
};

#endif // EROB_MASTER_NODE_HPP 