
// #include "rclcpp/rclcpp.hpp"
// #include "rcl_interfaces/srv/set_parameters.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "std_msgs/msg/float32.hpp"
// #include "std_msgs/msg/bool.hpp"
// #include <vector>
// #include <chrono>
// #include <cmath>
// #include <memory>
// #include <functional>
// #include <stdexcept>
// #include <sstream>
// #include "can_interface/caninterface.hpp"

// // Giao diện trừu tượng cho CAN
// class ICANInterface {
// public:
//     virtual bool setOrigin(uint8_t id, uint8_t mode) = 0;
//     virtual bool setPositionSpeed(uint8_t id, float pos, int speed, int accel) = 0;
//     virtual bool setCurrent(uint8_t id, float current) = 0;
//     virtual bool setRPM(uint8_t id, float rpm) = 0;
//     virtual bool receive(uint32_t& id, std::vector<uint8_t>& data) = 0;
//     virtual void decodeMotorData(const std::vector<uint8_t>& data, float& pos, float& vel, 
//                                 float& cur, int8_t& temp, int8_t& err) = 0;
//     virtual ~ICANInterface() = default;
// };

// // Triển khai CANInterface kế thừa ICANInterface
// class CANInterfaceWrapper : public ICANInterface {
// public:
//     CANInterfaceWrapper(const std::string& interface_name) : can_(interface_name) {}

//     bool setOrigin(uint8_t id, uint8_t mode) override {
//         try {
//             can_.setOrigin(id, mode);
//             return true;
//         } catch (const std::exception& e) {
//             std::cerr << "SetOrigin failed: " << e.what() << std::endl;
//             return false;
//         }
//     }

//     bool setPositionSpeed(uint8_t id, float pos, int speed, int accel) override {
//         try {
//             can_.setPositionSpeed(id, pos, speed, accel);
//             return true;
//         } catch (const std::exception& e) {
//             std::cerr << "SetPositionSpeed failed: " << e.what() << std::endl;
//             return false;
//         }
//     }

//     bool setCurrent(uint8_t id, float current) override {
//         try {
//             can_.setCurrent(id, current);
//             return true;
//         } catch (const std::exception& e) {
//             std::cerr << "SetCurrent failed: " << e.what() << std::endl;
//             return false;
//         }
//     }

//     bool setRPM(uint8_t id, float rpm) override {
//         try {
//             can_.setRPM(id, rpm);
//             return true;
//         } catch (const std::exception& e) {
//             std::cerr << "SetRPM failed: " << e.what() << std::endl;
//             return false;
//         }
//     }

//     bool receive(uint32_t& id, std::vector<uint8_t>& data) override {
//         return can_.receive(id, data);
//     }

//     void decodeMotorData(const std::vector<uint8_t>& data, float& pos, float& vel, 
//                          float& cur, int8_t& temp, int8_t& err) override {
//         can_.decodeMotorData(data, pos, vel, cur, temp, err);
//     }

// private:
//     CANInterface can_;
// };

// // Cấu trúc dữ liệu động cơ
// struct MotorData {
//     float position = 0.0;  // degrees
//     float velocity = 0.0;  // rpm
//     float current = 0.0;   // mA
//     int8_t temperature = 0; // °C
//     int8_t error = 0;      // error code
//     std::string state;     // trạng thái động cơ
// };

// // Giao diện điều khiển động cơ
// class IMotorController {
// public:
//     virtual void control(const MotorData& data) = 0;
//     virtual void setParameters(const std::vector<rcl_interfaces::msg::Parameter>& parameters) = 0;
//     virtual bool isOriginSet() const = 0;
//     virtual bool isTargetReached() const = 0;
//     virtual void stop() = 0;
//     virtual void handleCommand(const std::string& command) = 0;
//     virtual uint8_t motorId() const = 0;
//     virtual ~IMotorController() = default;
// };

// // Lớp kiểm tra an toàn
// class SafetyChecker {
// public:
//     SafetyChecker(float min_angle, float max_angle, int max_temperature)
//         : min_angle_(min_angle), max_angle_(max_angle), max_temperature_(max_temperature) {}

//     bool check(const MotorData& data, const rclcpp::Logger& logger) const {
//         if (data.temperature > max_temperature_ || data.error != 0) {
//             RCLCPP_ERROR(logger, "Motor issue! Temp: %d°C, Error: %d", data.temperature, data.error);
//             return false;
//         }
//         if (data.position < min_angle_ || data.position > max_angle_) {
//             RCLCPP_ERROR(logger, "Position %.2f out of bounds [%.2f, %.2f]", 
//                          data.position, min_angle_, max_angle_);
//             return false;
//         }
//         return true;
//     }

// private:
//     float min_angle_;
//     float max_angle_;
//     int max_temperature_;
// };

// // Lớp xuất bản trạng thái
// class StatePublisher {
// public:
//     StatePublisher(rclcpp::Node* node, const std::string& name)
//         : position_publisher_(node->create_publisher<std_msgs::msg::Float32>(name + "_position", 10)),
//           velocity_publisher_(node->create_publisher<std_msgs::msg::Float32>(name + "_vel_actual", 10)),
//           reached_publisher_(node->create_publisher<std_msgs::msg::Bool>(name + "_target_reached", 10)),
//           response_publisher_(node->create_publisher<std_msgs::msg::String>(name + "_response", 10)) {}


//     void publish(const MotorData& data, bool target_reached) {
//         std_msgs::msg::Float32 pos_msg;
//         pos_msg.data = data.position;
//         position_publisher_->publish(pos_msg);

//         std_msgs::msg::Float32 vel_msg;
//         vel_msg.data = data.velocity;
//         velocity_publisher_->publish(vel_msg);

//         std_msgs::msg::Bool reached_msg;
//         reached_msg.data = target_reached;
//         reached_publisher_->publish(reached_msg);
//     }

//     void publishResponse(const std::string& message) {
//         std_msgs::msg::String msg;
//         msg.data = message;
//         response_publisher_->publish(msg);
//     }

// private:
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_publisher_;
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_publisher_;
//     rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_publisher_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_publisher_;
// };

// // Lớp điều khiển động cơ
// class MotorController : public IMotorController {
//     public:
//         MotorController(uint8_t motor_id, std::shared_ptr<ICANInterface> can, 
//                         rclcpp::Node* node, const std::string& name)
//             : motor_id_(motor_id), can_(can), node_(node), logger_(node->get_logger()),
//               state_(State::INITIALIZING), is_origin_set_(false), is_home_(false),
//               current_target_index_(0), reach_counter_(0), target_reached_(true),
//               speed_(10000.0), accel_(1000.0), min_angle_(0.0), max_angle_(90.0),
//               step_(5), target_position_(0.0), reach_count_max_(100),
//               safety_checker_(min_angle_, max_angle_, MAX_MOTOR_TEMPERATURE),
//               state_publisher_(node, name), param_prefix_(name + "."), 
//               last_command_(""), last_command_time_(node_->now()) {
//             initializeParameters();
//             setOrigin();
//             initializeCommandMap(); // Thêm khởi tạo command map
//         }
    
//         void handleCommand(const std::string& command) override {
//             std::stringstream response;
//             response << "Motor 0x" << std::hex << static_cast<int>(motor_id_) << ": ";
        
//             // Kiểm tra điều kiện tiên quyết
//             if (!is_origin_set_) {
//                 response << "Cannot handle command '" << command << "' until origin set!";
//                 RCLCPP_WARN(logger_, "%s", response.str().c_str());
//                 state_publisher_.publishResponse(response.str());
//                 return;
//             }
        
//             if (!is_home_ && last_data_.position == 0.0) {
//                 response << "Position data not initialized, cannot process command '" << command << "'";
//                 RCLCPP_WARN(logger_, "%s", response.str().c_str());
//                 state_publisher_.publishResponse(response.str());
//                 return;
//             }
        
//             // Kiểm tra giới hạn an toàn cho lệnh UP/DOWN
//             if (command == "UP" || command == "DOWN") {
//                 if (last_data_.position < 0.0 || last_data_.position > 90.0) {
//                     response << "Position " << last_data_.position << " out of bounds [0.0, 90.0], stopping";
//                     RCLCPP_ERROR(logger_, "%s", response.str().c_str());
//                     state_publisher_.publishResponse(response.str());
//                     stop();
//                     transitionTo(State::STOPPED);
//                     return;
//                 }
//             }
        
//             // Cập nhật thời gian lệnh cuối
//             last_command_time_ = node_->now();
        
//             // Sử dụng Command Pattern thay vì if-else
//             auto it = command_map_.find(command);
//             if (it != command_map_.end()) {
//                 it->second(); // Gọi lambda function tương ứng
//             } else {
//                 response << "Unknown command: " << command;
//                 RCLCPP_WARN(logger_, "%s", response.str().c_str());
//                 state_publisher_.publishResponse(response.str());
//             }
//         }
        
//         void control(const MotorData& data) override {
//             last_data_ = data;
        
//             if (!safety_checker_.check(data, logger_)) {
//                 stop();
//                 return;
//             }
        
//             state_publisher_.publish(data, target_reached_);
        
//             switch (state_) {
//                 case State::INITIALIZING:
//                     if (std::abs(data.position) < POSITION_TOLERANCE && 
//                         std::abs(data.velocity) < VELOCITY_TOLERANCE) {
//                         is_origin_set_ = true;
//                         transitionTo(State::HOMING);
//                     }
//                     break;
//                 case State::HOMING:
//                     can_->setPositionSpeed(motor_id_, 0.0, static_cast<int>(speed_), static_cast<int>(accel_));
//                     if (std::abs(data.position) < POSITION_TOLERANCE && 
//                         std::abs(data.velocity) < VELOCITY_TOLERANCE) {
//                         is_home_ = true;
//                         target_reached_ = true;
//                         RCLCPP_INFO(logger_, "Motor 0x%02X reached Home Position!", motor_id_);
//                         transitionTo(State::MOVING);
//                     }
//                     break;
//                 case State::MOVING: {
//                     if (target_positions_.empty() && last_command_.empty()) {
//                         RCLCPP_ERROR(logger_, "Motor 0x%02X: No target positions or command!", motor_id_);
//                         stop();
//                         transitionTo(State::STOPPED);
//                         break;
//                     }
//                     if (!last_command_.empty()) {
//                         can_->setPositionSpeed(motor_id_, target_position_, static_cast<int>(speed_), static_cast<int>(accel_));
//                         RCLCPP_INFO(logger_, "Motor 0x%02X cmd chưa empty!", motor_id_);
//                         if (std::abs(data.position - target_position_) < POSITION_TOLERANCE) {
//                             reach_counter_++;
//                             if (reach_counter_ >= reach_count_max_) {
//                                 reach_counter_ = 0;
//                                 target_reached_ = true;
//                                 RCLCPP_INFO(logger_, "Motor 0x%02X: Reached target position %.2f", 
//                                             motor_id_, target_position_);
//                             }
//                         } else {
//                             reach_counter_ = 0;
//                             target_reached_ = false;
//                         }
//                     } else {
//                         float target = target_positions_[current_target_index_];
//                         can_->setPositionSpeed(motor_id_, target, static_cast<int>(speed_), static_cast<int>(accel_));
//                         bool is_reached = std::abs(data.position - target) < POSITION_TOLERANCE;
//                         if (is_reached) {
//                             reach_counter_++;
//                             if (reach_counter_ >= reach_count_max_) {
//                                 reach_counter_ = 0;
//                                 target_reached_ = true;
//                                 current_target_index_++;
//                                 if (current_target_index_ >= target_positions_.size()) {
//                                     static int current_repeat = 0;
//                                     current_repeat++;
//                                     if (current_repeat < repeat_) {
//                                         current_target_index_ = 0;
//                                         RCLCPP_INFO(logger_, "Motor 0x%02X: Repeating target_positions, repeat %d/%d", 
//                                                     motor_id_, current_repeat + 1, repeat_);
//                                     } else {
//                                         current_target_index_ = target_positions_.size() - 1;
//                                         current_repeat = 0;
//                                         RCLCPP_INFO(logger_, "Motor 0x%02X: All targets reached after %d repeats.", 
//                                                     motor_id_, repeat_);
//                                     }
//                                 }
//                             }
//                         } else {
//                             reach_counter_ = 0;
//                             target_reached_ = false;
//                         }
//                     }
//                     break;
//                 }
//                 case State::PAUSED:
//                     can_->setPositionSpeed(motor_id_, target_position_, static_cast<int>(speed_), static_cast<int>(accel_));
//                     if (std::abs(data.position - target_position_) < POSITION_TOLERANCE) {
//                         target_reached_ = true;
//                     }
//                     break;
//                 case State::STOPPED:
//                     stop();
//                     break;
//                 case State::ERROR:
//                     break;
//             }
//         }
        
//         void setParameters(const std::vector<rcl_interfaces::msg::Parameter>& parameters) override {
//             RCLCPP_INFO(logger_, "Motor 0x%02X: Received set_parameters request with %zu parameters", 
//                         motor_id_, parameters.size());
//             if (!is_origin_set_) {
//                 RCLCPP_WARN(logger_, "Motor 0x%02X: Cannot update parameters until origin is set!", motor_id_);
//                 return;
//             }
        
//             for (const auto& param : parameters) {
//                 RCLCPP_INFO(logger_, "Motor 0x%02X: Processing parameter: %s", motor_id_, param.name.c_str());
//                 if (param.name == "target_positions") {
//                     auto temp_positions = param.value.double_array_value;
//                     if (temp_positions.empty()) {
//                         RCLCPP_ERROR(logger_, "Motor 0x%02X: Empty target_positions received!", motor_id_);
//                         return;
//                     }
//                     validatePositions(temp_positions);
//                     target_positions_.resize(temp_positions.size());
//                     std::transform(temp_positions.begin(), temp_positions.end(), 
//                                    target_positions_.begin(),
//                                    [](double x) { return static_cast<float>(x); });
//                     target_reached_ = false;
//                     current_target_index_ = 0;
//                     RCLCPP_INFO(logger_, "Motor 0x%02X: Updated target_positions: %ld targets", 
//                                 motor_id_, temp_positions.size());
//                     if (state_ == State::PAUSED || last_command_.empty()) {
//                         transitionTo(State::MOVING);
//                     }
//                 } else if (param.name == "speed") {
//                     speed_ = static_cast<float>(param.value.double_value);
//                     validateSpeed();
//                     RCLCPP_INFO(logger_, "Motor 0x%02X: Updated speed: %.2f", motor_id_, speed_);
//                 } else if (param.name == "accel") {
//                     accel_ = static_cast<float>(param.value.double_value);
//                     validateAccel();
//                     RCLCPP_INFO(logger_, "Motor 0x%02X: Updated accel: %.2f", motor_id_, accel_);
//                 } else if (param.name == "step") {
//                     step_ = param.value.integer_value;
//                     validateStep();
//                     RCLCPP_INFO(logger_, "Motor 0x%02X: Updated step: %d", motor_id_, step_);
//                 } else if (param.name == "repeat") {
//                     repeat_ = param.value.integer_value;
//                     validateRepeat();
//                     RCLCPP_INFO(logger_, "Motor 0x%02X: Updated repeat: %d", motor_id_, repeat_); 
//                 } else {
//                     RCLCPP_WARN(logger_, "Motor 0x%02X: Unknown parameter: %s", motor_id_, param.name.c_str());
//                 }
//             }
//         }
    
//         bool isOriginSet() const override { return is_origin_set_; }
//         bool isTargetReached() const override { return target_reached_; }
//         uint8_t motorId() const override { return motor_id_; }
    
//         void stop() override {
//             try {
//                 can_->setCurrent(motor_id_, 0.0);
//                 can_->setPositionSpeed(motor_id_, last_data_.position, static_cast<int>(speed_), static_cast<int>(accel_));
//                 last_command_ = "";
//                 target_position_ = last_data_.position;
//                 transitionTo(State::STOPPED);
//             } catch (const std::exception& e) {
//                 RCLCPP_ERROR(logger_, "Motor 0x%02X: Stop failed: %s", motor_id_, e.what());
//                 transitionTo(State::ERROR);
//             }
//         }
    
//     private:
//         enum class State { INITIALIZING, HOMING, MOVING, STOPPED, PAUSED, ERROR };
    
//         // Khởi tạo Command Map với lambda functions
//         void initializeCommandMap() {
//             command_map_ = {
//                 {"UP", [this]() {
//                     std::stringstream response;
//                     response << "Motor 0x" << std::hex << static_cast<int>(motor_id_) << ": ";
                    
//                     target_position_ = last_data_.position + step_;
//                     if (target_position_ >= max_angle_) target_position_ = max_angle_;
//                     last_command_ = "UP";
                    
//                     if (can_->setPositionSpeed(motor_id_, target_position_, static_cast<int>(speed_), static_cast<int>(accel_))) {
//                         response << "Moving UP to position " << target_position_;
//                         target_positions_.clear();
//                         target_positions_.push_back(target_position_);
//                         current_target_index_ = 0;
//                         target_reached_ = false;
//                         RCLCPP_INFO(logger_, "%s", response.str().c_str());
//                         state_publisher_.publishResponse(response.str());
//                         transitionTo(State::MOVING);
//                     } else {
//                         response << "Failed to set position " << target_position_ << " for UP";
//                         RCLCPP_ERROR(logger_, "%s", response.str().c_str());
//                         state_publisher_.publishResponse(response.str());
//                     }
//                 }},
                
//                 {"DOWN", [this]() {
//                     std::stringstream response;
//                     response << "Motor 0x" << std::hex << static_cast<int>(motor_id_) << ": ";
                    
//                     target_position_ = last_data_.position - step_;
//                     if (target_position_ < min_angle_) target_position_ = min_angle_;
//                     last_command_ = "DOWN";
                    
//                     if (can_->setPositionSpeed(motor_id_, target_position_, static_cast<int>(speed_), static_cast<int>(accel_))) {
//                         response << "Moving DOWN to position " << target_position_;
//                         target_positions_.clear();
//                         target_positions_.push_back(target_position_);
//                         current_target_index_ = 0;
//                         target_reached_ = false;
//                         RCLCPP_INFO(logger_, "%s", response.str().c_str());
//                         state_publisher_.publishResponse(response.str());
//                         transitionTo(State::MOVING);
//                     } else {
//                         response << "Failed to set position " << target_position_ << " for DOWN";
//                         RCLCPP_ERROR(logger_, "%s", response.str().c_str());
//                         state_publisher_.publishResponse(response.str());
//                     }
//                 }},
                
//                 {"PAUSE", [this]() {
//                     std::stringstream response;
//                     response << "Motor 0x" << std::hex << static_cast<int>(motor_id_) << ": ";
                    
//                     last_command_ = "";
//                     target_position_ = last_data_.position;
//                     target_positions_.clear();
//                     target_positions_.push_back(target_position_);
//                     current_target_index_ = 0;
//                     target_reached_ = true; // Đặt target_reached_ thành true vì động cơ đã dừng
//                     transitionTo(State::PAUSED);
//                     RCLCPP_INFO(logger_, "Motor 0x%02X: Paused at position %.2f", motor_id_, target_position_);
//                     state_publisher_.publishResponse(response.str() + "Paused at position " + std::to_string(target_position_));
//                 }}
//             };
//         }
    
//         // Các hàm validation và utility khác (giữ nguyên từ code gốc)
//         void validatePositions(const std::vector<double>& positions) {
//             for (double pos : positions) {
//                 if (pos < min_angle_ || pos > max_angle_) {
//                     RCLCPP_ERROR(logger_, "Motor 0x%02X: Target position %.2f out of bounds [%.2f, %.2f]", 
//                                  motor_id_, pos, min_angle_, max_angle_);
//                     throw std::runtime_error("Target position out of bounds");
//                 }
//             }
//         }
    
//         void validateSpeed() {
//             if (speed_ <= 0.0 || speed_ > 50000.0) {
//                 RCLCPP_ERROR(logger_, "Motor 0x%02X: Speed %.2f out of bounds [1.0, 50000.0]", motor_id_, speed_);
//                 throw std::runtime_error("Speed out of bounds");
//             }
//             speed_ = std::round(speed_ * 100.0) / 100.0;
//         }
    
//         void validateAccel() {
//             if (accel_ <= 0.0 || accel_ > 10000.0) {
//                 RCLCPP_ERROR(logger_, "Motor 0x%02X: Accel %.2f out of bounds [1.0, 10000.0]", motor_id_, accel_);
//                 throw std::runtime_error("Accel out of bounds");
//             }
//             accel_ = std::round(accel_ * 100.0) / 100.0;
//         }
    
//         void validateStep() {
//             if (step_ <= 0 || step_ > 90) {
//                 RCLCPP_ERROR(logger_, "Motor 0x%02X: Step %d out of bounds [1, 90]", motor_id_, step_);
//                 throw std::runtime_error("Step out of bounds");
//             }
//         }
    
//         void validateRepeat() {
//             if (repeat_ < 0) {
//                 RCLCPP_ERROR(logger_, "Motor 0x%02X: Repeat %d out of bounds [1, 10]", motor_id_, repeat_);
//                 throw std::runtime_error("Repeat out of bounds");
//             }
//         }
    
//         void transitionTo(State new_state) {
//             if (state_ != new_state) {
//                 RCLCPP_INFO(logger_, "Motor 0x%02X: Transitioning from %d to %d", 
//                             motor_id_, static_cast<int>(state_), static_cast<int>(new_state));
//                 state_ = new_state;
//             }
//         }
    
//         void setOrigin() {
//             try {
//                 RCLCPP_INFO(logger_, "Motor 0x%02X: Setting motor origin...", motor_id_);
//                 if (!can_->setOrigin(motor_id_, 0)) {
//                     throw std::runtime_error("SetOrigin failed");
//                 }
//                 RCLCPP_INFO(logger_, "Motor 0x%02X: SetOrigin completed.", motor_id_);
//             } catch (const std::exception& e) {
//                 RCLCPP_ERROR(logger_, "Motor 0x%02X: SetOrigin failed: %s", motor_id_, e.what());
//                 stop();
//                 throw std::runtime_error("Failed to set motor origin");
//             }
//         }
    
//         void initializeParameters() {
//             node_->declare_parameter(param_prefix_ + "target_positions", std::vector<double>{0.0});
//             node_->declare_parameter(param_prefix_ + "speed", 10000.0);
//             node_->declare_parameter(param_prefix_ + "accel", 1000.0);
//             node_->declare_parameter(param_prefix_ + "min_angle", 0.0);
//             node_->declare_parameter(param_prefix_ + "max_angle", 90.0);
//             node_->declare_parameter(param_prefix_ + "step", 2);
//             node_->declare_parameter(param_prefix_ + "reach_count_max", 100);
//             node_->declare_parameter(param_prefix_ + "repeat", 1);
    
//             std::vector<double> temp_positions;
//             node_->get_parameter(param_prefix_ + "target_positions", temp_positions);
//             node_->get_parameter(param_prefix_ + "speed", speed_);
//             node_->get_parameter(param_prefix_ + "accel", accel_);
//             node_->get_parameter(param_prefix_ + "min_angle", min_angle_);
//             node_->get_parameter(param_prefix_ + "max_angle", max_angle_);
//             node_->get_parameter(param_prefix_ + "step", step_);
//             node_->get_parameter(param_prefix_ + "reach_count_max", reach_count_max_);
//             node_->get_parameter(param_prefix_ + "repeat", repeat_);
    
//             if (temp_positions.empty()) {
//                 RCLCPP_WARN(logger_, "Motor 0x%02X: No target positions provided. Using HOME (0).", motor_id_);
//                 temp_positions = {0.0};
//             }
    
//             validatePositions(temp_positions);
//             target_positions_.resize(temp_positions.size());
//             std::transform(temp_positions.begin(), temp_positions.end(), target_positions_.begin(),
//                            [](double x) { return static_cast<float>(x); });
    
//             validateSpeed();
//             validateAccel();
//             validateStep();
//             validateRepeat();
//         }
    
//     public:
//         static constexpr float VELOCITY_CONVERSION_FACTOR = 2 * 180.0 / (64 * 21 * 60.0);
    
//     private:
//         static constexpr float POSITION_TOLERANCE = 0.2;
//         static constexpr float VELOCITY_TOLERANCE = 0.1;
//         static constexpr int MAX_MOTOR_TEMPERATURE = 80;
    
//         // Thành viên dữ liệu
//         uint8_t motor_id_;
//         std::shared_ptr<ICANInterface> can_;
//         rclcpp::Node* node_;
//         rclcpp::Logger logger_;
//         State state_;
//         bool is_origin_set_;
//         bool is_home_;
//         std::vector<float> target_positions_;
//         int current_target_index_;
//         int reach_counter_;
//         int reach_count_max_;
//         float speed_;
//         float accel_;
//         int step_;
//         float target_position_;
//         bool target_reached_;
//         float min_angle_;
//         float max_angle_;
//         SafetyChecker safety_checker_;
//         StatePublisher state_publisher_;
//         std::string param_prefix_;
//         std::string last_command_;
//         MotorData last_data_;
//         rclcpp::Time last_command_time_;
//         int repeat_;
        
//         // Command Map - Thêm member variable
//         std::unordered_map<std::string, std::function<void()>> command_map_;
//     };

// // Lớp chính điều phối
// class MotorControlNode : public rclcpp::Node {
// public:
//     MotorControlNode() : Node("motor_control_node") {
//         can_ = std::make_shared<CANInterfaceWrapper>("can0");
//         timer_ = create_wall_timer(std::chrono::milliseconds(2), 
//                                   std::bind(&MotorControlNode::controlLoop, this));

//         // Khởi tạo hai động cơ
//         controllers_.push_back(std::make_unique<MotorController>(
//             0x68, can_, this, "/motor1_control_node"));
//         controllers_.push_back(std::make_unique<MotorController>(
//             0x69, can_, this, "/motor2_control_node"));

//         // Thêm subscriber cho CommandM1 và CommandM2
//         command_m1_sub_ = create_subscription<std_msgs::msg::String>(
//             "Motor1ControlCMD", 10, 
//             std::bind(&MotorControlNode::commandM1Callback, this, std::placeholders::_1));
//         command_m2_sub_ = create_subscription<std_msgs::msg::String>(
//             "Motor2ControlCMD", 10, 
//             std::bind(&MotorControlNode::commandM2Callback, this, std::placeholders::_1));

//         // Tạo service cho mỗi động cơ
//         services_.push_back(create_service<rcl_interfaces::srv::SetParameters>(
//             "/motor1_control_node/set_parameters",
//             std::bind(&MotorControlNode::setParametersCallback, this, std::placeholders::_1,
//                       std::placeholders::_2, 0)));
//         services_.push_back(create_service<rcl_interfaces::srv::SetParameters>(
//             "/motor2_control_node/set_parameters",
//             std::bind(&MotorControlNode::setParametersCallback, this, std::placeholders::_1,
//                       std::placeholders::_2, 1)));

//         rclcpp::on_shutdown(std::bind(&MotorControlNode::safeShutdown, this));
//     }

// private:
//     void commandM1Callback(const std_msgs::msg::String::SharedPtr msg) {
//         if (!controllers_.empty()) {
//             controllers_[0]->handleCommand(msg->data);
//         }
//     }

//     void commandM2Callback(const std_msgs::msg::String::SharedPtr msg) {
//         if (controllers_.size() > 1) {
//             controllers_[1]->handleCommand(msg->data);
//         }
//     }

//     void controlLoop() {
//         uint32_t id;
//         std::vector<uint8_t> raw_data;
//         try {
//             if (can_->receive(id, raw_data)) {
//                 for (const auto& controller : controllers_) {
//                     if (id == controller->motorId()) {
//                         MotorData data;
//                         can_->decodeMotorData(raw_data, data.position, data.velocity, data.current,
//                                               data.temperature, data.error);
//                         data.velocity *= MotorController::VELOCITY_CONVERSION_FACTOR;
//                         controller->control(data);
//                     }
//                 }
//             }
//         } catch (const std::exception& e) {
//             RCLCPP_ERROR(get_logger(), "CAN error: %s", e.what());
//             for (const auto& controller : controllers_) {
//                 controller->stop();
//             }
//         }
//     }

//     void setParametersCallback(
//         const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request,
//         std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response,
//         size_t controller_index) {
//         RCLCPP_INFO(get_logger(), "Received set_parameters request for controller index: %zu", controller_index);
//         if (controller_index >= controllers_.size()) {
//             RCLCPP_ERROR(get_logger(), "Invalid controller index: %zu", controller_index);
//             response->results.resize(request->parameters.size());
//             for (size_t i = 0; i < request->parameters.size(); ++i) {
//                 response->results[i].successful = false;
//                 response->results[i].reason = "Invalid controller index";
//             }
//             return;
//         }
//         controllers_[controller_index]->setParameters(request->parameters);
//         response->results.resize(request->parameters.size());
//         for (size_t i = 0; i < request->parameters.size(); ++i) {
//             response->results[i].successful = true;
//             response->results[i].reason = "";
//         }
//     }

//     void safeShutdown() {
//         RCLCPP_WARN(get_logger(), "ROS shutdown detected! Stopping motors.");
//         for (const auto& controller : controllers_) {
//             controller->stop();
//         }
//     }

//     std::shared_ptr<ICANInterface> can_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     std::vector<std::unique_ptr<IMotorController>> controllers_;
//     std::vector<rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr> services_;
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_m1_sub_;
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_m2_sub_;
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     try {
//         rclcpp::spin(std::make_shared<MotorControlNode>());
//     } catch (const std::exception& e) {
//         RCLCPP_ERROR(rclcpp::get_logger("motor_control_node"), "Fatal error: %s", e.what());
//     }
//     rclcpp::shutdown();
//     return 0;
// }













// #include "rclcpp/rclcpp.hpp"
// #include "rcl_interfaces/srv/set_parameters.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "std_msgs/msg/float32.hpp"
// #include "std_msgs/msg/bool.hpp"
// #include "std_msgs/msg/int8.hpp"
// #include <vector>
// #include <chrono>
// #include <cmath>
// #include <memory>
// #include <functional>
// #include <stdexcept>
// #include <sstream>
// #include "can_interface/caninterface.hpp"

// // Giao diện trừu tượng cho CAN
// class ICANInterface {
// public:
//     virtual bool setOrigin(uint8_t id, uint8_t mode) = 0;
//     virtual bool setPositionSpeed(uint8_t id, float pos, int speed, int accel) = 0;
//     virtual bool setCurrent(uint8_t id, float current) = 0;
//     virtual bool setRPM(uint8_t id, float rpm) = 0;
//     virtual bool receive(uint32_t& id, std::vector<uint8_t>& data) = 0;
//     virtual void decodeMotorData(const std::vector<uint8_t>& data, float& pos, float& vel, 
//                                 float& cur, int8_t& temp, int8_t& err) = 0;
//     virtual ~ICANInterface() = default;
// };

// // Triển khai CANInterface kế thừa ICANInterface
// class CANInterfaceWrapper : public ICANInterface {
// public:
//     CANInterfaceWrapper(const std::string& interface_name) : can_(interface_name) {}

//     bool setOrigin(uint8_t id, uint8_t mode) override {
//         try {
//             can_.setOrigin(id, mode);
//             return true;
//         } catch (const std::exception& e) {
//             std::cerr << "SetOrigin failed: " << e.what() << std::endl;
//             return false;
//         }
//     }

//     bool setPositionSpeed(uint8_t id, float pos, int speed, int accel) override {
//         try {
//             can_.setPositionSpeed(id, pos, speed, accel);
//             return true;
//         } catch (const std::exception& e) {
//             std::cerr << "SetPositionSpeed failed: " << e.what() << std::endl;
//             return false;
//         }
//     }

//     bool setCurrent(uint8_t id, float current) override {
//         try {
//             can_.setCurrent(id, current);
//             return true;
//         } catch (const std::exception& e) {
//             std::cerr << "SetCurrent failed: " << e.what() << std::endl;
//             return false;
//         }
//     }

//     bool setRPM(uint8_t id, float rpm) override {
//         try {
//             can_.setRPM(id, rpm);
//             return true;
//         } catch (const std::exception& e) {
//             std::cerr << "SetRPM failed: " << e.what() << std::endl;
//             return false;
//         }
//     }

//     bool receive(uint32_t& id, std::vector<uint8_t>& data) override {
//         return can_.receive(id, data);
//     }

//     void decodeMotorData(const std::vector<uint8_t>& data, float& pos, float& vel, 
//                          float& cur, int8_t& temp, int8_t& err) override {
//         can_.decodeMotorData(data, pos, vel, cur, temp, err);
//     }

// private:
//     CANInterface can_;
// };

// // Cấu trúc dữ liệu động cơ
// struct MotorData {
//     float position = 0.0;  // degrees
//     float velocity = 0.0;  // rpm
//     float current = 0.0;   // mA
//     int8_t temperature = 0; // °C
//     int8_t error = 0;      // error code
//     std::string state;     // trạng thái động cơ
// };

// // Giao diện điều khiển động cơ
// class IMotorController {
// public:
//     virtual void control(const MotorData& data) = 0;
//     virtual void setParameters(const std::vector<rcl_interfaces::msg::Parameter>& parameters) = 0;
//     virtual bool isOriginSet() const = 0;
//     virtual bool isTargetReached() const = 0;
//     virtual void stop() = 0;
//     virtual void handleCommand(const std::string& command) = 0;
//     virtual uint8_t motorId() const = 0;
//     virtual ~IMotorController() = default;
// };

// // Lớp kiểm tra an toàn
// class SafetyChecker {
// public:
//     SafetyChecker(float min_angle, float max_angle, int max_temperature)
//         : min_angle_(min_angle), max_angle_(max_angle), max_temperature_(max_temperature) {}

//     bool check(const MotorData& data, const rclcpp::Logger& logger) const {
//         if (data.temperature > max_temperature_ || data.error != 0) {
//             RCLCPP_ERROR(logger, "Motor issue! Temp: %d°C, Error: %d", data.temperature, data.error);
//             return false;
//         }
//         if (data.position < min_angle_ || data.position > max_angle_) {
//             RCLCPP_ERROR(logger, "Position %.2f out of bounds [%.2f, %.2f]", 
//                          data.position, min_angle_, max_angle_);
//             return false;
//         }
//         return true;
//     }

// private:
//     float min_angle_;
//     float max_angle_;
//     int max_temperature_;
// };

// // Lớp xuất bản trạng thái
// class StatePublisher {
// public:
//     StatePublisher(rclcpp::Node* node, const std::string& name)
//         : position_publisher_(node->create_publisher<std_msgs::msg::Float32>(name + "_position", 10)),
//           velocity_publisher_(node->create_publisher<std_msgs::msg::Float32>(name + "_vel_actual", 10)),
//           reached_publisher_(node->create_publisher<std_msgs::msg::Bool>(name + "_target_reached", 10)),
//           response_publisher_(node->create_publisher<std_msgs::msg::String>(name + "_response", 10)),
//           current_publisher_(node->create_publisher<std_msgs::msg::Float32>(name + "_current", 10)),
//           temperature_publisher_(node->create_publisher<std_msgs::msg::Int8>(name + "_temperature", 10)),
//           state_publisher_(node->create_publisher<std_msgs::msg::String>(name + "_state", 10)) {}

//     void publish(const MotorData& data, bool target_reached) {
//         std_msgs::msg::Float32 pos_msg;
//         pos_msg.data = data.position;
//         position_publisher_->publish(pos_msg);

//         std_msgs::msg::Float32 vel_msg;
//         vel_msg.data = data.velocity;
//         velocity_publisher_->publish(vel_msg);

//         std_msgs::msg::Bool reached_msg;
//         reached_msg.data = target_reached;
//         reached_publisher_->publish(reached_msg);

//         std_msgs::msg::Float32 cur_msg;
//         cur_msg.data = data.current;
//         current_publisher_->publish(cur_msg);

//         std_msgs::msg::Int8 temp_msg;
//         temp_msg.data = data.temperature;
//         temperature_publisher_->publish(temp_msg);

//         std_msgs::msg::String state_msg;
//         state_msg.data = data.state;
//         state_publisher_->publish(state_msg);
//     }

//     void publishResponse(const std::string& message) {
//         std_msgs::msg::String msg;
//         msg.data = message;
//         response_publisher_->publish(msg);
//     }

// private:
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_publisher_;
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_publisher_;
//     rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_publisher_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_publisher_;
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_publisher_;
//     rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr temperature_publisher_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
// };

// // Lớp điều khiển động cơ
// class MotorController : public IMotorController {
// public:
//     MotorController(uint8_t motor_id, std::shared_ptr<ICANInterface> can, 
//                     rclcpp::Node* node, const std::string& name)
//         : motor_id_(motor_id), can_(can), node_(node), logger_(node->get_logger()),
//           state_(State::INITIALIZING), is_origin_set_(false), is_home_(false),
//           current_target_index_(0), reach_counter_(0), target_reached_(true),
//           speed_(10000.0), accel_(1000.0), min_angle_(0.0), max_angle_(90.0),
//           step_(5), target_position_(0.0), reach_count_max_(100),
//           safety_checker_(min_angle_, max_angle_, MAX_MOTOR_TEMPERATURE),
//           state_publisher_(node, name), param_prefix_(name + "."), 
//           last_command_(""), last_command_time_(node_->now()), repeat_(1) {
//         initializeParameters();
//         setOrigin();
//         initializeCommandMap();
//     }

//     void handleCommand(const std::string& command) override {
//         std::stringstream response;
//         response << "Motor 0x" << std::hex << static_cast<int>(motor_id_) << ": ";

//         if (!is_origin_set_) {
//             response << "Cannot handle command '" << command << "' until origin set!";
//             RCLCPP_WARN(logger_, "%s", response.str().c_str());
//             state_publisher_.publishResponse(response.str());
//             return;
//         }

//         if (!is_home_ && last_data_.position == 0.0) {
//             response << "Position data not initialized, cannot process command '" << command << "'";
//             RCLCPP_WARN(logger_, "%s", response.str().c_str());
//             state_publisher_.publishResponse(response.str());
//             return;
//         }

//         if (command == "UP" || command == "DOWN") {
//             if (last_data_.position < 0.0 || last_data_.position > 90.0) {
//                 response << "Position " << last_data_.position << " out of bounds [0.0, 90.0], stopping";
//                 RCLCPP_ERROR(logger_, "%s", response.str().c_str());
//                 state_publisher_.publishResponse(response.str());
//                 stop();
//                 transitionTo(State::STOPPED);
//                 return;
//             }
//         }

//         last_command_time_ = node_->now();

//         auto it = command_map_.find(command);
//         if (it != command_map_.end()) {
//             it->second();
//         } else {
//             response << "Unknown command: " << command;
//             RCLCPP_WARN(logger_, "%s", response.str().c_str());
//             state_publisher_.publishResponse(response.str());
//         }
//     }

//     void control(const MotorData& data) override {
//         last_data_ = data;
//         last_data_.state = stateToString(state_);

//         if (!safety_checker_.check(data, logger_)) {
//             stop();
//             return;
//         }

//         state_publisher_.publish(last_data_, target_reached_);

//         switch (state_) {
//             case State::INITIALIZING:
//                 if (std::abs(data.position) < POSITION_TOLERANCE && 
//                     std::abs(data.velocity) < VELOCITY_TOLERANCE) {
//                     is_origin_set_ = true;
//                     transitionTo(State::HOMING);
//                 }
//                 break;
//             case State::HOMING:
//                 can_->setPositionSpeed(motor_id_, 0.0, static_cast<int>(speed_), static_cast<int>(accel_));
//                 if (std::abs(data.position) < POSITION_TOLERANCE && 
//                     std::abs(data.velocity) < VELOCITY_TOLERANCE) {
//                     is_home_ = true;
//                     target_reached_ = true;
//                     RCLCPP_INFO(logger_, "Motor 0x%02X reached Home Position!", motor_id_);
//                     transitionTo(State::MOVING);
//                 }
//                 break;
//             case State::MOVING: {
//                 if (target_positions_.empty() && last_command_.empty()) {
//                     RCLCPP_ERROR(logger_, "Motor 0x%02X: No target positions or command!", motor_id_);
//                     stop();
//                     transitionTo(State::STOPPED);
//                     break;
//                 }
//                 if (!last_command_.empty()) {
//                     can_->setPositionSpeed(motor_id_, target_position_, static_cast<int>(speed_), static_cast<int>(accel_));
//                     RCLCPP_INFO(logger_, "Motor 0x%02X cmd chưa empty!", motor_id_);
//                     if (std::abs(data.position - target_position_) < POSITION_TOLERANCE) {
//                         reach_counter_++;
//                         if (reach_counter_ >= reach_count_max_) {
//                             reach_counter_ = 0;
//                             target_reached_ = true;
//                             RCLCPP_INFO(logger_, "Motor 0x%02X: Reached target position %.2f", 
//                                         motor_id_, target_position_);
//                         }
//                     } else {
//                         reach_counter_ = 0;
//                         target_reached_ = false;
//                     }
//                 } else {
//                     float target = target_positions_[current_target_index_];
//                     can_->setPositionSpeed(motor_id_, target, static_cast<int>(speed_), static_cast<int>(accel_));
//                     bool is_reached = std::abs(data.position - target) < POSITION_TOLERANCE;
//                     if (is_reached) {
//                         reach_counter_++;
//                         if (reach_counter_ >= reach_count_max_) {
//                             reach_counter_ = 0;
//                             target_reached_ = true;
//                             current_target_index_++;
//                             if (current_target_index_ >= target_positions_.size()) {
//                                 current_target_index_ = target_positions_.size() - 1;
//                                 RCLCPP_INFO(logger_, "Motor 0x%02X: All targets reached.", motor_id_);
//                             }
//                         }
//                     } else {
//                         reach_counter_ = 0;
//                         target_reached_ = false;
//                     }
//                 }
//                 break;
//             }
//             case State::PAUSED:
//                 can_->setPositionSpeed(motor_id_, target_position_, static_cast<int>(speed_), static_cast<int>(accel_));
//                 if (std::abs(data.position - target_position_) < POSITION_TOLERANCE) {
//                     target_reached_ = true;
//                 }
//                 break;
//             case State::STOPPED:
//                 stop();
//                 break;
//             case State::ERROR:
//                 break;
//         }
//     }

//     void setParameters(const std::vector<rcl_interfaces::msg::Parameter>& parameters) override {
//         RCLCPP_INFO(logger_, "Motor 0x%02X: Received set_parameters request with %zu parameters", 
//                     motor_id_, parameters.size());
//         if (!is_origin_set_) {
//             RCLCPP_WARN(logger_, "Motor 0x%02X: Cannot update parameters until origin is set!", motor_id_);
//             return;
//         }
    
//         std::vector<double> temp_positions;
//         bool update_positions = false;
//         float new_speed = speed_;
//         float new_accel = accel_;
//         int new_step = step_;
//         int new_repeat = repeat_;
    
//         // Giai đoạn 1: Lưu tham số vào biến tạm
//         for (const auto& param : parameters) {
//             RCLCPP_INFO(logger_, "Motor 0x%02X: Processing parameter: %s", motor_id_, param.name.c_str());
//             if (param.name == "target_positions") {
//                 temp_positions = param.value.double_array_value;
//                 if (temp_positions.empty()) {
//                     RCLCPP_ERROR(logger_, "Motor 0x%02X: Empty target_positions received!", motor_id_);
//                     return;
//                 }
//                 validatePositions(temp_positions);
//                 update_positions = true;
//             } else if (param.name == "speed") {
//                 new_speed = static_cast<float>(param.value.double_value);
//                 // validateSpeed(new_speed);
//                 RCLCPP_INFO(logger_, "Motor 0x%02X: Updated speed: %.2f", motor_id_, new_speed);
//             } else if (param.name == "accel") {
//                 new_accel = static_cast<float>(param.value.double_value);
//                 // validateAccel(new_accel);
//                 RCLCPP_INFO(logger_, "Motor 0x%02X: Updated accel: %.2f", motor_id_, new_accel);
//             } else if (param.name == "step") {
//                 new_step = param.value.integer_value;
//                 // validateStep(new_step);
//                 RCLCPP_INFO(logger_, "Motor 0x%02X: Updated step: %d", motor_id_, new_step);
//             } else if (param.name == "repeat") {
//                 new_repeat = param.value.integer_value;
//                 // validateRepeat(new_repeat);
//                 RCLCPP_INFO(logger_, "Motor 0x%02X: Updated repeat: %d", motor_id_, new_repeat);
//             } else {
//                 RCLCPP_WARN(logger_, "Motor 0x%02X: Unknown parameter: %s", motor_id_, param.name.c_str());
//             }
//         }
    
//         // Giai đoạn 2: Cập nhật các giá trị và xử lý lặp lại target_positions
//         speed_ = new_speed;
//         accel_ = new_accel;
//         step_ = new_step;
//         repeat_ = new_repeat;
    
//         if (update_positions) {
//             std::vector<float> repeated_positions;
//             for (int i = 0; i < repeat_; ++i) {
//                 for (double pos : temp_positions) {
//                     repeated_positions.push_back(static_cast<float>(pos));
//                 }
//             }
//             if (repeated_positions.empty()) {
//                 RCLCPP_ERROR(logger_, "Motor 0x%02X: Repeated target_positions is empty!", motor_id_);
//                 return;
//             }
//             target_positions_ = repeated_positions;
//             target_reached_ = false;
//             current_target_index_ = 0;
//             RCLCPP_INFO(logger_, "Motor 0x%02X: Updated target_positions: %ld targets (repeated %d times)", 
//                         motor_id_, target_positions_.size(), repeat_);
//             if (state_ == State::PAUSED || last_command_.empty()) {
//                 transitionTo(State::MOVING);
//             }
//         }
//     }

//     bool isOriginSet() const override { return is_origin_set_; }
//     bool isTargetReached() const override { return target_reached_; }
//     uint8_t motorId() const override { return motor_id_; }

//     void stop() override {
//         try {
//             can_->setCurrent(motor_id_, 0.0);
//             can_->setPositionSpeed(motor_id_, last_data_.position, static_cast<int>(speed_), static_cast<int>(accel_));
//             last_command_ = "";
//             target_position_ = last_data_.position;
//             transitionTo(State::STOPPED);
//         } catch (const std::exception& e) {
//             RCLCPP_ERROR(logger_, "Motor 0x%02X: Stop failed: %s", motor_id_, e.what());
//             transitionTo(State::ERROR);
//         }
//     }

// private:
//     enum class State { INITIALIZING, HOMING, MOVING, STOPPED, PAUSED, ERROR };

//     std::string stateToString(State state) const {
//         switch (state) {
//             case State::INITIALIZING: return "INITIALIZING";
//             case State::HOMING: return "HOMING";
//             case State::MOVING: return "MOVING";
//             case State::STOPPED: return "STOPPED";
//             case State::PAUSED: return "PAUSED";
//             case State::ERROR: return "ERROR";
//             default: return "UNKNOWN";
//         }
//     }

//     void initializeCommandMap() {
//         command_map_ = {
//             {"UP", [this]() {
//                 std::stringstream response;
//                 response << "Motor 0x" << std::hex << static_cast<int>(motor_id_) << ": ";
                
//                 target_position_ = last_data_.position + step_;
//                 if (target_position_ >= max_angle_) target_position_ = max_angle_;
//                 last_command_ = "UP";
                
//                 if (can_->setPositionSpeed(motor_id_, target_position_, static_cast<int>(speed_), static_cast<int>(accel_))) {
//                     response << "Moving UP to position " << target_position_;
//                     target_positions_cmd_.push_back(target_position_);
//                     current_target_index_ = 0;
//                     target_reached_ = false;
//                     RCLCPP_INFO(logger_, "%s", response.str().c_str());
//                     state_publisher_.publishResponse(response.str());
//                     transitionTo(State::MOVING);
//                 } else {
//                     response << "Failed to set position " << target_position_ << " for UP";
//                     RCLCPP_ERROR(logger_, "%s", response.str().c_str());
//                     state_publisher_.publishResponse(response.str());
//                 }
//             }},
            
//             {"DOWN", [this]() {
//                 std::stringstream response;
//                 response << "Motor 0x" << std::hex << static_cast<int>(motor_id_) << ": ";
                
//                 target_position_ = last_data_.position - step_;
//                 if (target_position_ < min_angle_) target_position_ = min_angle_;
//                 last_command_ = "DOWN";
                
//                 if (can_->setPositionSpeed(motor_id_, target_position_, static_cast<int>(speed_), static_cast<int>(accel_))) {
//                     response << "Moving DOWN to position " << target_position_;
//                     target_positions_cmd_.push_back(target_position_);
//                     current_target_index_ = 0;
//                     target_reached_ = false;
//                     RCLCPP_INFO(logger_, "%s", response.str().c_str());
//                     state_publisher_.publishResponse(response.str());
//                     transitionTo(State::MOVING);
//                 } else {
//                     response << "Failed to set position " << target_position_ << " for DOWN";
//                     RCLCPP_ERROR(logger_, "%s", response.str().c_str());
//                     state_publisher_.publishResponse(response.str());
//                 }
//             }},
            
//             {"PAUSE", [this]() {
//                 std::stringstream response;
//                 response << "Motor 0x" << std::hex << static_cast<int>(motor_id_) << ": ";                
//                 last_command_ = "";
//                 target_position_ = last_data_.position;
//                 // target_positions_.clear();
//                 target_positions_.push_back(target_position_);
//                 current_target_index_ = 0;
//                 target_reached_ = true;
//                 transitionTo(State::PAUSED);
//                 RCLCPP_INFO(logger_, "Motor 0x%02X: Paused at position %.2f", motor_id_, target_position_);
//                 state_publisher_.publishResponse(response.str() + "Paused at position " + std::to_string(target_position_));
//             }},
            
//             {"HOME", [this]() {
//                 std::stringstream response;
//                 response << "Motor 0x" << std::hex << static_cast<int>(motor_id_) << ": ";
                
//                 target_position_ = 0.0;
//                 target_positions_.clear();
//                 target_positions_.push_back(target_position_);
//                 current_target_index_ = 0;
//                 target_reached_ = false;
                
//                 response << "Initiating move to HOME position (0.0)";
//                 RCLCPP_INFO(logger_, "%s", response.str().c_str());
//                 state_publisher_.publishResponse(response.str());
//                 transitionTo(State::HOMING);
//             }}
//         };
//     }

//     void validatePositions(const std::vector<double>& positions) {
//         for (double pos : positions) {
//             if (pos < min_angle_ || pos > max_angle_) {
//                 RCLCPP_ERROR(logger_, "Motor 0x%02X: Target position %.2f out of bounds [%.2f, %.2f]", 
//                              motor_id_, pos, min_angle_, max_angle_);
//                 throw std::runtime_error("Target position out of bounds");
//             }
//         }
//     }

//     void validateSpeed() {
//         if (speed_ <= 0.0 || speed_ > 50000.0) {
//             RCLCPP_ERROR(logger_, "Motor 0x%02X: Speed %.2f out of bounds [1.0, 50000.0]", motor_id_, speed_);
//             throw std::runtime_error("Speed out of bounds");
//         }
//         speed_ = std::round(speed_ * 100.0) / 100.0;
//     }

//     void validateAccel() {
//         if (accel_ <= 0.0 || accel_ > 10000.0) {
//             RCLCPP_ERROR(logger_, "Motor 0x%02X: Accel %.2f out of bounds [1.0, 10000.0]", motor_id_, accel_);
//             throw std::runtime_error("Accel out of bounds");
//         }
//         accel_ = std::round(accel_ * 100.0) / 100.0;
//     }

//     void validateStep() {
//         if (step_ <= 0 || step_ > 90) {
//             RCLCPP_ERROR(logger_, "Motor 0x%02X: Step %d out of bounds [1, 90]", motor_id_, step_);
//             throw std::runtime_error("Step out of bounds");
//         }
//     }

//     void validateRepeat() {
//         if (repeat_ < 0) {
//             RCLCPP_ERROR(logger_, "Motor 0x%02X: Repeat %d out of bounds [0, 10]", motor_id_, repeat_);
//             throw std::runtime_error("Repeat out of bounds");
//         }
//     }

//     void transitionTo(State new_state) {
//         if (state_ != new_state) {
//             RCLCPP_INFO(logger_, "Motor 0x%02X: Transitioning from %d to %d", 
//                         motor_id_, static_cast<int>(state_), static_cast<int>(new_state));
//             state_ = new_state;
//         }
//     }

//     void setOrigin() {
//         try {
//             RCLCPP_INFO(logger_, "Motor 0x%02X: Setting motor origin...", motor_id_);
//             if (!can_->setOrigin(motor_id_, 0)) {
//                 throw std::runtime_error("SetOrigin failed");
//             }
//             RCLCPP_INFO(logger_, "Motor 0x%02X: SetOrigin completed.", motor_id_);
//         } catch (const std::exception& e) {
//             RCLCPP_ERROR(logger_, "Motor 0x%02X: SetOrigin failed: %s", motor_id_, e.what());
//             stop();
//             throw std::runtime_error("Failed to set motor origin");
//         }
//     }

//     void initializeParameters() {
//         node_->declare_parameter(param_prefix_ + "target_positions", std::vector<double>{0.0});
//         node_->declare_parameter(param_prefix_ + "target_positions_cmd", std::vector<double>{0.0});
//         node_->declare_parameter(param_prefix_ + "speed", 10000.0);
//         node_->declare_parameter(param_prefix_ + "accel", 1000.0);
//         node_->declare_parameter(param_prefix_ + "min_angle", 0.0);
//         node_->declare_parameter(param_prefix_ + "max_angle", 90.0);
//         node_->declare_parameter(param_prefix_ + "step", 2);
//         node_->declare_parameter(param_prefix_ + "reach_count_max", 100);
//         node_->declare_parameter(param_prefix_ + "repeat", 1);

//         std::vector<double> temp_positions;
//         node_->get_parameter(param_prefix_ + "target_positions", temp_positions);
//         node_->get_parameter(param_prefix_ + "speed", speed_);
//         node_->get_parameter(param_prefix_ + "accel", accel_);
//         node_->get_parameter(param_prefix_ + "min_angle", min_angle_);
//         node_->get_parameter(param_prefix_ + "max_angle", max_angle_);
//         node_->get_parameter(param_prefix_ + "step", step_);
//         node_->get_parameter(param_prefix_ + "reach_count_max", reach_count_max_);
//         node_->get_parameter(param_prefix_ + "repeat", repeat_);

//         if (temp_positions.empty()) {
//             RCLCPP_WARN(logger_, "Motor 0x%02X: No target positions provided. Using HOME (0).", motor_id_);
//             temp_positions = {0.0};
//         }

//         validatePositions(temp_positions);
//         target_positions_.resize(temp_positions.size() * repeat_);
//         for (int i = 0; i < repeat_; ++i) {
//             std::transform(temp_positions.begin(), temp_positions.end(), 
//                            target_positions_.begin() + i * temp_positions.size(),
//                            [](double x) { return static_cast<float>(x); });
//         }


//         validateSpeed();
//         validateAccel();
//         validateStep();
//         validateRepeat();
//     }

// public:
//     static constexpr float VELOCITY_CONVERSION_FACTOR = 2 * 180.0 / (64 * 21 * 60.0);

// private:
//     static constexpr float POSITION_TOLERANCE = 0.2;
//     static constexpr float VELOCITY_TOLERANCE = 0.1;
//     static constexpr int MAX_MOTOR_TEMPERATURE = 80;

//     uint8_t motor_id_;
//     std::shared_ptr<ICANInterface> can_;
//     rclcpp::Node* node_;
//     rclcpp::Logger logger_;
//     State state_;
//     bool is_origin_set_;
//     bool is_home_;
//     std::vector<float> target_positions_;
//     std::vector<float> target_positions_cmd_;
//     int current_target_index_;
//     int reach_counter_;
//     int reach_count_max_;
//     float speed_;
//     float accel_;
//     int step_;
//     float target_position_;
//     bool target_reached_;
//     float min_angle_;
//     float max_angle_;
//     SafetyChecker safety_checker_;
//     StatePublisher state_publisher_;
//     std::string param_prefix_;
//     std::string last_command_;
//     MotorData last_data_;
//     rclcpp::Time last_command_time_;
//     int repeat_;
//     std::unordered_map<std::string, std::function<void()>> command_map_;
// };

// // Lớp chính điều phối
// class MotorControlNode : public rclcpp::Node {
// public:
//     MotorControlNode() : Node("motor_control_node") {
//         can_ = std::make_shared<CANInterfaceWrapper>("can0");
//         timer_ = create_wall_timer(std::chrono::milliseconds(1), 
//                                   std::bind(&MotorControlNode::controlLoop, this));

//         controllers_.push_back(std::make_unique<MotorController>(
//             0x68, can_, this, "/motor1_control_node"));
//         controllers_.push_back(std::make_unique<MotorController>(
//             0x69, can_, this, "/motor2_control_node"));

//         command_m1_sub_ = create_subscription<std_msgs::msg::String>(
//             "Motor1ControlCMD", 10, 
//             std::bind(&MotorControlNode::commandM1Callback, this, std::placeholders::_1));
//         command_m2_sub_ = create_subscription<std_msgs::msg::String>(
//             "Motor2ControlCMD", 10, 
//             std::bind(&MotorControlNode::commandM2Callback, this, std::placeholders::_1));

//         services_.push_back(create_service<rcl_interfaces::srv::SetParameters>(
//             "/motor1_control_node/set_parameters",
//             std::bind(&MotorControlNode::setParametersCallback, this, std::placeholders::_1,
//                       std::placeholders::_2, 0)));
//         services_.push_back(create_service<rcl_interfaces::srv::SetParameters>(
//             "/motor2_control_node/set_parameters",
//             std::bind(&MotorControlNode::setParametersCallback, this, std::placeholders::_1,
//                       std::placeholders::_2, 1)));

//         rclcpp::on_shutdown(std::bind(&MotorControlNode::safeShutdown, this));
//     }

// private:
//     void commandM1Callback(const std_msgs::msg::String::SharedPtr msg) {
//         if (!controllers_.empty()) {
//             controllers_[0]->handleCommand(msg->data);
//         }
//     }

//     void commandM2Callback(const std_msgs::msg::String::SharedPtr msg) {
//         if (controllers_.size() > 1) {
//             controllers_[1]->handleCommand(msg->data);
//         }
//     }

//     void controlLoop() {
//         uint32_t id;
//         std::vector<uint8_t> raw_data;
//         try {
//             if (can_->receive(id, raw_data)) {
//                 for (const auto& controller : controllers_) {
//                     if (id == controller->motorId()) {
//                         MotorData data;
//                         can_->decodeMotorData(raw_data, data.position, data.velocity, data.current,
//                                               data.temperature, data.error);
//                         data.velocity *= MotorController::VELOCITY_CONVERSION_FACTOR;
//                         controller->control(data);
//                     }
//                 }
//             }
//         } catch (const std::exception& e) {
//             RCLCPP_ERROR(get_logger(), "CAN error: %s", e.what());
//             for (const auto& controller : controllers_) {
//                 controller->stop();
//             }
//         }
//     }

//     void setParametersCallback(
//         const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request,
//         std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response,
//         size_t controller_index) {
//         RCLCPP_INFO(get_logger(), "Received set_parameters request for controller index: %zu", controller_index);
//         if (controller_index >= controllers_.size()) {
//             RCLCPP_ERROR(get_logger(), "Invalid controller index: %zu", controller_index);
//             response->results.resize(request->parameters.size());
//             for (size_t i = 0; i < request->parameters.size(); ++i) {
//                 response->results[i].successful = false;
//                 response->results[i].reason = "Invalid controller index";
//             }
//             return;
//         }
//         controllers_[controller_index]->setParameters(request->parameters);
//         response->results.resize(request->parameters.size());
//         for (size_t i = 0; i < request->parameters.size(); ++i) {
//             response->results[i].successful = true;
//             response->results[i].reason = "";
//         }
//     }

//     void safeShutdown() {
//         RCLCPP_WARN(get_logger(), "ROS shutdown detected! Stopping motors.");
//         for (const auto& controller : controllers_) {
//             controller->stop();
//         }
//     }

//     std::shared_ptr<ICANInterface> can_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     std::vector<std::unique_ptr<IMotorController>> controllers_;
//     std::vector<rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr> services_;
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_m1_sub_;
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_m2_sub_;
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     try {
//         rclcpp::spin(std::make_shared<MotorControlNode>());
//     } catch (const std::exception& e) {
//         RCLCPP_ERROR(rclcpp::get_logger("motor_control_node"), "Fatal error: %s", e.what());
//     }
//     rclcpp::shutdown();
//     return 0;
// }





// // CODE TEST NGÀY 29/5 WILL TEST


#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include <vector>
#include <chrono>
#include <cmath>
#include <memory>
#include <functional>
#include <stdexcept>
#include <sstream>
#include "can_interface/caninterface.hpp"

// Giao diện trừu tượng cho CAN
class ICANInterface {
public:
    virtual bool setOrigin(uint8_t id, uint8_t mode) = 0;
    virtual bool setPositionSpeed(uint8_t id, float pos, int speed, int accel) = 0;
    virtual bool setCurrent(uint8_t id, float current) = 0;
    virtual bool setRPM(uint8_t id, float rpm) = 0;
    virtual bool receive(uint32_t& id, std::vector<uint8_t>& data) = 0;
    virtual void decodeMotorData(const std::vector<uint8_t>& data, float& pos, float& vel, 
                                float& cur, int8_t& temp, int8_t& err) = 0;
    virtual ~ICANInterface() = default;
};

// Triển khai CANInterface kế thừa ICANInterface
class CANInterfaceWrapper : public ICANInterface {
public:
    CANInterfaceWrapper(const std::string& interface_name) : can_(interface_name) {}

    bool setOrigin(uint8_t id, uint8_t mode) override {
        try {
            can_.setOrigin(id, mode);
            return true;
        } catch (const std::exception& e) {
            std::cerr << "SetOrigin failed: " << e.what() << std::endl;
            return false;
        }
    }

    bool setPositionSpeed(uint8_t id, float pos, int speed, int accel) override {
        try {
            can_.setPositionSpeed(id, pos, speed, accel);
            return true;
        } catch (const std::exception& e) {
            std::cerr << "SetPositionSpeed failed: " << e.what() << std::endl;
            return false;
        }
    }

    bool setCurrent(uint8_t id, float current) override {
        try {
            can_.setCurrent(id, current);
            return true;
        } catch (const std::exception& e) {
            std::cerr << "SetCurrent failed: " << e.what() << std::endl;
            return false;
        }
    }

    bool setRPM(uint8_t id, float rpm) override {
        try {
            can_.setRPM(id, rpm);
            return true;
        } catch (const std::exception& e) {
            std::cerr << "SetRPM failed: " << e.what() << std::endl;
            return false;
        }
    }

    bool receive(uint32_t& id, std::vector<uint8_t>& data) override {
        return can_.receive(id, data);
    }

    void decodeMotorData(const std::vector<uint8_t>& data, float& pos, float& vel, 
                         float& cur, int8_t& temp, int8_t& err) override {
        can_.decodeMotorData(data, pos, vel, cur, temp, err);
    }

private:
    CANInterface can_;
};

// Cấu trúc dữ liệu động cơ
struct MotorData {
    float position = 0.0;  // degrees
    float velocity = 0.0;  // rpm
    float current = 0.0;   // mA
    int8_t temperature = 0; // °C
    int8_t error = 0;      // error code
    std::string state;     // trạng thái động cơ
};

// Giao diện điều khiển động cơ
class IMotorController {
public:
    virtual void control(const MotorData& data) = 0;
    virtual void setParameters(const std::vector<rcl_interfaces::msg::Parameter>& parameters) = 0;
    virtual bool isOriginSet() const = 0;
    virtual bool isTargetReached() const = 0;
    virtual void stop() = 0;
    virtual void handleCommand(const std::string& command) = 0;
    virtual uint8_t motorId() const = 0;
    virtual ~IMotorController() = default;
};

// Lớp kiểm tra an toàn
class SafetyChecker {
public:
    SafetyChecker(float min_angle, float max_angle, int max_temperature)
        : min_angle_(min_angle), max_angle_(max_angle), max_temperature_(max_temperature) {}

    bool check(const MotorData& data, const rclcpp::Logger& logger) const {
        if (data.temperature > max_temperature_ || data.error != 0) {
            RCLCPP_ERROR(logger, "Motor issue! Temp: %d°C, Error: %d", data.temperature, data.error);
            return false;
        }
        if (data.position < min_angle_ || data.position > max_angle_) {
            RCLCPP_ERROR(logger, "Position %.2f out of bounds [%.2f, %.2f]", 
                         data.position, min_angle_, max_angle_);
            return false;
        }
        return true;
    }

private:
    float min_angle_;
    float max_angle_;
    int max_temperature_;
};

// Lớp xuất bản trạng thái
class StatePublisher {
public:
    StatePublisher(rclcpp::Node* node, const std::string& name)
        : position_publisher_(node->create_publisher<std_msgs::msg::Float32>(name + "_position", 10)),
          velocity_publisher_(node->create_publisher<std_msgs::msg::Float32>(name + "_vel_actual", 10)),
          reached_publisher_(node->create_publisher<std_msgs::msg::Bool>(name + "_target_reached", 10)),
          response_publisher_(node->create_publisher<std_msgs::msg::String>(name + "_response", 10)),
          current_publisher_(node->create_publisher<std_msgs::msg::Float32>(name + "_current", 10)),
          temperature_publisher_(node->create_publisher<std_msgs::msg::Int8>(name + "_temperature", 10)),
          state_publisher_(node->create_publisher<std_msgs::msg::String>(name + "_state", 10)) {}

    void publish(const MotorData& data, bool target_reached) {
        std_msgs::msg::Float32 pos_msg;
        pos_msg.data = data.position;
        position_publisher_->publish(pos_msg);

        std_msgs::msg::Float32 vel_msg;
        vel_msg.data = data.velocity;
        velocity_publisher_->publish(vel_msg);

        std_msgs::msg::Bool reached_msg;
        reached_msg.data = target_reached;
        reached_publisher_->publish(reached_msg);

        std_msgs::msg::Float32 cur_msg;
        cur_msg.data = data.current;
        current_publisher_->publish(cur_msg);

        std_msgs::msg::Int8 temp_msg;
        temp_msg.data = data.temperature;
        temperature_publisher_->publish(temp_msg);

        std_msgs::msg::String state_msg;
        state_msg.data = data.state;
        state_publisher_->publish(state_msg);
    }

    void publishResponse(const std::string& message) {
        std_msgs::msg::String msg;
        msg.data = message;
        response_publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr temperature_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
};

// Lớp điều khiển động cơ
class MotorController : public IMotorController {
public:
    MotorController(uint8_t motor_id, std::shared_ptr<ICANInterface> can, 
                    rclcpp::Node* node, const std::string& name)
        : motor_id_(motor_id), can_(can), node_(node), logger_(node->get_logger()),
          state_(State::INITIALIZING), mode_(Mode::MANUAL), is_origin_set_(false), 
          is_home_(false), current_target_index_(0), reach_counter_(0), 
          target_reached_(true), speed_(10000.0), accel_(1000.0), min_angle_(0.0), 
          max_angle_(90.0), step_(5), target_position_(0.0), pause_position_(0.0),
          reach_count_max_(100), safety_checker_(min_angle_, max_angle_, MAX_MOTOR_TEMPERATURE),
          state_publisher_(node, name), param_prefix_(name + "."), 
          last_command_(""), pre_pause_command_(""), last_command_time_(node_->now()), 
          repeat_(1) {
        initializeParameters();
        setOrigin();
        initializeCommandMap();
    }

    void handleCommand(const std::string& command) override {
        std::stringstream response;
        response << "Motor 0x" << std::hex << static_cast<int>(motor_id_) << ": ";

        if (!is_origin_set_) {
            response << "Cannot handle command '" << command << "' until origin set!";
            RCLCPP_WARN(logger_, "%s", response.str().c_str());
            state_publisher_.publishResponse(response.str());
            return;
        }

        if (!is_home_ && last_data_.position == 0.0) {
            response << "Position data not initialized, cannot process command '" << command << "'";
            RCLCPP_WARN(logger_, "%s", response.str().c_str());
            state_publisher_.publishResponse(response.str());
            return;
        }

        if (command == "UP" || command == "DOWN") {
            if (last_data_.position < 0.0 || last_data_.position > 90.0) {
                response << "Position " << last_data_.position << " out of bounds [0.0, 90.0], stopping";
                RCLCPP_ERROR(logger_, "%s", response.str().c_str());
                state_publisher_.publishResponse(response.str());
                stop();
                transitionTo(State::STOPPED);
                return;
            }
            mode_ = Mode::MANUAL; // Chuyển sang chế độ MANUAL khi nhận UP/DOWN
        }

        last_command_time_ = node_->now();

        auto it = command_map_.find(command);
        if (it != command_map_.end()) {
            it->second();
        } else {
            response << "Unknown command: " << command;
            RCLCPP_WARN(logger_, "%s", response.str().c_str());
            state_publisher_.publishResponse(response.str());
        }
    }

    void control(const MotorData& data) override {
        last_data_ = data;
        last_data_.state = stateToString(state_);

        if (!safety_checker_.check(data, logger_)) {
            stop();
            transitionTo(State::ERROR);
            return;
        }

        state_publisher_.publish(last_data_, target_reached_);

        switch (state_) {
            case State::INITIALIZING:
                handleInitializing(data);
                break;
            case State::HOMING:
                handleHoming(data);
                break;
            case State::MOVING:
                handleMoving(data);
                break;
            case State::PAUSED:
                handlePaused(data);
                break;
            case State::STOPPED:
                handleStopped(data);
                break;
            case State::ERROR:
                handleError(data);
                break;
        }
    }

    void setParameters(const std::vector<rcl_interfaces::msg::Parameter>& parameters) override {
        RCLCPP_INFO(logger_, "Motor 0x%02X: Received set_parameters request with %zu parameters", 
                    motor_id_, parameters.size());
        if (!is_origin_set_) {
            RCLCPP_WARN(logger_, "Motor 0x%02X: Cannot update parameters until origin is set!", motor_id_);
            return;
        }

        std::vector<double> temp_positions;
        bool update_positions = false;
        float new_speed = speed_;
        float new_accel = accel_;
        int new_step = step_;
        int new_repeat = repeat_;

        // Giai đoạn 1: Lưu tham số vào biến tạm
        for (const auto& param : parameters) {
            RCLCPP_INFO(logger_, "Motor 0x%02X: Processing parameter: %s", motor_id_, param.name.c_str());
            if (param.name == "target_positions") {
                temp_positions = param.value.double_array_value;
                if (temp_positions.empty()) {
                    RCLCPP_ERROR(logger_, "Motor 0x%02X: Empty target_positions received!", motor_id_);
                    return;
                }
                validatePositions(temp_positions);
                update_positions = true;
                mode_ = Mode::AUTO; // Chuyển sang AUTO khi nhận target_positions
            } else if (param.name == "speed") {
                new_speed = static_cast<float>(param.value.double_value);
                RCLCPP_INFO(logger_, "Motor 0x%02X: Updated speed: %.2f", motor_id_, new_speed);
            } else if (param.name == "accel") {
                new_accel = static_cast<float>(param.value.double_value);
                RCLCPP_INFO(logger_, "Motor 0x%02X: Updated accel: %.2f", motor_id_, new_accel);
            } else if (param.name == "step") {
                new_step = param.value.integer_value;
                RCLCPP_INFO(logger_, "Motor 0x%02X: Updated step: %d", motor_id_, new_step);
            } else if (param.name == "repeat") {
                new_repeat = param.value.integer_value;
                RCLCPP_INFO(logger_, "Motor 0x%02X: Updated repeat: %d", motor_id_, new_repeat);
            } else {
                RCLCPP_WARN(logger_, "Motor 0x%02X: Unknown parameter: %s", motor_id_, param.name.c_str());
            }
        }

        // Giai đoạn 2: Cập nhật các giá trị và xử lý lặp lại target_positions
        speed_ = new_speed;
        accel_ = new_accel;
        step_ = new_step;
        repeat_ = new_repeat;

        if (update_positions) {
            std::vector<float> repeated_positions;
            for (int i = 0; i < repeat_; ++i) {
                for (double pos : temp_positions) {
                    repeated_positions.push_back(static_cast<float>(pos));
                }
            }
            if (repeated_positions.empty()) {
                RCLCPP_ERROR(logger_, "Motor 0x%02X: Repeated target_positions is empty!", motor_id_);
                return;
            }
            target_positions_ = repeated_positions;
            target_reached_ = false;
            current_target_index_ = 0;
            RCLCPP_INFO(logger_, "Motor 0x%02X: Updated target_positions: %ld targets (repeated %d times)", 
                        motor_id_, target_positions_.size(), repeat_);
            if (state_ == State::PAUSED || last_command_.empty()) {
                transitionTo(State::MOVING);
            }
        }

        validateSpeed();
        validateAccel();
        validateStep();
        validateRepeat();
    }

    bool isOriginSet() const override { return is_origin_set_; }
    bool isTargetReached() const override { return target_reached_; }
    uint8_t motorId() const override { return motor_id_; }

    void stop() override {
        try {
            can_->setPositionSpeed(motor_id_, last_data_.position, static_cast<int>(speed_), static_cast<int>(accel_));
            target_position_ = last_data_.position;
            last_command_ = "";
            pre_pause_command_ = "";
            transitionTo(State::STOPPED);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Motor 0x%02X: Stop failed: %s", motor_id_, e.what());
            transitionTo(State::ERROR);
        }
    }

private:
    enum class State { INITIALIZING, HOMING, MOVING, STOPPED, PAUSED, ERROR };
    enum class Mode { AUTO, MANUAL };

    std::string stateToString(State state) const {
        switch (state) {
            case State::INITIALIZING: return "INITIALIZING";
            case State::HOMING: return "HOMING";
            case State::MOVING: return "MOVING";
            case State::STOPPED: return "STOPPED";
            case State::PAUSED: return "PAUSED";
            case State::ERROR: return "ERROR";
            default: return "UNKNOWN";
        }
    }

    void handleInitializing(const MotorData& data) {
        if (std::abs(data.position) < POSITION_TOLERANCE && 
            std::abs(data.velocity) < VELOCITY_TOLERANCE) {
            is_origin_set_ = true;
            transitionTo(State::HOMING);
        }
    }

    void handleHoming(const MotorData& data) {
        can_->setPositionSpeed(motor_id_, 0.0, static_cast<int>(speed_), static_cast<int>(accel_));
        if (std::abs(data.position) < POSITION_TOLERANCE && 
            std::abs(data.velocity) < VELOCITY_TOLERANCE) {
            is_home_ = true;
            target_reached_ = true;
            RCLCPP_INFO(logger_, "Motor 0x%02X reached Home Position!", motor_id_);
            transitionTo(State::MOVING);
        }
    }

    void handleMoving(const MotorData& data) {
        if (mode_ == Mode::MANUAL && !last_command_.empty()) {
            can_->setPositionSpeed(motor_id_, target_position_, static_cast<int>(speed_), static_cast<int>(accel_));
            if (std::abs(data.position - target_position_) < POSITION_TOLERANCE) {
                reach_counter_++;
                if (reach_counter_ >= reach_count_max_) {
                    reach_counter_ = 0;
                    target_reached_ = true;
                    RCLCPP_INFO(logger_, "Motor 0x%02X: Reached target position %.2f", 
                                motor_id_, target_position_);
                }
            } else {
                reach_counter_ = 0;
                target_reached_ = false;
            }
        } else if (mode_ == Mode::AUTO && !target_positions_.empty()) {
            float target = target_positions_[current_target_index_];
            can_->setPositionSpeed(motor_id_, target, static_cast<int>(speed_), static_cast<int>(accel_));
            if (std::abs(data.position - target) < POSITION_TOLERANCE) {
                reach_counter_++;
                if (reach_counter_ >= reach_count_max_) {
                    reach_counter_ = 0;
                    target_reached_ = true;
                    current_target_index_++;
                    if (current_target_index_ >= target_positions_.size()) {
                        current_target_index_ = target_positions_.size() - 1;
                        RCLCPP_INFO(logger_, "Motor 0x%02X: All targets reached.", motor_id_);
                    }
                }
            } else {
                reach_counter_ = 0;
                target_reached_ = false;
            }
        } else {
            RCLCPP_ERROR(logger_, "Motor 0x%02X: No target positions or command!", motor_id_);
            stop();
            transitionTo(State::STOPPED);
        }
    }

    void handlePaused(const MotorData& data) {
        can_->setPositionSpeed(motor_id_, pause_position_, static_cast<int>(speed_), static_cast<int>(accel_));
        if (std::abs(data.position - pause_position_) < POSITION_TOLERANCE) {
            target_reached_ = true;
        }
    }

    void handleStopped(const MotorData& data) {
        can_->setPositionSpeed(motor_id_, data.position, static_cast<int>(speed_), static_cast<int>(accel_));
    }

    void handleError(const MotorData& data) {
        if (safety_checker_.check(data, logger_)) {
            setOrigin();
            transitionTo(State::INITIALIZING);
        }
    }

    void initializeCommandMap() {
        command_map_ = {
            {"UP", [this]() {
                std::stringstream response;
                response << "Motor 0x" << std::hex << static_cast<int>(motor_id_) << ": ";
                target_position_ = last_data_.position + step_;
                if (target_position_ >= max_angle_) target_position_ = max_angle_;
                last_command_ = "UP";
                mode_ = Mode::MANUAL;
                if (can_->setPositionSpeed(motor_id_, target_position_, static_cast<int>(speed_), static_cast<int>(accel_))) {
                    response << "Moving UP to position " << target_position_;
                    target_positions_cmd_.clear();
                    target_positions_cmd_.push_back(target_position_);
                    current_target_index_ = 0;
                    target_reached_ = false;
                    RCLCPP_INFO(logger_, "%s", response.str().c_str());
                    state_publisher_.publishResponse(response.str());
                    transitionTo(State::MOVING);
                } else {
                    response << "Failed to set position " << target_position_ << " for UP";
                    RCLCPP_ERROR(logger_, "%s", response.str().c_str());
                    state_publisher_.publishResponse(response.str());
                }
            }},
            {"DOWN", [this]() {
                std::stringstream response;
                response << "Motor 0x" << std::hex << static_cast<int>(motor_id_) << ": ";
                target_position_ = last_data_.position - step_;
                if (target_position_ < min_angle_) target_position_ = min_angle_;
                last_command_ = "DOWN";
                mode_ = Mode::MANUAL;
                if (can_->setPositionSpeed(motor_id_, target_position_, static_cast<int>(speed_), static_cast<int>(accel_))) {
                    response << "Moving DOWN to position " << target_position_;
                    target_positions_cmd_.clear();
                    target_positions_cmd_.push_back(target_position_);
                    current_target_index_ = 0;
                    target_reached_ = false;
                    RCLCPP_INFO(logger_, "%s", response.str().c_str());
                    state_publisher_.publishResponse(response.str());
                    transitionTo(State::MOVING);
                } else {
                    response << "Failed to set position " << target_position_ << " for DOWN";
                    RCLCPP_ERROR(logger_, "%s", response.str().c_str());
                    state_publisher_.publishResponse(response.str());
                }
            }},
            {"PAUSE", [this]() {
                std::stringstream response;
                response << "Motor 0x" << std::hex << static_cast<int>(motor_id_) << ": ";
                pre_pause_command_ = last_command_;
                last_command_ = "";
                pause_position_ = last_data_.position;
                target_reached_ = true;
                can_->setPositionSpeed(motor_id_, pause_position_, static_cast<int>(speed_), static_cast<int>(accel_));
                response << "Paused at position " << pause_position_;
                RCLCPP_INFO(logger_, "%s", response.str().c_str());
                state_publisher_.publishResponse(response.str());
                transitionTo(State::PAUSED);
            }},
            {"RUN", [this]() {
                std::stringstream response;
                response << "Motor 0x" << std::hex << static_cast<int>(motor_id_) << ": ";
                if (mode_ == Mode::AUTO && !target_positions_.empty()) {
                    current_target_index_ = std::min(current_target_index_, static_cast<int>(target_positions_.size() - 1));
                    target_position_ = target_positions_[current_target_index_];
                    response << "Running AUTO mode at position " << target_position_;
                    target_reached_ = false;
                    transitionTo(State::MOVING);
                } else if (mode_ == Mode::MANUAL && !pre_pause_command_.empty()) {
                    handleCommand(pre_pause_command_);
                    response << "Running MANUAL mode with command " << pre_pause_command_;
                } else {
                    response << "No valid state to run";
                    RCLCPP_WARN(logger_, "%s", response.str().c_str());
                    state_publisher_.publishResponse(response.str());
                    return;
                }
                RCLCPP_INFO(logger_, "%s", response.str().c_str());
                state_publisher_.publishResponse(response.str());
            }},
            {"HOME", [this]() {
                std::stringstream response;
                response << "Motor 0x" << std::hex << static_cast<int>(motor_id_) << ": ";
                target_position_ = 0.0;
                target_positions_.clear();
                target_positions_.push_back(target_position_);
                current_target_index_ = 0;
                target_reached_ = false;
                response << "Initiating move to HOME position (0.0)";
                RCLCPP_INFO(logger_, "%s", response.str().c_str());
                state_publisher_.publishResponse(response.str());
                transitionTo(State::HOMING);
            }}
        };
    }

    void validatePositions(const std::vector<double>& positions) {
        for (double pos : positions) {
            if (pos < min_angle_ || pos > max_angle_) {
                RCLCPP_ERROR(logger_, "Motor 0x%02X: Target position %.2f out of bounds [%.2f, %.2f]", 
                             motor_id_, pos, min_angle_, max_angle_);
                throw std::runtime_error("Target position out of bounds");
            }
        }
    }

    void validateSpeed() {
        if (speed_ <= 0.0 || speed_ > 50000.0) {
            RCLCPP_ERROR(logger_, "Motor 0x%02X: Speed %.2f out of bounds [1.0, 50000.0]", motor_id_, speed_);
            throw std::runtime_error("Speed out of bounds");
        }
        speed_ = std::round(speed_ * 100.0) / 100.0;
    }

    void validateAccel() {
        if (accel_ <= 0.0 || accel_ > 10000.0) {
            RCLCPP_ERROR(logger_, "Motor 0x%02X: Accel %.2f out of bounds [1.0, 10000.0]", motor_id_, accel_);
            throw std::runtime_error("Accel out of bounds");
        }
        accel_ = std::round(accel_ * 100.0) / 100.0;
    }

    void validateStep() {
        if (step_ <= 0 || step_ > 90) {
            RCLCPP_ERROR(logger_, "Motor 0x%02X: Step %d out of bounds [1, 90]", motor_id_, step_);
            throw std::runtime_error("Step out of bounds");
        }
    }

    void validateRepeat() {
        if (repeat_ < 0) {
            RCLCPP_ERROR(logger_, "Motor 0x%02X: Repeat %d out of bounds [0, 10]", motor_id_, repeat_);
            throw std::runtime_error("Repeat out of bounds");
        }
    }

    void transitionTo(State new_state) {
        if (state_ != new_state) {
            RCLCPP_INFO(logger_, "Motor 0x%02X: Transitioning from %s to %s", 
                        motor_id_, stateToString(state_).c_str(), stateToString(new_state).c_str());
            state_ = new_state;
        }
    }

    void setOrigin() {
        try {
            RCLCPP_INFO(logger_, "Motor 0x%02X: Setting motor origin...", motor_id_);
            if (!can_->setOrigin(motor_id_, 0)) {
                throw std::runtime_error("SetOrigin failed");
            }
            RCLCPP_INFO(logger_, "Motor 0x%02X: SetOrigin completed.", motor_id_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Motor 0x%02X: SetOrigin failed: %s", motor_id_, e.what());
            stop();
            throw std::runtime_error("Failed to set motor origin");
        }
    }

    void initializeParameters() {
        node_->declare_parameter(param_prefix_ + "target_positions", std::vector<double>{0.0});
        node_->declare_parameter(param_prefix_ + "target_positions_cmd", std::vector<double>{0.0});
        node_->declare_parameter(param_prefix_ + "speed", 10000.0);
        node_->declare_parameter(param_prefix_ + "accel", 1000.0);
        node_->declare_parameter(param_prefix_ + "min_angle", 0.0);
        node_->declare_parameter(param_prefix_ + "max_angle", 90.0);
        node_->declare_parameter(param_prefix_ + "step", 2);
        node_->declare_parameter(param_prefix_ + "reach_count_max", 100);
        node_->declare_parameter(param_prefix_ + "repeat", 1);

        std::vector<double> temp_positions;
        node_->get_parameter(param_prefix_ + "target_positions", temp_positions);
        node_->get_parameter(param_prefix_ + "speed", speed_);
        node_->get_parameter(param_prefix_ + "accel", accel_);
        node_->get_parameter(param_prefix_ + "min_angle", min_angle_);
        node_->get_parameter(param_prefix_ + "max_angle", max_angle_);
        node_->get_parameter(param_prefix_ + "step", step_);
        node_->get_parameter(param_prefix_ + "reach_count_max", reach_count_max_);
        node_->get_parameter(param_prefix_ + "repeat", repeat_);

        if (temp_positions.empty()) {
            RCLCPP_WARN(logger_, "Motor 0x%02X: No target positions provided. Using HOME (0).", motor_id_);
            temp_positions = {0.0};
        }

        validatePositions(temp_positions);
        target_positions_.resize(temp_positions.size() * repeat_);
        for (int i = 0; i < repeat_; ++i) {
            std::transform(temp_positions.begin(), temp_positions.end(), 
                           target_positions_.begin() + i * temp_positions.size(),
                           [](double x) { return static_cast<float>(x); });
        }

        validateSpeed();
        validateAccel();
        validateStep();
        validateRepeat();
    }

public:
    static constexpr float VELOCITY_CONVERSION_FACTOR = 2 * 180.0 / (64 * 21 * 60.0);

private:
    static constexpr float POSITION_TOLERANCE = 0.2;
    static constexpr float VELOCITY_TOLERANCE = 0.1;
    static constexpr int MAX_MOTOR_TEMPERATURE = 80;

    uint8_t motor_id_;
    std::shared_ptr<ICANInterface> can_;
    rclcpp::Node* node_;
    rclcpp::Logger logger_;
    State state_;
    Mode mode_;
    bool is_origin_set_;
    bool is_home_;
    std::vector<float> target_positions_;
    std::vector<float> target_positions_cmd_;
    int current_target_index_;
    int reach_counter_;
    int reach_count_max_;
    float speed_;
    float accel_;
    int step_;
    float target_position_;
    float pause_position_;
    bool target_reached_;
    float min_angle_;
    float max_angle_;
    SafetyChecker safety_checker_;
    StatePublisher state_publisher_;
    std::string param_prefix_;
    std::string last_command_;
    std::string pre_pause_command_;
    MotorData last_data_;
    rclcpp::Time last_command_time_;
    int repeat_;
    std::unordered_map<std::string, std::function<void()>> command_map_;
};

// Lớp chính điều phối
class MotorControlNode : public rclcpp::Node {
public:
    MotorControlNode() : Node("motor_control_node") {
        can_ = std::make_shared<CANInterfaceWrapper>("can0");
        timer_ = create_wall_timer(std::chrono::milliseconds(1), 
                                  std::bind(&MotorControlNode::controlLoop, this));

        controllers_.push_back(std::make_unique<MotorController>(
            0x68, can_, this, "/motor1_control_node"));
        controllers_.push_back(std::make_unique<MotorController>(
            0x69, can_, this, "/motor2_control_node"));

        command_m1_sub_ = create_subscription<std_msgs::msg::String>(
            "Motor1ControlCMD", 10, 
            std::bind(&MotorControlNode::commandM1Callback, this, std::placeholders::_1));
        command_m2_sub_ = create_subscription<std_msgs::msg::String>(
            "Motor2ControlCMD", 10, 
            std::bind(&MotorControlNode::commandM2Callback, this, std::placeholders::_1));

        services_.push_back(create_service<rcl_interfaces::srv::SetParameters>(
            "/motor1_control_node/set_parameters",
            std::bind(&MotorControlNode::setParametersCallback, this, std::placeholders::_1,
                      std::placeholders::_2, 0)));
        services_.push_back(create_service<rcl_interfaces::srv::SetParameters>(
            "/motor2_control_node/set_parameters",
            std::bind(&MotorControlNode::setParametersCallback, this, std::placeholders::_1,
                      std::placeholders::_2, 1)));

        rclcpp::on_shutdown(std::bind(&MotorControlNode::safeShutdown, this));
    }

private:
    void commandM1Callback(const std_msgs::msg::String::SharedPtr msg) {
        if (!controllers_.empty()) {
            controllers_[0]->handleCommand(msg->data);
        }
    }

    void commandM2Callback(const std_msgs::msg::String::SharedPtr msg) {
        if (controllers_.size() > 1) {
            controllers_[1]->handleCommand(msg->data);
        }
    }

    void controlLoop() {
        uint32_t id;
        std::vector<uint8_t> raw_data;
        try {
            if (can_->receive(id, raw_data)) {
                for (const auto& controller : controllers_) {
                    if (id == controller->motorId()) {
                        MotorData data;
                        can_->decodeMotorData(raw_data, data.position, data.velocity, data.current,
                                              data.temperature, data.error);
                        data.velocity *= MotorController::VELOCITY_CONVERSION_FACTOR;
                        controller->control(data);
                    }
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "CAN error: %s", e.what());
            for (const auto& controller : controllers_) {
                controller->stop();
            }
        }
    }

    void setParametersCallback(
        const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request,
        std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response,
        size_t controller_index) {
        RCLCPP_INFO(get_logger(), "Received set_parameters request for controller index: %zu", controller_index);
        if (controller_index >= controllers_.size()) {
            RCLCPP_ERROR(get_logger(), "Invalid controller index: %zu", controller_index);
            response->results.resize(request->parameters.size());
            for (size_t i = 0; i < request->parameters.size(); ++i) {
                response->results[i].successful = false;
                response->results[i].reason = "Invalid controller index";
            }
            return;
        }
        controllers_[controller_index]->setParameters(request->parameters);
        response->results.resize(request->parameters.size());
        for (size_t i = 0; i < request->parameters.size(); ++i) {
            response->results[i].successful = true;
            response->results[i].reason = "";
        }
    }

    void safeShutdown() {
        RCLCPP_WARN(get_logger(), "ROS shutdown detected! Stopping motors.");
        for (const auto& controller : controllers_) {
            controller->stop();
        }
    }

    std::shared_ptr<ICANInterface> can_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::unique_ptr<IMotorController>> controllers_;
    std::vector<rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr> services_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_m1_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_m2_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<MotorControlNode>());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_control_node"), "Fatal error: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}






// TESTING USING RPM REPLACE SETPOSSPEED 


// #include "rclcpp/rclcpp.hpp"
// #include "rcl_interfaces/srv/set_parameters.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "std_msgs/msg/float32.hpp"
// #include "std_msgs/msg/bool.hpp"
// #include "std_msgs/msg/int8.hpp"
// #include <vector>
// #include <chrono>
// #include <cmath>
// #include <memory>
// #include <functional>
// #include <stdexcept>
// #include <sstream>
// #include "can_interface/caninterface.hpp"

// // Giao diện trừu tượng cho CAN
// class ICANInterface {
// public:
//     virtual bool setOrigin(uint8_t id, uint8_t mode) = 0;
//     virtual bool setPositionSpeed(uint8_t id, float pos, int speed, int accel) = 0;
//     virtual bool setCurrent(uint8_t id, float current) = 0;
//     virtual bool setRPM(uint8_t id, float rpm) = 0;
//     virtual bool receive(uint32_t& id, std::vector<uint8_t>& data) = 0;
//     virtual void decodeMotorData(const std::vector<uint8_t>& data, float& pos, float& vel, 
//                                 float& cur, int8_t& temp, int8_t& err) = 0;
//     virtual ~ICANInterface() = default;
// };

// // Triển khai CANInterface kế thừa ICANInterface
// class CANInterfaceWrapper : public ICANInterface {
// public:
//     CANInterfaceWrapper(const std::string& interface_name) : can_(interface_name) {}

//     bool setOrigin(uint8_t id, uint8_t mode) override {
//         try {
//             can_.setOrigin(id, mode);
//             return true;
//         } catch (const std::exception& e) {
//             std::cerr << "SetOrigin failed: " << e.what() << std::endl;
//             return false;
//         }
//     }

//     bool setPositionSpeed(uint8_t id, float pos, int speed, int accel) override {
//         try {
//             can_.setPositionSpeed(id, pos, speed, accel);
//             return true;
//         } catch (const std::exception& e) {
//             std::cerr << "SetPositionSpeed failed: " << e.what() << std::endl;
//             return false;
//         }
//     }

//     bool setCurrent(uint8_t id, float current) override {
//         try {
//             can_.setCurrent(id, current);
//             return true;
//         } catch (const std::exception& e) {
//             std::cerr << "SetCurrent failed: " << e.what() << std::endl;
//             return false;
//         }
//     }

//     bool setRPM(uint8_t id, float rpm) override {
//         try {
//             can_.setRPM(id, rpm);
//             return true;
//         } catch (const std::exception& e) {
//             std::cerr << "SetRPM failed: " << e.what() << std::endl;
//             return false;
//         }
//     }

//     bool receive(uint32_t& id, std::vector<uint8_t>& data) override {
//         return can_.receive(id, data);
//     }

//     void decodeMotorData(const std::vector<uint8_t>& data, float& pos, float& vel, 
//                          float& cur, int8_t& temp, int8_t& err) override {
//         can_.decodeMotorData(data, pos, vel, cur, temp, err);
//     }

// private:
//     CANInterface can_;
// };

// // Cấu trúc dữ liệu động cơ
// struct MotorData {
//     float position = 0.0;  // degrees
//     float velocity = 0.0;  // rpm
//     float current = 0.0;   // mA
//     int8_t temperature = 0; // °C
//     int8_t error = 0;      // error code
//     std::string state;     // trạng thái động cơ
// };

// // Giao diện điều khiển động cơ
// class IMotorController {
// public:
//     virtual void control(const MotorData& data) = 0;
//     virtual void setParameters(const std::vector<rcl_interfaces::msg::Parameter>& parameters) = 0;
//     virtual bool isOriginSet() const = 0;
//     virtual bool isTargetReached() const = 0;
//     virtual void stop() = 0;
//     virtual void handleCommand(const std::string& command) = 0;
//     virtual uint8_t motorId() const = 0;
//     virtual ~IMotorController() = default;
// };

// // Lớp kiểm tra an toàn
// class SafetyChecker {
// public:
//     SafetyChecker(float min_angle, float max_angle, int max_temperature)
//         : min_angle_(min_angle), max_angle_(max_angle), max_temperature_(max_temperature) {}

//     bool check(const MotorData& data, const rclcpp::Logger& logger) const {
//         if (data.temperature > max_temperature_ || data.error != 0) {
//             RCLCPP_ERROR(logger, "Motor issue! Temp: %d°C, Error: %d", data.temperature, data.error);
//             return false;
//         }
//         if (data.position < min_angle_ || data.position > max_angle_) {
//             RCLCPP_ERROR(logger, "Position %.2f out of bounds [%.2f, %.2f]", 
//                          data.position, min_angle_, max_angle_);
//             return false;
//         }
//         return true;
//     }

// private:
//     float min_angle_;
//     float max_angle_;
//     int max_temperature_;
// };

// // Lớp xuất bản trạng thái
// class StatePublisher {
// public:
//     StatePublisher(rclcpp::Node* node, const std::string& name)
//         : position_publisher_(node->create_publisher<std_msgs::msg::Float32>(name + "_position", 10)),
//           velocity_publisher_(node->create_publisher<std_msgs::msg::Float32>(name + "_vel_actual", 10)),
//           reached_publisher_(node->create_publisher<std_msgs::msg::Bool>(name + "_target_reached", 10)),
//           response_publisher_(node->create_publisher<std_msgs::msg::String>(name + "_response", 10)),
//           current_publisher_(node->create_publisher<std_msgs::msg::Float32>(name + "_current", 10)),
//           temperature_publisher_(node->create_publisher<std_msgs::msg::Int8>(name + "_temperature", 10)),
//           state_publisher_(node->create_publisher<std_msgs::msg::String>(name + "_state", 10)) {}

//     void publish(const MotorData& data, bool target_reached) {
//         std_msgs::msg::Float32 pos_msg;
//         pos_msg.data = data.position;
//         position_publisher_->publish(pos_msg);

//         std_msgs::msg::Float32 vel_msg;
//         vel_msg.data = data.velocity;
//         velocity_publisher_->publish(vel_msg);

//         std_msgs::msg::Bool reached_msg;
//         reached_msg.data = target_reached;
//         reached_publisher_->publish(reached_msg);

//         std_msgs::msg::Float32 cur_msg;
//         cur_msg.data = data.current;
//         current_publisher_->publish(cur_msg);

//         std_msgs::msg::Int8 temp_msg;
//         temp_msg.data = data.temperature;
//         temperature_publisher_->publish(temp_msg);

//         std_msgs::msg::String state_msg;
//         state_msg.data = data.state;
//         state_publisher_->publish(state_msg);
//     }

//     void publishResponse(const std::string& message) {
//         std_msgs::msg::String msg;
//         msg.data = message;
//         response_publisher_->publish(msg);
//     }

// private:
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_publisher_;
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_publisher_;
//     rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_publisher_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_publisher_;
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_publisher_;
//     rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr temperature_publisher_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
// };

// // Lớp điều khiển động cơ
// class MotorController : public IMotorController {
// public:
//     MotorController(uint8_t motor_id, std::shared_ptr<ICANInterface> can, 
//                     rclcpp::Node* node, const std::string& name)
//         : motor_id_(motor_id), can_(can), node_(node), logger_(node->get_logger()),
//           state_(State::INITIALIZING), mode_(Mode::MANUAL), is_origin_set_(false), 
//           is_home_(false), current_target_index_(0), reach_counter_(0), 
//           target_reached_(true), speed_(10000.0), accel_(1000.0), min_angle_(0.0), 
//           max_angle_(90.0), step_(5), target_position_(0.0), pause_position_(0.0),
//           reach_count_max_(100), safety_checker_(min_angle_, max_angle_, MAX_MOTOR_TEMPERATURE),
//           state_publisher_(node, name), param_prefix_(name + "."), 
//           last_command_(""), pre_pause_command_(""), last_command_time_(node_->now()), 
//           repeat_(1) {
//         initializeParameters();
//         setOrigin();
//         initializeCommandMap();
//     }

//     void handleCommand(const std::string& command) override {
//         std::stringstream response;
//         response << "Motor 0x" << std::hex << static_cast<int>(motor_id_) << ": ";

//         if (!is_origin_set_) {
//             response << "Cannot handle command '" << command << "' until origin set!";
//             RCLCPP_WARN(logger_, "%s", response.str().c_str());
//             state_publisher_.publishResponse(response.str());
//             return;
//         }

//         if (!is_home_ && last_data_.position == 0.0) {
//             response << "Position data not initialized, cannot process command '" << command << "'";
//             RCLCPP_WARN(logger_, "%s", response.str().c_str());
//             state_publisher_.publishResponse(response.str());
//             return;
//         }

//         if (command == "UP" || command == "DOWN") {
//             if (last_data_.position < 0.0 || last_data_.position > 90.0) {
//                 response << "Position " << last_data_.position << " out of bounds [0.0, 90.0], stopping";
//                 RCLCPP_ERROR(logger_, "%s", response.str().c_str());
//                 state_publisher_.publishResponse(response.str());
//                 stop();
//                 transitionTo(State::STOPPED);
//                 return;
//             }
//             mode_ = Mode::MANUAL; // Chuyển sang chế độ MANUAL khi nhận UP/DOWN
//         }

//         last_command_time_ = node_->now();

//         auto it = command_map_.find(command);
//         if (it != command_map_.end()) {
//             it->second();
//         } else {
//             response << "Unknown command: " << command;
//             RCLCPP_WARN(logger_, "%s", response.str().c_str());
//             state_publisher_.publishResponse(response.str());
//         }
//     }

//     void control(const MotorData& data) override {
//         last_data_ = data;
//         last_data_.state = stateToString(state_);

//         if (!safety_checker_.check(data, logger_)) {
//             stop();
//             transitionTo(State::ERROR);
//             return;
//         }

//         state_publisher_.publish(last_data_, target_reached_);

//         switch (state_) {
//             case State::INITIALIZING:
//                 handleInitializing(data);
//                 break;
//             case State::HOMING:
//                 handleHoming(data);
//                 break;
//             case State::MOVING:
//                 handleMoving(data);
//                 break;
//             case State::PAUSED:
//                 handlePaused(data);
//                 break;
//             case State::STOPPED:
//                 handleStopped(data);
//                 break;
//             case State::ERROR:
//                 handleError(data);
//                 break;
//         }
//     }

//     void setParameters(const std::vector<rcl_interfaces::msg::Parameter>& parameters) override {
//         RCLCPP_INFO(logger_, "Motor 0x%02X: Received set_parameters request with %zu parameters", 
//                     motor_id_, parameters.size());
//         if (!is_origin_set_) {
//             RCLCPP_WARN(logger_, "Motor 0x%02X: Cannot update parameters until origin is set!", motor_id_);
//             return;
//         }

//         std::vector<double> temp_positions;
//         bool update_positions = false;
//         float new_speed = speed_;
//         float new_accel = accel_;
//         int new_step = step_;
//         int new_repeat = repeat_;

//         // Giai đoạn 1: Lưu tham số vào biến tạm
//         for (const auto& param : parameters) {
//             RCLCPP_INFO(logger_, "Motor 0x%02X: Processing parameter: %s", motor_id_, param.name.c_str());
//             if (param.name == "target_positions") {
//                 temp_positions = param.value.double_array_value;
//                 if (temp_positions.empty()) {
//                     RCLCPP_ERROR(logger_, "Motor 0x%02X: Empty target_positions received!", motor_id_);
//                     return;
//                 }
//                 validatePositions(temp_positions);
//                 update_positions = true;
//                 mode_ = Mode::AUTO; // Chuyển sang AUTO khi nhận target_positions
//             } else if (param.name == "speed") {
//                 new_speed = static_cast<float>(param.value.double_value);
//                 RCLCPP_INFO(logger_, "Motor 0x%02X: Updated speed: %.2f", motor_id_, new_speed);
//             } else if (param.name == "accel") {
//                 new_accel = static_cast<float>(param.value.double_value);
//                 RCLCPP_INFO(logger_, "Motor 0x%02X: Updated accel: %.2f", motor_id_, new_accel);
//             } else if (param.name == "step") {
//                 new_step = param.value.integer_value;
//                 RCLCPP_INFO(logger_, "Motor 0x%02X: Updated step: %d", motor_id_, new_step);
//             } else if (param.name == "repeat") {
//                 new_repeat = param.value.integer_value;
//                 RCLCPP_INFO(logger_, "Motor 0x%02X: Updated repeat: %d", motor_id_, new_repeat);
//             } else {
//                 RCLCPP_WARN(logger_, "Motor 0x%02X: Unknown parameter: %s", motor_id_, param.name.c_str());
//             }
//         }

//         // Giai đoạn 2: Cập nhật các giá trị và xử lý lặp lại target_positions
//         speed_ = new_speed;
//         accel_ = new_accel;
//         step_ = new_step;
//         repeat_ = new_repeat;

//         if (update_positions) {
//             std::vector<float> repeated_positions;
//             for (int i = 0; i < repeat_; ++i) {
//                 for (double pos : temp_positions) {
//                     repeated_positions.push_back(static_cast<float>(pos));
//                 }
//             }
//             if (repeated_positions.empty()) {
//                 RCLCPP_ERROR(logger_, "Motor 0x%02X: Repeated target_positions is empty!", motor_id_);
//                 return;
//             }
//             target_positions_ = repeated_positions;
//             target_reached_ = false;
//             current_target_index_ = 0;
//             RCLCPP_INFO(logger_, "Motor 0x%02X: Updated target_positions: %ld targets (repeated %d times)", 
//                         motor_id_, target_positions_.size(), repeat_);
//             if (state_ == State::PAUSED || last_command_.empty()) {
//                 transitionTo(State::MOVING);
//             }
//         }

//         validateSpeed();
//         validateAccel();
//         validateStep();
//         validateRepeat();
//     }

//     bool isOriginSet() const override { return is_origin_set_; }
//     bool isTargetReached() const override { return target_reached_; }
//     uint8_t motorId() const override { return motor_id_; }

//     void stop() override {
//         try {
//             can_->setPositionSpeed(motor_id_, last_data_.position, static_cast<int>(speed_), static_cast<int>(accel_));
//             target_position_ = last_data_.position;
//             last_command_ = "";
//             pre_pause_command_ = "";
//             transitionTo(State::STOPPED);
//         } catch (const std::exception& e) {
//             RCLCPP_ERROR(logger_, "Motor 0x%02X: Stop failed: %s", motor_id_, e.what());
//             transitionTo(State::ERROR);
//         }
//     }

// private:
//     enum class State { INITIALIZING, HOMING, MOVING, STOPPED, PAUSED, ERROR };
//     enum class Mode { AUTO, MANUAL };

//     std::string stateToString(State state) const {
//         switch (state) {
//             case State::INITIALIZING: return "INITIALIZING";
//             case State::HOMING: return "HOMING";
//             case State::MOVING: return "MOVING";
//             case State::STOPPED: return "STOPPED";
//             case State::PAUSED: return "PAUSED";
//             case State::ERROR: return "ERROR";
//             default: return "UNKNOWN";
//         }
//     }

//     void handleInitializing(const MotorData& data) {
//         if (std::abs(data.position) < POSITION_TOLERANCE && 
//             std::abs(data.velocity) < VELOCITY_TOLERANCE) {
//             is_origin_set_ = true;
//             transitionTo(State::HOMING);
//         }
//     }

//     void handleHoming(const MotorData& data) {
//         can_->setPositionSpeed(motor_id_, 0.0, static_cast<int>(speed_), static_cast<int>(accel_));
//         if (std::abs(data.position) < POSITION_TOLERANCE && 
//             std::abs(data.velocity) < VELOCITY_TOLERANCE) {
//             is_home_ = true;
//             target_reached_ = true;
//             RCLCPP_INFO(logger_, "Motor 0x%02X reached Home Position!", motor_id_);
//             transitionTo(State::MOVING);
//         }
//     }

//     void handleMoving(const MotorData& data) {
//         if (mode_ == Mode::MANUAL && !last_command_.empty()) {
//             can_->setPositionSpeed(motor_id_, target_position_, static_cast<int>(speed_), static_cast<int>(accel_));
//             if (std::abs(data.position - target_position_) < POSITION_TOLERANCE) {
//                 reach_counter_++;
//                 if (reach_counter_ >= reach_count_max_) {
//                     reach_counter_ = 0;
//                     target_reached_ = true;
//                     RCLCPP_INFO(logger_, "Motor 0x%02X: Reached target position %.2f", 
//                                 motor_id_, target_position_);
//                 }
//             } else {
//                 reach_counter_ = 0;
//                 target_reached_ = false;
//             }
//         } else if (mode_ == Mode::AUTO && !target_positions_.empty()) {
//             float target = target_positions_[current_target_index_];
//             can_->setPositionSpeed(motor_id_, target, static_cast<int>(speed_), static_cast<int>(accel_));
//             if (std::abs(data.position - target) < POSITION_TOLERANCE) {
//                 reach_counter_++;
//                 if (reach_counter_ >= reach_count_max_) {
//                     reach_counter_ = 0;
//                     target_reached_ = true;
//                     current_target_index_++;
//                     if (current_target_index_ >= target_positions_.size()) {
//                         current_target_index_ = target_positions_.size() - 1;
//                         RCLCPP_INFO(logger_, "Motor 0x%02X: All targets reached.", motor_id_);
//                     }
//                 }
//             } else {
//                 reach_counter_ = 0;
//                 target_reached_ = false;
//             }
//         } else {
//             RCLCPP_ERROR(logger_, "Motor 0x%02X: No target positions or command!", motor_id_);
//             stop();
//             transitionTo(State::STOPPED);
//         }
//     }

//     void handlePaused(const MotorData& data) {
//         can_->setPositionSpeed(motor_id_, pause_position_, static_cast<int>(speed_), static_cast<int>(accel_));
//         if (std::abs(data.position - pause_position_) < POSITION_TOLERANCE) {
//             target_reached_ = true;
//         }
//     }

//     void handleStopped(const MotorData& data) {
//         can_->setPositionSpeed(motor_id_, data.position, static_cast<int>(speed_), static_cast<int>(accel_));
//     }

//     void handleError(const MotorData& data) {
//         if (safety_checker_.check(data, logger_)) {
//             setOrigin();
//             transitionTo(State::INITIALIZING);
//         }
//     }

//     void initializeCommandMap() {
//         command_map_ = {
//             {"UP", [this]() {
//                 std::stringstream response;
//                 response << "Motor 0x" << std::hex << static_cast<int>(motor_id_) << ": ";
//                 if (abs(last_data_.position - max_angle_ >= POSITION_TOLERANCE)) can_->setRPM(motor_id_, 0.0);
//                 last_command_ = "UP";
//                 mode_ = Mode::MANUAL;
//                 if (can_->setRPM(motor_id_, 1000.0)) {
//                     response << "Moving UP with 500 RPM";
//                     target_reached_ = false;
//                     RCLCPP_INFO(logger_, "%s", response.str().c_str());
//                     state_publisher_.publishResponse(response.str());
//                     transitionTo(State::MOVING);
//                 } else {
//                     response << "Failed to set 500 RPM for UP";
//                     std::cerr << "SetRPM failed" << std::endl;
//                     RCLCPP_ERROR(logger_, "%s", response.str().c_str());
//                     state_publisher_.publishResponse(response.str());
//                 }
//             }},
//             {"DOWN", [this]() {
//                 std::stringstream response;
//                 response << "Motor 0x" << std::hex << static_cast<int>(motor_id_) << ": ";
//                 if (last_data_.position - min_angle_ >= POSITION_TOLERANCE) can_->setRPM(motor_id_, 0.0);
//                 last_command_ = "DOWN";
//                 mode_ = Mode::MANUAL;
//                 if (can_->setRPM(motor_id_, -500.0)) {
//                     response << "Moving DOWN with -500 RPM";
//                     target_reached_ = false;
//                     RCLCPP_INFO(logger_, "%s", response.str().c_str());
//                     state_publisher_.publishResponse(response.str());
//                     transitionTo(State::MOVING);
//                 } else {
//                     response << "Failed to set -500 RPM for DOWN";
//                 std::cerr << "SetRPM failed" << std::endl;
//                 RCLCPP_ERROR(logger_, "%s", response.str().c_str());
//                 state_publisher_.publishResponse(response.str());
//                 }
//             }},
//             {"PAUSE", [this]() {
//                 std::stringstream response;
//                 response << "Motor 0x" << std::hex << static_cast<int>(motor_id_) << ": ";
//                 pre_pause_command_ = last_command_;
//                 last_command_ = "";
//                 pause_position_ = last_data_.position;
//                 target_reached_ = true;
//                 can_->setPositionSpeed(motor_id_, pause_position_, static_cast<int>(speed_), static_cast<int>(accel_));
//                 response << "Paused at position " << pause_position_;
//                 RCLCPP_INFO(logger_, "%s", response.str().c_str());
//                 state_publisher_.publishResponse(response.str());
//                 transitionTo(State::PAUSED);
//             }},
//             {"RUN", [this]() {
//                 std::stringstream response;
//                 response << "Motor 0x" << std::hex << static_cast<int>(motor_id_) << ": ";
//                 if (mode_ == Mode::AUTO && !target_positions_.empty()) {
//                     current_target_index_ = std::min(current_target_index_, static_cast<int>(target_positions_.size() - 1));
//                     target_position_ = target_positions_[current_target_index_];
//                     response << "Running AUTO mode at position " << target_position_;
//                     target_reached_ = false;
//                     transitionTo(State::MOVING);
//                 } else if (mode_ == Mode::MANUAL && !pre_pause_command_.empty()) {
//                     handleCommand(pre_pause_command_);
//                     response << "Running MANUAL mode with command " << pre_pause_command_;
//                 } else {
//                     response << "No valid state to run";
//                     RCLCPP_WARN(logger_, "%s", response.str().c_str());
//                     state_publisher_.publishResponse(response.str());
//                     return;
//                 }
//                 RCLCPP_INFO(logger_, "%s", response.str().c_str());
//                 state_publisher_.publishResponse(response.str());
//             }},
//             {"HOME", [this]() {
//                 std::stringstream response;
//                 response << "Motor 0x" << std::hex << static_cast<int>(motor_id_) << ": ";
//                 target_position_ = 0.0;
//                 target_positions_.clear();
//                 target_positions_.push_back(target_position_);
//                 current_target_index_ = 0;
//                 target_reached_ = false;
//                 response << "Initiating move to HOME position (0.0)";
//                 RCLCPP_INFO(logger_, "%s", response.str().c_str());
//                 state_publisher_.publishResponse(response.str());
//                 transitionTo(State::HOMING);
//             }}
//         };
//     }

//     void validatePositions(const std::vector<double>& positions) {
//         for (double pos : positions) {
//             if (pos < min_angle_ || pos > max_angle_) {
//                 RCLCPP_ERROR(logger_, "Motor 0x%02X: Target position %.2f out of bounds [%.2f, %.2f]", 
//                              motor_id_, pos, min_angle_, max_angle_);
//                 throw std::runtime_error("Target position out of bounds");
//             }
//         }
//     }

//     void validateSpeed() {
//         if (speed_ <= 0.0 || speed_ > 50000.0) {
//             RCLCPP_ERROR(logger_, "Motor 0x%02X: Speed %.2f out of bounds [1.0, 50000.0]", motor_id_, speed_);
//             throw std::runtime_error("Speed out of bounds");
//         }
//         speed_ = std::round(speed_ * 100.0) / 100.0;
//     }

//     void validateAccel() {
//         if (accel_ <= 0.0 || accel_ > 10000.0) {
//             RCLCPP_ERROR(logger_, "Motor 0x%02X: Accel %.2f out of bounds [1.0, 10000.0]", motor_id_, accel_);
//             throw std::runtime_error("Accel out of bounds");
//         }
//         accel_ = std::round(accel_ * 100.0) / 100.0;
//     }

//     void validateStep() {
//         if (step_ <= 0 || step_ > 90) {
//             RCLCPP_ERROR(logger_, "Motor 0x%02X: Step %d out of bounds [1, 90]", motor_id_, step_);
//             throw std::runtime_error("Step out of bounds");
//         }
//     }

//     void validateRepeat() {
//         if (repeat_ < 0) {
//             RCLCPP_ERROR(logger_, "Motor 0x%02X: Repeat %d out of bounds [0, 10]", motor_id_, repeat_);
//             throw std::runtime_error("Repeat out of bounds");
//         }
//     }

//     void transitionTo(State new_state) {
//         if (state_ != new_state) {
//             RCLCPP_INFO(logger_, "Motor 0x%02X: Transitioning from %s to %s", 
//                         motor_id_, stateToString(state_).c_str(), stateToString(new_state).c_str());
//             state_ = new_state;
//         }
//     }

//     void setOrigin() {
//         try {
//             RCLCPP_INFO(logger_, "Motor 0x%02X: Setting motor origin...", motor_id_);
//             if (!can_->setOrigin(motor_id_, 0)) {
//                 throw std::runtime_error("SetOrigin failed");
//             }
//             RCLCPP_INFO(logger_, "Motor 0x%02X: SetOrigin completed.", motor_id_);
//         } catch (const std::exception& e) {
//             RCLCPP_ERROR(logger_, "Motor 0x%02X: SetOrigin failed: %s", motor_id_, e.what());
//             stop();
//             throw std::runtime_error("Failed to set motor origin");
//         }
//     }

//     void initializeParameters() {
//         node_->declare_parameter(param_prefix_ + "target_positions", std::vector<double>{0.0});
//         node_->declare_parameter(param_prefix_ + "target_positions_cmd", std::vector<double>{0.0});
//         node_->declare_parameter(param_prefix_ + "speed", 10000.0);
//         node_->declare_parameter(param_prefix_ + "accel", 1000.0);
//         node_->declare_parameter(param_prefix_ + "min_angle", 0.0);
//         node_->declare_parameter(param_prefix_ + "max_angle", 90.0);
//         node_->declare_parameter(param_prefix_ + "step", 2);
//         node_->declare_parameter(param_prefix_ + "reach_count_max", 100);
//         node_->declare_parameter(param_prefix_ + "repeat", 1);

//         std::vector<double> temp_positions;
//         node_->get_parameter(param_prefix_ + "target_positions", temp_positions);
//         node_->get_parameter(param_prefix_ + "speed", speed_);
//         node_->get_parameter(param_prefix_ + "accel", accel_);
//         node_->get_parameter(param_prefix_ + "min_angle", min_angle_);
//         node_->get_parameter(param_prefix_ + "max_angle", max_angle_);
//         node_->get_parameter(param_prefix_ + "step", step_);
//         node_->get_parameter(param_prefix_ + "reach_count_max", reach_count_max_);
//         node_->get_parameter(param_prefix_ + "repeat", repeat_);

//         if (temp_positions.empty()) {
//             RCLCPP_WARN(logger_, "Motor 0x%02X: No target positions provided. Using HOME (0).", motor_id_);
//             temp_positions = {0.0};
//         }

//         validatePositions(temp_positions);
//         target_positions_.resize(temp_positions.size() * repeat_);
//         for (int i = 0; i < repeat_; ++i) {
//             std::transform(temp_positions.begin(), temp_positions.end(), 
//                            target_positions_.begin() + i * temp_positions.size(),
//                            [](double x) { return static_cast<float>(x); });
//         }

//         validateSpeed();
//         validateAccel();
//         validateStep();
//         validateRepeat();
//     }

// public:
//     static constexpr float VELOCITY_CONVERSION_FACTOR = 2 * 180.0 / (64 * 21 * 60.0);

// private:
//     static constexpr float POSITION_TOLERANCE = 0.2;
//     static constexpr float VELOCITY_TOLERANCE = 0.1;
//     static constexpr int MAX_MOTOR_TEMPERATURE = 80;

//     uint8_t motor_id_;
//     std::shared_ptr<ICANInterface> can_;
//     rclcpp::Node* node_;
//     rclcpp::Logger logger_;
//     State state_;
//     Mode mode_;
//     bool is_origin_set_;
//     bool is_home_;
//     std::vector<float> target_positions_;
//     std::vector<float> target_positions_cmd_;
//     int current_target_index_;
//     int reach_counter_;
//     int reach_count_max_;
//     float speed_;
//     float accel_;
//     int step_;
//     float target_position_;
//     float pause_position_;
//     bool target_reached_;
//     float min_angle_;
//     float max_angle_;
//     SafetyChecker safety_checker_;
//     StatePublisher state_publisher_;
//     std::string param_prefix_;
//     std::string last_command_;
//     std::string pre_pause_command_;
//     MotorData last_data_;
//     rclcpp::Time last_command_time_;
//     int repeat_;
//     std::unordered_map<std::string, std::function<void()>> command_map_;
// };

// // Lớp chính điều phối
// class MotorControlNode : public rclcpp::Node {
// public:
//     MotorControlNode() : Node("motor_control_node") {
//         can_ = std::make_shared<CANInterfaceWrapper>("can0");
//         timer_ = create_wall_timer(std::chrono::milliseconds(1), 
//                                   std::bind(&MotorControlNode::controlLoop, this));

//         controllers_.push_back(std::make_unique<MotorController>(
//             0x68, can_, this, "/motor1_control_node"));
//         controllers_.push_back(std::make_unique<MotorController>(
//             0x69, can_, this, "/motor2_control_node"));

//         command_m1_sub_ = create_subscription<std_msgs::msg::String>(
//             "Motor1ControlCMD", 10, 
//             std::bind(&MotorControlNode::commandM1Callback, this, std::placeholders::_1));
//         command_m2_sub_ = create_subscription<std_msgs::msg::String>(
//             "Motor2ControlCMD", 10, 
//             std::bind(&MotorControlNode::commandM2Callback, this, std::placeholders::_1));

//         services_.push_back(create_service<rcl_interfaces::srv::SetParameters>(
//             "/motor1_control_node/set_parameters",
//             std::bind(&MotorControlNode::setParametersCallback, this, std::placeholders::_1,
//                       std::placeholders::_2, 0)));
//         services_.push_back(create_service<rcl_interfaces::srv::SetParameters>(
//             "/motor2_control_node/set_parameters",
//             std::bind(&MotorControlNode::setParametersCallback, this, std::placeholders::_1,
//                       std::placeholders::_2, 1)));

//         rclcpp::on_shutdown(std::bind(&MotorControlNode::safeShutdown, this));
//     }

// private:
//     void commandM1Callback(const std_msgs::msg::String::SharedPtr msg) {
//         if (!controllers_.empty()) {
//             controllers_[0]->handleCommand(msg->data);
//         }
//     }

//     void commandM2Callback(const std_msgs::msg::String::SharedPtr msg) {
//         if (controllers_.size() > 1) {
//             controllers_[1]->handleCommand(msg->data);
//         }
//     }

//     void controlLoop() {
//         uint32_t id;
//         std::vector<uint8_t> raw_data;
//         try {
//             if (can_->receive(id, raw_data)) {
//                 for (const auto& controller : controllers_) {
//                     if (id == controller->motorId()) {
//                         MotorData data;
//                         can_->decodeMotorData(raw_data, data.position, data.velocity, data.current,
//                                               data.temperature, data.error);
//                         data.velocity *= MotorController::VELOCITY_CONVERSION_FACTOR;
//                         controller->control(data);
//                     }
//                 }
//             }
//         } catch (const std::exception& e) {
//             RCLCPP_ERROR(get_logger(), "CAN error: %s", e.what());
//             for (const auto& controller : controllers_) {
//                 controller->stop();
//             }
//         }
//     }

//     void setParametersCallback(
//         const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request,
//         std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response,
//         size_t controller_index) {
//         RCLCPP_INFO(get_logger(), "Received set_parameters request for controller index: %zu", controller_index);
//         if (controller_index >= controllers_.size()) {
//             RCLCPP_ERROR(get_logger(), "Invalid controller index: %zu", controller_index);
//             response->results.resize(request->parameters.size());
//             for (size_t i = 0; i < request->parameters.size(); ++i) {
//                 response->results[i].successful = false;
//                 response->results[i].reason = "Invalid controller index";
//             }
//             return;
//         }
//         controllers_[controller_index]->setParameters(request->parameters);
//         response->results.resize(request->parameters.size());
//         for (size_t i = 0; i < request->parameters.size(); ++i) {
//             response->results[i].successful = true;
//             response->results[i].reason = "";
//         }
//     }

//     void safeShutdown() {
//         RCLCPP_WARN(get_logger(), "ROS shutdown detected! Stopping motors.");
//         for (const auto& controller : controllers_) {
//             controller->stop();
//         }
//     }

//     std::shared_ptr<ICANInterface> can_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     std::vector<std::unique_ptr<IMotorController>> controllers_;
//     std::vector<rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr> services_;
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_m1_sub_;
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_m2_sub_;
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     try {
//         rclcpp::spin(std::make_shared<MotorControlNode>());
//     } catch (const std::exception& e) {
//         RCLCPP_ERROR(rclcpp::get_logger("motor_control_node"), "Fatal error: %s", e.what());
//     }
//     rclcpp::shutdown();
//     return 0;
// }