#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <opossum_msgs/msg/robot_data.hpp>
#include <opossum_msgs/msg/goal_detection.hpp>

#include <serial/serial.h> // Standard ROS serial library

#include <thread>
#include <vector>
#include <string>
#include <sstream>
#include <map>
#include <memory>
#include <atomic>

class Communication : public rclcpp::Node {
public:
    Communication() : Node("beacon_detector_node"), enable_send_(true) {
        // 1. Declare Parameters
        this->declare_parameter<bool>("simulation", false);
        this->declare_parameter<std::string>("send_comm_topic", "");
        this->declare_parameter<std::string>("rcv_comm_topic", "");
        this->declare_parameter<std::string>("comm_state_topic", "comm_state");
        this->declare_parameter<std::vector<std::string>>("cards_name", {"zynq"});
        this->declare_parameter<std::string>("command_topic", "command");
        this->declare_parameter<std::string>("feedback_command_topic", "feedback");
        this->declare_parameter<std::string>("robot_data_topic", "robot_data");
        this->declare_parameter<std::string>("goal_position_topic", "goal_position");
        this->declare_parameter<double>("frequency", 10.0);

        simulation_ = this->get_parameter("simulation").as_bool();
        
        // Note: The python code loaded YAMLs here but never used them in the provided snippet. 
        // We omit them for performance, but you can add yaml-cpp if needed.

        init_publishers();
        init_subscribers();

        if (!simulation_) {
            start_reading_threads();
        }

        RCLCPP_INFO(this->get_logger(), "Simu: %s", simulation_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Communication node initialized.");
    }

    ~Communication() {
        // Clean up threads gracefully
        for (auto& thread : serial_threads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
    }

private:
    bool simulation_;
    std::atomic<bool> enable_send_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_comm_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_feedback_command_;
    rclcpp::Publisher<opossum_msgs::msg::RobotData>::SharedPtr pub_robot_data_;
    rclcpp::Publisher<opossum_msgs::msg::GoalDetection>::SharedPtr pub_goal_position_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_comm_state_;

    // Subscribers & Callbacks
    rclcpp::CallbackGroup::SharedPtr mutex_clb_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_comm_topic_;
    std::map<std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> sub_command_map_;
    std::string buffer_simu_rcv_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_command_simu_;
    rclcpp::TimerBase::SharedPtr read_timer_;

    // Serial & Threading
    struct CardInfo {
        std::string port;
        int baudrate;
        std::shared_ptr<serial::Serial> serial_port;
    };
    std::map<std::string, CardInfo> cards_;
    std::vector<std::thread> serial_threads_;

    void init_publishers() {
        if (simulation_) {
            std::string send_topic = this->get_parameter("send_comm_topic").as_string();
            pub_comm_ = this->create_publisher<std_msgs::msg::String>(send_topic, 10);
        }

        pub_feedback_command_ = this->create_publisher<std_msgs::msg::String>(
            this->get_parameter("feedback_command_topic").as_string(), 10);
        pub_robot_data_ = this->create_publisher<opossum_msgs::msg::RobotData>(
            this->get_parameter("robot_data_topic").as_string(), 10);
        pub_goal_position_ = this->create_publisher<opossum_msgs::msg::GoalDetection>(
            this->get_parameter("goal_position_topic").as_string(), 10);
        pub_comm_state_ = this->create_publisher<std_msgs::msg::Bool>(
            this->get_parameter("comm_state_topic").as_string(), 10);
    }

    void init_subscribers() {
        mutex_clb_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        std::string cmd_topic = this->get_parameter("command_topic").as_string();

        if (simulation_) {
            std::string rcv_topic = this->get_parameter("rcv_comm_topic").as_string();
            
            // 1. Subscribe to simulated Zynq data
            sub_comm_topic_ = this->create_subscription<std_msgs::msg::String>(
                rcv_topic, 10, std::bind(&Communication::save_in_buffer, this, std::placeholders::_1));

            // 2. Subscribe to incoming commands
            sub_command_simu_ = this->create_subscription<std_msgs::msg::String>(
                cmd_topic, 10, std::bind(&Communication::send_card_simu, this, std::placeholders::_1));

            // 3. Create the reading timer based on the frequency parameter
            double freq = this->get_parameter("frequency").as_double();
            auto timer_period = std::chrono::duration<double>(1.0 / freq);
            
            read_timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
                std::bind(&Communication::read_card_simu, this),
                mutex_clb_);
        } else {
            std::vector<std::string> cards_name = this->get_parameter("cards_name").as_string_array();
            for (const auto& name : cards_name) {
                // Initialize parameters for each card
                this->declare_parameter<std::string>("cards." + name + ".port", "/dev/ttyZynq");
                this->declare_parameter<int>("cards." + name + ".baudrate", 115200);
                
                cards_[name].port = this->get_parameter("cards." + name + ".port").as_string();
                cards_[name].baudrate = this->get_parameter("cards." + name + ".baudrate").as_int();
                
                // Initialize hardware
                init_card(name);

                // Create subscriber binding the specific card name
                auto sub_cb = [this, name](const std_msgs::msg::String::SharedPtr msg) {
                    this->send_card(msg, name);
                };
                sub_command_map_[name] = this->create_subscription<std_msgs::msg::String>(cmd_topic, 10, sub_cb);
            }
        }
    }

    void init_card(const std::string& name) {
        while (rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Trying to connect to %s on %s", name.c_str(), cards_[name].port.c_str());
            try {
                auto port = std::make_shared<serial::Serial>(
                    cards_[name].port, 
                    cards_[name].baudrate, 
                    serial::Timeout::simpleTimeout(1000)
                );

                if (port->isOpen()) {
                    port->write("VERSION\n");
                    std::string response = port->readline(65536, "\n");
                    
                    RCLCPP_INFO(this->get_logger(), "Card %s connected. Version: %s", name.c_str(), response.c_str());
                    cards_[name].serial_port = port;
                    return;
                }
            } catch (serial::IOException& e) {
                RCLCPP_ERROR(this->get_logger(), "Scanned serial port is already opened nor existing! %s", e.what());
            }

            RCLCPP_WARN(this->get_logger(), "Retrying to connect the '%s' card in 1s", name.c_str());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    // --- SIMULATION METHODS ---

    void save_in_buffer(const std_msgs::msg::String::SharedPtr msg) {
        // Simulate the Zynq sending data
        buffer_simu_rcv_ += msg->data;
    }

    void read_card_simu() {
        // Look at the result of the command send in simulation
        if (buffer_simu_rcv_.empty()) return;

        std::istringstream stream(buffer_simu_rcv_);
        std::string line;
        
        // Split buffer by newline and process each line
        while (std::getline(stream, line)) {
            // Remove carriage returns if they exist
            line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
            if (!line.empty()) {
                handle_received_line(line);
            }
        }
        // Clear buffer after reading
        buffer_simu_rcv_ = ""; 
    }

    void send_card_simu(const std_msgs::msg::String::SharedPtr msg) {
        // Send to the card the command in simulation
        if (!enable_send_) return;
        
        std::string out = process_data_send(msg->data);
        if (!out.empty()) {
            auto out_msg = std_msgs::msg::String();
            out_msg.data = out;
            pub_comm_->publish(out_msg);
        }
    }

    void start_reading_threads() {
        for (const auto& pair : cards_) {
            std::string name = pair.first;
            serial_threads_.emplace_back(&Communication::serial_read_worker, this, name);
            RCLCPP_INFO(this->get_logger(), "Started reading thread for card: %s", name.c_str());
        }
    }

    void serial_read_worker(std::string name) {
        auto serial_card = cards_[name].serial_port;
        serial_card->setTimeout(serial::Timeout::max(), 1000, 0, 1000, 0); // 1 sec timeout

        RCLCPP_INFO(this->get_logger(), "Thread listening on %s started.", name.c_str());

        while (rclcpp::ok()) {
            try {
                // Read up to the newline character
                std::string line = serial_card->readline(65536, "\n");
                
                if (!line.empty()) {
                    // LOG 1: Print the absolute raw string including hidden characters
                    RCLCPP_INFO(this->get_logger(), "[RAW RX %s]: '%s'", name.c_str(), line.c_str());

                    // Remove trailing newline, carriage return, and whitespace
                    line.erase(line.find_last_not_of(" \n\r\t") + 1);
                    
                    if (!line.empty()) {
                        handle_received_line(line);
                    }
                }
            } catch (std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Serial exception on %s: %s", name.c_str(), e.what());
                break;
            }
        }
    }

    void handle_received_line(const std::string& data) {
        // LOG 2: What does the data look like after we stripped the \n and \r?
        RCLCPP_INFO(this->get_logger(), "[CLEANED RX]: '%s'", data.c_str());

        // WARNING: If the first character is not a letter (e.g., it's a space or number), it gets dropped here!
        if (data.empty() || !std::isalpha(data[0])) {
            RCLCPP_WARN(this->get_logger(), "[DROPPED]: Message did not start with a letter.");
            return;
        }

        if (data.find("GREENSWITCH") != 0 && data.find("ROBOTDATA") != 0 && data.find("ERROR") != 0) {
            auto msg = std_msgs::msg::String();
            msg.data = data;
            pub_feedback_command_->publish(msg);
        }
        process_data_rcv(data);
    }

    void send_card(const std_msgs::msg::String::SharedPtr msg, const std::string& name) {
        RCLCPP_INFO(this->get_logger(), "HW RX on %s: '%s'", name.c_str(), msg->data.c_str());

        if (!enable_send_) {
            RCLCPP_WARN(this->get_logger(), "HW IGNORED: enable_send_ is FALSE");
            return;
        }

        std::string out = process_data_send(msg->data);
        if (!out.empty()) {
            try {
                // 1. Add \r\n just in case the hardware parser requires it
                std::string hardware_cmd = out + "\r\n"; 
                
                RCLCPP_INFO(this->get_logger(), "HW SERIAL WRITE: '%s'", out.c_str());
                
                // 2. Write the data
                cards_[name].serial_port->write(hardware_cmd);
                
                // 3. FORCE the port to send it immediately
                cards_[name].serial_port->flush(); 
                
            } catch (std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to write to card %s: %s", name.c_str(), e.what());
            }
        }
    }

    // --- PROCESSING STRINGS ---

    std::vector<std::string> split(const std::string& str) {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(str);
        while (tokenStream >> token) {
            tokens.push_back(token);
        }
        return tokens;
    }

    std::string process_data_send(std::string data) {
        auto splitted_data = split(data);
        if (splitted_data.empty()) return "";

        if (splitted_data[0] == "MOVE" && splitted_data.size() == 4) {
            try {
                auto goal_pos = opossum_msgs::msg::GoalDetection();
                goal_pos.goal_position.x = std::stod(splitted_data[1]);
                goal_pos.goal_position.y = std::stod(splitted_data[2]);
                goal_pos.goal_position.z = std::stod(splitted_data[3]);
                goal_pos.detection_mode = -1;
                goal_pos.obstacle_detection_distance = 0.5;
                pub_goal_position_->publish(goal_pos);
                
                return splitted_data[0] + " " + splitted_data[1] + " " + splitted_data[2] + " " + splitted_data[3];
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "GREGOIRE SEND GOOD COMMAND: %s", e.what());
            }
        } else if (splitted_data[0] == "MOVE" && splitted_data.size() == 5) {
            return splitted_data[0] + " " + splitted_data[1] + " " + splitted_data[2] + " " + splitted_data[3];
        }
        return data;
    }

    void process_data_rcv(const std::string& data) {
        auto splitted_data = split(data);
        if (splitted_data.empty()) return;

        if (splitted_data[0] == "GREENSWITCH") {
            enable_send_ = !enable_send_;
            auto msg = std_msgs::msg::Bool();
            msg.data = enable_send_.load();
            pub_comm_state_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Enable send: %d", msg.data);
            
        } else if (splitted_data[0] == "ROBOTDATA" && splitted_data.size() == 7) {
            try {
                auto rdata = opossum_msgs::msg::RobotData();
                rdata.x = std::stod(splitted_data[1]);
                rdata.y = std::stod(splitted_data[2]);
                rdata.theta = std::stod(splitted_data[3]);
                rdata.vlin = std::stod(splitted_data[4]);
                rdata.vdir = std::stod(splitted_data[5]);
                rdata.vt = std::stod(splitted_data[6]);
                pub_robot_data_->publish(rdata);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to parse ROBOTDATA: %s", e.what());
            }
        } else if (splitted_data[0] == "ERROR") {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", data.c_str());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Communication>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}