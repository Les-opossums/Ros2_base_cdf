#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <opossum_msgs/msg/robot_data.hpp>
#include <opossum_msgs/msg/goal_detection.hpp>

#include <serial/serial.h>
#include <yaml-cpp/yaml.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <thread>
#include <vector>
#include <string>
#include <sstream>
#include <map>
#include <memory>
#include <atomic>
#include <cstring>
#include <cerrno>
#include <algorithm>
#include <cctype>
#include <clocale>
#include <stdexcept>

/* ------------------------------------------------------------------------
 * Miroir de eth_protocol.h : header de trame + CRC. Le mapping port/id est
 * lui charge depuis eth_protocol.yaml (voir EthProtocolConfig plus bas),
 * PAS code en dur ici -- c'est le but de cette version.
 * ------------------------------------------------------------------------ */
namespace eth {

constexpr uint16_t FRAME_MAGIC = 0xC0DE;
constexpr uint8_t  PROTOCOL_VERSION = 1;

#pragma pack(push, 1)
struct FrameHeader {
    uint16_t magic;
    uint8_t  version;
    uint8_t  msg_type;
    uint16_t seq;
    uint32_t timestamp_us;
    uint16_t payload_len;
    uint16_t crc16;
};

/* Miroir de robot_messages.h */
struct PayloadRobotState {
    uint32_t timestamp_ms;
    float    x, y, theta;
    float    speed_linear, speed_direction, speed_angular;
    uint8_t  motion_done;
};

/* Miroir de Asserv_type.h -- structs utilisees pour les commandes structurees */
struct Position { float x, y, t; };
struct SetLidar { float x, y, t; uint32_t delay; };
struct SetCamera { float x, y, t; uint32_t delay; float noise_x, noise_y, noise_t; };
#pragma pack(pop)

static_assert(sizeof(FrameHeader) == 14, "FrameHeader doit faire 14 octets (desync avec le firmware sinon)");

inline uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF) {
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                  : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

} // namespace eth

/* ------------------------------------------------------------------------
 * Config protocole chargee depuis eth_protocol.yaml : channels (nom -> port
 * + direction) et messages (nom -> id + canal). Source de verite unique
 * pour le node -- aucun port/ID code en dur ailleurs dans ce fichier.
 * ------------------------------------------------------------------------ */
struct ChannelDesc {
    std::string name;
    uint16_t    port;
    std::string direction; // "tx" (Zynq emet, le Pi ecoute) ou "rx" (le Pi emet)
};

struct MessageDesc {
    std::string name;
    uint8_t     id;
    std::string channel;
};

class EthProtocolConfig {
public:
    std::map<std::string, ChannelDesc> channels_by_name;
    std::map<std::string, MessageDesc> messages_by_name;
    std::map<uint8_t, MessageDesc>     messages_by_id;

    void load(const std::string &path) {
        YAML::Node root = YAML::LoadFile(path);

        for (const auto &kv : root["channels"]) {
            ChannelDesc c;
            c.name = kv.first.as<std::string>();
            c.port = static_cast<uint16_t>(kv.second["port"].as<int>());
            c.direction = kv.second["direction"].as<std::string>();
            channels_by_name[c.name] = c;
        }

        for (const auto &kv : root["messages"]) {
            MessageDesc m;
            m.name = kv.first.as<std::string>();
            m.id = static_cast<uint8_t>(kv.second["id"].as<int>());
            m.channel = kv.second["channel"].as<std::string>();
            messages_by_name[m.name] = m;
            messages_by_id[m.id] = m;
        }
    }

    uint16_t port_for_message(const std::string &msg_name) const {
        const auto &m = messages_by_name.at(msg_name); // .at() leve si inconnu -- fail-fast voulu
        return channels_by_name.at(m.channel).port;
    }

    uint8_t id_for_message(const std::string &msg_name) const {
        return messages_by_name.at(msg_name).id;
    }
};

class Communication : public rclcpp::Node {
public:
    Communication() : Node("beacon_detector_node"), enable_send_(true) {
        this->declare_parameter<bool>("simulation", false);
        this->declare_parameter<std::string>("send_comm_topic", "");
        this->declare_parameter<std::string>("rcv_comm_topic", "");
        this->declare_parameter<std::string>("comm_state_topic", "comm_state");
        this->declare_parameter<std::vector<std::string>>("cards_name", {"zynq"});
        this->declare_parameter<std::string>("command_topic", "command");
        this->declare_parameter<std::string>("feedback_command_topic", "feedback");
        this->declare_parameter<std::string>("robot_data_topic", "robot_data");
        this->declare_parameter<std::string>("motion_done_topic", "motion_done");
        this->declare_parameter<std::string>("goal_position_topic", "goal_position");
        this->declare_parameter<double>("frequency", 10.0);
        this->declare_parameter<std::string>("protocol_config_path", "");

        simulation_ = this->get_parameter("simulation").as_bool();

        std::string protocol_path = this->get_parameter("protocol_config_path").as_string();
        if (protocol_path.empty()) {
            RCLCPP_FATAL(this->get_logger(),
                "Parametre 'protocol_config_path' non renseigne -- chemin vers eth_protocol.yaml requis.");
            throw std::runtime_error("protocol_config_path manquant");
        }
        try {
            protocol_.load(protocol_path);
            RCLCPP_INFO(this->get_logger(), "Protocole charge depuis %s (%zu canaux, %zu messages)",
                        protocol_path.c_str(), protocol_.channels_by_name.size(), protocol_.messages_by_name.size());
        } catch (const std::exception &e) {
            RCLCPP_FATAL(this->get_logger(), "Echec chargement %s : %s", protocol_path.c_str(), e.what());
            throw;
        }

        init_publishers();
        init_subscribers();

        if (!simulation_) {
            start_reading_threads();
        }

        RCLCPP_INFO(this->get_logger(), "Simu: %s", simulation_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Communication node initialized.");
    }

    ~Communication() {
        for (auto& thread : serial_threads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
        for (auto& pair : cards_) {
            CardInfo &card = pair.second;
            for (auto &rf : card.recv_fds) {
                if (rf.second >= 0) close(rf.second);
            }
            if (card.udp_send_fd >= 0) close(card.udp_send_fd);
        }
    }

private:
    bool simulation_;
    std::atomic<bool> enable_send_;
    EthProtocolConfig protocol_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_comm_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_feedback_command_;
    rclcpp::Publisher<opossum_msgs::msg::RobotData>::SharedPtr pub_robot_data_;
    rclcpp::Publisher<opossum_msgs::msg::GoalDetection>::SharedPtr pub_goal_position_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_comm_state_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_motion_done_;

    rclcpp::CallbackGroup::SharedPtr mutex_clb_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_comm_topic_;
    std::map<std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> sub_command_map_;
    std::string buffer_simu_rcv_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_command_simu_;
    rclcpp::TimerBase::SharedPtr read_timer_;

    struct CardInfo {
        std::string transport = "serial"; // "serial" ou "ethernet"

        // --- serial ---
        std::string port;
        int baudrate = 115200;
        std::shared_ptr<serial::Serial> serial_port;

        // --- ethernet ---
        std::string zynq_ip;
        int udp_send_fd = -1;
        uint16_t tx_seq = 0;
        std::map<std::string, int> recv_fds; // nom du canal (direction tx) -> fd bind
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
        pub_motion_done_ = this->create_publisher<std_msgs::msg::Bool>(
            this->get_parameter("motion_done_topic").as_string(), 10);
    }

    void init_subscribers() {
        mutex_clb_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        std::string cmd_topic = this->get_parameter("command_topic").as_string();

        if (simulation_) {
            std::string rcv_topic = this->get_parameter("rcv_comm_topic").as_string();

            sub_comm_topic_ = this->create_subscription<std_msgs::msg::String>(
                rcv_topic, 10, std::bind(&Communication::save_in_buffer, this, std::placeholders::_1));

            sub_command_simu_ = this->create_subscription<std_msgs::msg::String>(
                cmd_topic, 10, std::bind(&Communication::send_card_simu, this, std::placeholders::_1));

            double freq = this->get_parameter("frequency").as_double();
            auto timer_period = std::chrono::duration<double>(1.0 / freq);

            read_timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
                std::bind(&Communication::read_card_simu, this),
                mutex_clb_);
        } else {
            std::vector<std::string> cards_name = this->get_parameter("cards_name").as_string_array();
            for (const auto& name : cards_name) {
                this->declare_parameter<std::string>("cards." + name + ".transport", "serial");
                this->declare_parameter<std::string>("cards." + name + ".port", "/dev/ttyZynq");
                this->declare_parameter<int>("cards." + name + ".baudrate", 115200);
                this->declare_parameter<std::string>("cards." + name + ".zynq_ip", "192.168.1.10");

                CardInfo &card = cards_[name];
                card.transport = this->get_parameter("cards." + name + ".transport").as_string();
                card.port      = this->get_parameter("cards." + name + ".port").as_string();
                card.baudrate  = this->get_parameter("cards." + name + ".baudrate").as_int();
                card.zynq_ip   = this->get_parameter("cards." + name + ".zynq_ip").as_string();

                init_card(name);

                auto sub_cb = [this, name](const std_msgs::msg::String::SharedPtr msg) {
                    this->send_card(msg, name);
                };
                sub_command_map_[name] = this->create_subscription<std_msgs::msg::String>(cmd_topic, 10, sub_cb);
            }
        }
    }

    void init_card(const std::string& name) {
        if (cards_[name].transport == "ethernet") {
            init_card_ethernet(name);
        } else {
            init_card_serial(name);
        }
    }

    void init_card_serial(const std::string& name) {
        while (rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Trying to connect to %s on %s", name.c_str(), cards_[name].port.c_str());
            try {
                auto port = std::make_shared<serial::Serial>(
                    cards_[name].port, cards_[name].baudrate, serial::Timeout::simpleTimeout(1000));

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

    int udp_bind_socket(uint16_t port) {
        int fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd < 0) return -1;

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 500000; // 500ms, pour re-checker rclcpp::ok() periodiquement
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        struct sockaddr_in addr {};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);

        if (bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            close(fd);
            return -1;
        }
        return fd;
    }

    void init_card_ethernet(const std::string& name) {
        CardInfo &card = cards_[name];
        card.udp_send_fd = socket(AF_INET, SOCK_DGRAM, 0);

        // Un canal "tx" (Zynq -> Pi) doit etre ecoute par le Pi. Un canal "rx"
        // (Pi -> Zynq) n'a rien a ecouter, seulement un port de destination.
        // Entierement pilote par eth_protocol.yaml -- ajouter un canal tx la-bas
        // suffit a le faire ecouter ici, sans toucher a ce code.
        for (const auto& kv : protocol_.channels_by_name) {
            const ChannelDesc &chan = kv.second;
            if (chan.direction != "tx") continue;

            int fd = udp_bind_socket(chan.port);
            if (fd < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to bind port %d (channel %s) for card %s",
                             chan.port, chan.name.c_str(), name.c_str());
                continue;
            }
            card.recv_fds[chan.name] = fd;
        }

        RCLCPP_INFO(this->get_logger(), "Card %s (ethernet) ready: %s, %zu canaux en ecoute",
                    name.c_str(), card.zynq_ip.c_str(), card.recv_fds.size());
    }

    // --- SIMULATION ---

    void save_in_buffer(const std_msgs::msg::String::SharedPtr msg) {
        buffer_simu_rcv_ += msg->data;
    }

    void read_card_simu() {
        if (buffer_simu_rcv_.empty()) return;
        std::istringstream stream(buffer_simu_rcv_);
        std::string line;
        while (std::getline(stream, line)) {
            line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
            if (!line.empty()) handle_received_line(line);
        }
        buffer_simu_rcv_ = "";
    }

    void send_card_simu(const std_msgs::msg::String::SharedPtr msg) {
        if (!enable_send_) return;
        std::string out = process_data_send(msg->data);
        if (!out.empty()) {
            auto out_msg = std_msgs::msg::String();
            out_msg.data = out;
            pub_comm_->publish(out_msg);
        }
    }

    void start_reading_threads() {
        for (auto& pair : cards_) {
            std::string name = pair.first;
            CardInfo &card = pair.second;
            if (card.transport == "ethernet") {
                for (const auto& rf : card.recv_fds) {
                    serial_threads_.emplace_back(&Communication::eth_read_worker, this, name, rf.first);
                }
                RCLCPP_INFO(this->get_logger(), "Started %zu ethernet reading threads for card: %s",
                            card.recv_fds.size(), name.c_str());
            } else {
                serial_threads_.emplace_back(&Communication::serial_read_worker, this, name);
                RCLCPP_INFO(this->get_logger(), "Started reading thread for card: %s", name.c_str());
            }
        }
    }

    void serial_read_worker(std::string name) {
        auto serial_card = cards_[name].serial_port;
        serial_card->setTimeout(serial::Timeout::max(), 1000, 0, 1000, 0);

        while (rclcpp::ok()) {
            try {
                std::string line = serial_card->readline(65536, "\n");
                if (!line.empty()) {
                    size_t start = line.find_first_not_of(" \n\r\t");
                    if (start == std::string::npos) {
                        line = "";
                    } else {
                        line = line.substr(start);
                        line.erase(line.find_last_not_of(" \n\r\t") + 1);
                    }
                    if (!line.empty()) handle_received_line(line);
                }
            } catch (std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Serial exception on %s: %s", name.c_str(), e.what());
                break;
            }
        }
    }

    void eth_read_worker(std::string name, std::string chan_name) {
        CardInfo &card = cards_[name];
        int fd = card.recv_fds.at(chan_name);
        uint8_t buf[1024];
        while (rclcpp::ok()) {
            ssize_t n = recv(fd, buf, sizeof(buf), 0);
            if (n <= 0) continue; // timeout (500ms) ou erreur transitoire
            handle_eth_frame(buf, static_cast<size_t>(n), name);
        }
    }

    void handle_eth_frame(const uint8_t* buf, size_t n, const std::string& name) {
        if (n < sizeof(eth::FrameHeader)) return;

        eth::FrameHeader hdr;
        std::memcpy(&hdr, buf, sizeof(hdr));

        if (hdr.magic != eth::FRAME_MAGIC || sizeof(hdr) + hdr.payload_len != n) {
            RCLCPP_WARN(this->get_logger(), "Malformed eth frame from card %s", name.c_str());
            return;
        }

        const uint8_t* payload = buf + sizeof(hdr);

        uint16_t rx_crc = hdr.crc16;
        uint16_t calc_crc = eth::crc16_ccitt(buf, 12, 0xFFFF); // header avant le champ crc16
        static const uint8_t zero_crc_field[2] = {0, 0};
        calc_crc = eth::crc16_ccitt(zero_crc_field, 2, calc_crc);
        calc_crc = eth::crc16_ccitt(payload, hdr.payload_len, calc_crc);

        if (calc_crc != rx_crc) {
            RCLCPP_WARN(this->get_logger(), "CRC error on eth frame from card %s", name.c_str());
            return;
        }

        auto it = protocol_.messages_by_id.find(hdr.msg_type);
        std::string msg_name = (it != protocol_.messages_by_id.end()) ? it->second.name : "UNKNOWN";

        if (msg_name == "ROBOT_STATE") {
            if (hdr.payload_len != sizeof(eth::PayloadRobotState)) return;
            eth::PayloadRobotState rs;
            std::memcpy(&rs, payload, sizeof(rs));

            auto rdata = opossum_msgs::msg::RobotData();
            rdata.name = name;
            rdata.x = rs.x;
            rdata.y = rs.y;
            rdata.theta = rs.theta;
            rdata.vlin = rs.speed_linear;
            rdata.vdir = rs.speed_direction;
            rdata.vt = rs.speed_angular;
            pub_robot_data_->publish(rdata);

            auto md = std_msgs::msg::Bool();
            md.data = (rs.motion_done != 0);
            pub_motion_done_->publish(md);

        } else if (msg_name == "DEBUG_TEXT") {
            std::string text(reinterpret_cast<const char*>(payload), hdr.payload_len);
            while (!text.empty() && (text.back() == '\n' || text.back() == '\r')) text.pop_back();
            if (!text.empty()) handle_received_line(text);

        } else {
            // HEARTBEAT, ODOM, IMU, MOTOR_STATE -- pas encore cables cote ROS2
        }
    }

    void handle_received_line(const std::string& data) {
        if (data.empty() || !std::isalpha(data[0])) return;

        if (data.find("GREENSWITCH") != 0 && data.find("ROBOTDATA") != 0 && data.find("ERROR") != 0) {
            auto msg = std_msgs::msg::String();
            msg.data = data;
            pub_feedback_command_->publish(msg);
        }
        process_data_rcv(data);
    }

    // Construit et envoie une trame framee (header + CRC) sur le canal associe
    // au nom de message donne dans eth_protocol.yaml. Retourne false si le nom
    // de message est inconnu ou l'envoi a echoue.
    bool send_framed(CardInfo &card, const std::string &msg_name,
                      const uint8_t *payload, uint16_t payload_len) {
        uint8_t id;
        uint16_t port;
        try {
            id = protocol_.id_for_message(msg_name);
            port = protocol_.port_for_message(msg_name);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Message '%s' inconnu dans eth_protocol.yaml: %s",
                         msg_name.c_str(), e.what());
            return false;
        }

        uint8_t buf[sizeof(eth::FrameHeader) + 256];
        if (payload_len > sizeof(buf) - sizeof(eth::FrameHeader)) {
            RCLCPP_ERROR(this->get_logger(), "Payload trop grand pour '%s' (%u octets)", msg_name.c_str(), payload_len);
            return false;
        }

        eth::FrameHeader hdr{};
        hdr.magic = eth::FRAME_MAGIC;
        hdr.version = eth::PROTOCOL_VERSION;
        hdr.msg_type = id;
        hdr.seq = card.tx_seq++;
        hdr.timestamp_us = 0; // pas d'horloge partagee avec le Zynq cote Pi, non utilise par le firmware
        hdr.payload_len = payload_len;
        hdr.crc16 = 0;

        std::memcpy(buf, &hdr, sizeof(hdr));
        if (payload_len) {
            std::memcpy(buf + sizeof(hdr), payload, payload_len);
        }

        uint16_t total_len = static_cast<uint16_t>(sizeof(hdr) + payload_len);
        uint16_t crc = eth::crc16_ccitt(buf, total_len);
        std::memcpy(buf + 12, &crc, sizeof(crc)); // offset du champ crc16 dans le header

        struct sockaddr_in dst {};
        dst.sin_family = AF_INET;
        dst.sin_port = htons(port);
        inet_pton(AF_INET, card.zynq_ip.c_str(), &dst.sin_addr);

        ssize_t sent = sendto(card.udp_send_fd, buf, total_len, 0,
                               reinterpret_cast<struct sockaddr*>(&dst), sizeof(dst));
        if (sent != static_cast<ssize_t>(total_len)) {
            RCLCPP_ERROR(this->get_logger(), "UDP send failed for '%s': %s", msg_name.c_str(), strerror(errno));
            return false;
        }
        return true;
    }

    void send_card(const std_msgs::msg::String::SharedPtr msg, const std::string& name) {
        if (!enable_send_) return;
        CardInfo &card = cards_[name];

        if (card.transport != "ethernet") {
            std::string out = process_data_send(msg->data);
            if (!out.empty()) {
                try {
                    card.serial_port->write(out + "\n");
                } catch (std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to write to card %s: %s", name.c_str(), e.what());
                }
            }
            return;
        }

        // Chemin ethernet : on essaie de reconnaitre une commande structuree
        // connue et de l'encoder en trame binaire ; sinon fallback texte libre
        // encapsule en CMD_GENERIC (framed + CRC).
        //
        // Grammaire assumee -- a adapter a ta convention reelle cote strategie :
        //   MOVE x y t                              -> CMD_GOAL_POSITION
        //   SETLIDAR x y t delay                    -> CMD_SET_LIDAR
        //   SETCAM1/2/3 x y t delay nx ny nt         -> CMD_SET_CAMERA_1/2/3
        //   BLOCK / FREE                             -> CMD_BLOCK / CMD_FREE
        auto tokens = split(msg->data);
        if (tokens.empty()) return;

        try {
            if (tokens[0] == "MOVE" && tokens.size() >= 4) {
                eth::Position pos{};
                pos.x = std::stof(tokens[1]);
                pos.y = std::stof(tokens[2]);
                pos.t = std::stof(tokens[3]);
                send_framed(card, "CMD_GOAL_POSITION", reinterpret_cast<uint8_t*>(&pos), sizeof(pos));

                auto goal_pos = opossum_msgs::msg::GoalDetection();
                goal_pos.goal_position.x = pos.x;
                goal_pos.goal_position.y = pos.y;
                goal_pos.goal_position.z = pos.t;
                goal_pos.detection_mode = -1;
                goal_pos.obstacle_detection_distance = 0.5;
                pub_goal_position_->publish(goal_pos);
                return;
            }

            if (tokens[0] == "SETLIDAR" && tokens.size() >= 5) {
                eth::SetLidar sl{};
                sl.x = std::stof(tokens[1]);
                sl.y = std::stof(tokens[2]);
                sl.t = std::stof(tokens[3]);
                sl.delay = static_cast<uint32_t>(std::stoul(tokens[4]));
                send_framed(card, "CMD_SET_LIDAR", reinterpret_cast<uint8_t*>(&sl), sizeof(sl));
                return;
            }

            if ((tokens[0] == "SETCAM1" || tokens[0] == "SETCAM2" || tokens[0] == "SETCAM3") && tokens.size() >= 8) {
                eth::SetCamera sc{};
                sc.x = std::stof(tokens[1]);
                sc.y = std::stof(tokens[2]);
                sc.t = std::stof(tokens[3]);
                sc.delay = static_cast<uint32_t>(std::stoul(tokens[4]));
                sc.noise_x = std::stof(tokens[5]);
                sc.noise_y = std::stof(tokens[6]);
                sc.noise_t = std::stof(tokens[7]);
                std::string msg_name = (tokens[0] == "SETCAM1") ? "CMD_SET_CAMERA_1"
                                      : (tokens[0] == "SETCAM2") ? "CMD_SET_CAMERA_2"
                                                                  : "CMD_SET_CAMERA_3";
                send_framed(card, msg_name, reinterpret_cast<uint8_t*>(&sc), sizeof(sc));
                return;
            }

            if (tokens[0] == "BLOCK") {
                uint8_t v = 1;
                send_framed(card, "CMD_BLOCK", &v, sizeof(v));
                return;
            }

            if (tokens[0] == "FREE") {
                uint8_t v = 1;
                send_framed(card, "CMD_FREE", &v, sizeof(v));
                return;
            }
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Failed to parse structured command '%s': %s", msg->data.c_str(), e.what());
            return;
        }

        // Fallback : texte libre encapsule en CMD_GENERIC
        std::string out = process_data_send(msg->data);
        if (!out.empty()) {
            send_framed(card, "CMD_GENERIC", reinterpret_cast<const uint8_t*>(out.data()),
                        static_cast<uint16_t>(out.size()));
        }
    }

    // --- PROCESSING STRINGS ---

    std::vector<std::string> split(const std::string& str) {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(str);
        while (tokenStream >> token) tokens.push_back(token);
        return tokens;
    }

    std::string limit_decimals(const std::string& token, int max_decimals = 3) {
        size_t dot_pos = token.find('.');
        if (dot_pos != std::string::npos && token.length() > dot_pos + 1 + max_decimals) {
            return token.substr(0, dot_pos + 1 + max_decimals);
        }
        return token;
    }

    // Reconstruction texte generique (utilisee pour le serial, et en fallback
    // ethernet pour tout ce qui n'est pas une commande structuree reconnue).
    std::string process_data_send(std::string data) {
        auto splitted_data = split(data);
        if (splitted_data.empty()) return "";

        std::string out_msg = splitted_data[0];
        size_t arg_count = (splitted_data[0] == "MOVE" && splitted_data.size() == 5) ? 4 : splitted_data.size();
        for (size_t i = 1; i < arg_count; ++i) {
            out_msg += " " + limit_decimals(splitted_data[i], 3);
        }
        return out_msg;
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
    std::setlocale(LC_NUMERIC, "C");

    rclcpp::init(argc, argv);
    auto node = std::make_shared<Communication>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}