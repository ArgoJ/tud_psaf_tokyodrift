#include "button_launch_node.h"


ButtonLaunchNode::ButtonLaunchNode() : Node("button_launch") {
    this->declare_parameters();
    this->load_config();
    this->log_config();

    // Subscriptions
    this->subButton = this->create_subscription<psaf_ucbridge_msgs::msg::Pbs>(
        "pb_data", 1,
        std::bind(&ButtonLaunchNode::button_callback, this, std::placeholders::_1)
    );
}

ButtonLaunchNode::~ButtonLaunchNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Button launch");
    std::this_thread::sleep_for(std::chrono::seconds(3));
    {
        std::lock_guard<std::mutex> lock(this->pid_mutex);
        // Stopp vorhandener Launch-Prozesse und Reset der PIDs
        stop_current_launch(this->a_pid);
        stop_current_launch(this->b_pid);
        stop_current_launch(this->c_pid);
        this->a_pid = this->b_pid = this->c_pid = -1;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

bool ButtonLaunchNode::stop_current_launch(pid_t pid) {
    if (pid > 0) {
        RCLCPP_INFO(this->get_logger(), "Stopping Launch with PID: %d", pid);

        std::thread([this, pid]() {
            kill(-pid, SIGTERM);

            std::this_thread::sleep_for(std::chrono::seconds(2));

            int status;
            int kill_attempts = 0;
            const int max_attempts = 3;

            while (waitpid(pid, &status, WNOHANG) == 0) {
                RCLCPP_WARN(this->get_logger(), "Process still running... Force Killing!");
                kill(-pid, SIGKILL); // Hard Kill, wenn Prozess noch lebt
                std::this_thread::sleep_for(std::chrono::seconds(1));
                kill_attempts++;
                if (kill_attempts >= max_attempts) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to stop process after %d attempts!", max_attempts);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Process terminated successfully.");  
                }
            }
        }).detach();
        return true;
    }
    return false;
}

pid_t ButtonLaunchNode::launch_process(const std::string &command) {
    pid_t pid = fork();
    if (pid == 0) {
        if (setpgid(0, 0) < 0) {
            perror("setpgid failed");
            _exit(EXIT_FAILURE);
        }
        // Kindprozess: Exec mit bash -c "command"
        execl("/bin/bash", "bash", "-c", command.c_str(), (char*)NULL);
        // Falls exec fehlschlägt, beenden wir den Kindprozess
        _exit(EXIT_FAILURE);
    } else if (pid < 0) {
        RCLCPP_ERROR(this->get_logger(), "Fork failed for command: %s", command.c_str());
        return -1;
    }
    // Im Elternprozess: Rückgabe der PID des Kindprozesses
    return pid;
}


bool ButtonLaunchNode::start_launch(int8_t num) {
    {
        std::lock_guard<std::mutex> lock(this->pid_mutex);
        // Stopp vorhandener Launch-Prozesse und Reset der PIDs
        stop_current_launch(this->a_pid);
        stop_current_launch(this->b_pid);
        stop_current_launch(this->c_pid);
        this->a_pid = this->b_pid = this->c_pid = -1;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    
    // Starte den Launch-Prozess in einem eigenen Thread
    std::thread([this, num]() {
        std::string launch_file = (num == 1) ? config.first :
                                  (num == 2) ? config.second : 
                                  (num == 3) ? config.third : " ";
        std::string command = "ros2 launch utility button_tokyodrift.launch.py " + launch_file;
        pid_t pid = launch_process(command);
        if (pid > 0) {
            RCLCPP_INFO(this->get_logger(), "Started command: %s, with PID %d", command.c_str(), pid);
            std::lock_guard<std::mutex> lock(pid_mutex);
            if (num == 1) {
                this->a_pid = pid;
            } else if (num == 2) {
                this->b_pid = pid;
            } else if (num == 3) {
                this->c_pid = pid;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to start launch for %s", launch_file.c_str());
        }
    }).detach();
    return true;
}

void ButtonLaunchNode::button_callback(
    const psaf_ucbridge_msgs::msg::Pbs::SharedPtr msg
) {
    int8_t pba = msg->pba;
    int8_t pbb = msg->pbb;
    int8_t pbc = msg->pbc;

    // RCLCPP_INFO(this->get_logger(), "%d, %d, %d", pba, pbb, pbc);
    if (pba > this->button_a_state + 1) {
        if (this->a_pid < 0) {
            RCLCPP_INFO(this->get_logger(), "Launching first node: %s", this->config.first.c_str());
            this->start_launch(1);
        } else {
            RCLCPP_INFO(this->get_logger(), "Stopping first node");
            this->stop_current_launch(this->a_pid);
        }
        this->set_states(pba, pbb, pbc);
        return;
    }

    if (pbb > this->button_b_state + 1) {
        if (this->b_pid < 0) {
            RCLCPP_INFO(this->get_logger(), "Launching second node: %s", this->config.second.c_str());
            this->start_launch(2);
        } else {
            RCLCPP_INFO(this->get_logger(), "Stopping second node");
            this->stop_current_launch(this->b_pid);
        }
        this->set_states(pba, pbb, pbc);
        return;
    }
     
    if (pbc > this->button_c_state + 1) {
        if (this->c_pid < 0) {
            RCLCPP_INFO(this->get_logger(), "Launching third node: %s", this->config.third.c_str());
            this->start_launch(3);
        } else {
            RCLCPP_INFO(this->get_logger(), "Stopping third node");
            this->stop_current_launch(this->c_pid);
        }
        this->set_states(pba, pbb, pbc);
        return;
    }
}

void ButtonLaunchNode::set_states(const int8_t a, const int8_t b, const int8_t c) {
    this->button_a_state = a;
    this->button_b_state = b;
    this->button_c_state = c;
}

void ButtonLaunchNode::declare_parameters() {
    this->declare_parameter("first", " ");
    this->declare_parameter("second", " ");
    this->declare_parameter("third", " ");
}

void ButtonLaunchNode::load_config() {
    this->get_parameter("first", this->config.first);
    this->get_parameter("second", this->config.second);
    this->get_parameter("third", this->config.third);
}

void ButtonLaunchNode::log_config() {
    RCLCPP_DEBUG(this->get_logger(), "####################################");
    RCLCPP_DEBUG(this->get_logger(), "First Launch: %s", this->config.first.c_str());
    RCLCPP_DEBUG(this->get_logger(), "Second Launch: %s", this->config.second.c_str());
    RCLCPP_DEBUG(this->get_logger(), "Third Launch: %s", this->config.third.c_str());
}

rcl_interfaces::msg::SetParametersResult ButtonLaunchNode::on_parameter_update(
        const std::vector<rclcpp::Parameter> &params
) {
    RCLCPP_DEBUG(this->get_logger(), "Parameter update received");
    for (const auto& param : params) {
        if (param.get_name() == "first") {
            this->config.first = param.as_string();
        } else if (param.get_name() == "second") {
            this->config.second = param.as_string();
        } else if (param.get_name() == "third") {
            this->config.third = param.as_string();
        }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ButtonLaunchNode>());
    rclcpp::shutdown();
    return 0;
}