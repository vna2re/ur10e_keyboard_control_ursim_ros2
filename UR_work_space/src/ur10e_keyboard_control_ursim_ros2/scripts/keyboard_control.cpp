#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <iostream>

class KeyboardControl : public rclcpp::Node {
public:
    KeyboardControl() : Node("keyboard_control"), active_joint_(0) {
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/scaled_joint_trajectory_controller/joint_trajectory", 10);

        joint_names_ = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        };

        positions_.resize(joint_names_.size(), 0.0);

        std::cout << "Use arrow keys: ←/→ to switch joint, ↑/↓ to move. Press 'q' to quit.\n";

        loop();
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    std::vector<std::string> joint_names_;
    std::vector<double> positions_;
    int active_joint_;

    int get_key() {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        struct timeval tv {0, 0};
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        int ret = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv);
        int ch = -1;
        if (ret > 0) {
            ch = getchar();
            if (ch == 27 && getchar() == '[') {
                ch = getchar();
                switch (ch) {
                    case 'A': ch = 'U'; break; // Up
                    case 'B': ch = 'D'; break; // Down
                    case 'C': ch = 'R'; break; // Right
                    case 'D': ch = 'L'; break; // Left
                    default: break;
                }
            }
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }

    void send_trajectory() {
        auto msg = trajectory_msgs::msg::JointTrajectory();
        msg.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = positions_;
        point.time_from_start.sec = 1;

        msg.points.push_back(point);
        traj_pub_->publish(msg);
    }

    void loop() {
        rclcpp::Rate rate(10);
        while (rclcpp::ok()) {
            int key = get_key();

            if (key == 'q') break;
            if (key == 'R') {
                active_joint_ = (active_joint_ + 1) % joint_names_.size();
                std::cout << "Selected joint " << active_joint_ << ": " << joint_names_[active_joint_] << "\n";
            } else if (key == 'L') {
                active_joint_ = (active_joint_ - 1 + joint_names_.size()) % joint_names_.size();
                std::cout << "Selected joint " << active_joint_ << ": " << joint_names_[active_joint_] << "\n";
            } else if (key == 'U') {
                positions_[active_joint_] += 0.1;
                std::cout << joint_names_[active_joint_] << ": " << positions_[active_joint_] << "\n";
                send_trajectory();
            } else if (key == 'D') {
                positions_[active_joint_] -= 0.1;
                std::cout << joint_names_[active_joint_] << ": " << positions_[active_joint_] << "\n";
                send_trajectory();
            }

            rate.sleep();
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardControl>();
    rclcpp::shutdown();
    return 0;
}

