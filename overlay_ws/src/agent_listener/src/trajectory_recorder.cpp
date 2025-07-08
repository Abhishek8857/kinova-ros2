# include <memory>
# include <rclcpp/rclcpp.hpp>
# include <geometry_msgs/msg/point.hpp>
# include <sensor_msgs/msg/joint_state.hpp>
# include <moveit/move_group_interface/move_group_interface.h>
# include <std_msgs/msg/float64_multi_array.hpp>
# include <std_msgs/msg/bool.hpp>
# include <thread>
# include <chrono>
# include <yaml-cpp/yaml.h>
# include <fstream>
# include <iomanip>
# include <filesystem>

using std::placeholders::_1;

class TrajectoryRecorder : public rclcpp::Node
{
    public: 
        // Initialise the Node 
        TrajectoryRecorder() : Node("trajectory_recorder")
        {
            RCLCPP_INFO(this->get_logger(), "Trajectory Recorder Initialised");

            this->declare_parameter<std::string>("output_dir", "/kinova-ros2/trajectories/");
            output_dir = this->get_parameter("output_dir").as_string();

            std::filesystem::path dir(output_dir);
            std::error_code ec;

            // Check if directory exists and delete its contents
            if (std::filesystem::exists(dir))
            {
                RCLCPP_INFO(this->get_logger(), "Clearing contents of existing folder: %s", output_dir.c_str());
                for (const auto& entry : std::filesystem::directory_iterator(dir))
                {
                    std::filesystem::remove_all(entry.path(), ec);
                    if (ec)
                    {
                        RCLCPP_WARN(this->get_logger(), "Could not delete %s: %s",
                                    entry.path().c_str(), ec.message().c_str());
                    }
                }
            }
            else
            {
                std::filesystem::create_directories(dir, ec);
                if (ec)
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to create directory %s: %s", output_dir.c_str(), ec.message().c_str());
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Created output directory: %s", output_dir.c_str());
                }
            }

            joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&TrajectoryRecorder::joint_state_callback, this, _1));

            timer = this->create_wall_timer(
                std::chrono::seconds(40), 
                std::bind(&TrajectoryRecorder::saveToFile, this));
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
        rclcpp::TimerBase::SharedPtr timer;
        std::vector<sensor_msgs::msg::JointState> recorded_states_;
        std::string output_dir;
        rclcpp::Time last_change_time;
        sensor_msgs::msg::JointState last_saved_state;
        bool first_sample = true;
        double position_threshold = 0.001;

        const std::vector<std::string> joint_order = {
                "joint_1",
                "robotiq_85_left_knuckle_joint",
                "joint_2",
                "joint_4",
                "joint_5",  
                "joint_3",
                "joint_6",
                "joint_7",
            };


        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            last_change_time = this->now();
            
            if (first_sample)
            {
                recorded_states_.push_back(*msg);
                last_saved_state = *msg;
                first_sample = false;
                return;
            }

            if (has_significant_change(*msg, last_saved_state, position_threshold))
            {
                recorded_states_.push_back(*msg);
                last_saved_state = *msg;
            }
        }

        bool has_significant_change (
        const sensor_msgs::msg::JointState &current, 
        const sensor_msgs::msg::JointState &last, 
        double threshold
        )
        {
            std::map<std::string, double> last_positions;
            for(size_t i = 0;i < last.name.size(); ++i)
            {
                last_positions[last.name[i]] = last.position[i];
            }

            for (size_t i = 0; i < current.name.size(); ++i)
            {
                const std::string &joint = current.name[i];
                double pos = current.position[i];

                if (std::find(joint_order.begin(), joint_order.end(), joint) == joint_order.end())
                {
                    continue;
                }
                
                if (last_positions.count(joint))
                {
                    if(std::abs(pos - last_positions[joint]) > threshold)
                    {
                        return true;
                    }
                }
                else
                {
                    return true;
                }
            }

            return false;
        }

        
        void saveToFile()
        {
            if (recorded_states_.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No joint states recorded. Skipping save.");
                return;
            }



            std::string timestamp = getTimestamp();
            std::string filename = output_dir + "/joint_states_" + timestamp + ".yaml";

            YAML::Emitter out;
            out << YAML::BeginMap;
            out << YAML::Key << "joint_order" << YAML::Value << joint_order;
            out << YAML::Key << "points" << YAML::Value << YAML::BeginSeq;

            for (const auto& state : recorded_states_)
            {
                std::map<std::string, double> pos_map;
                std::map<std::string, double> vel_map;

                // Build map only from joints that are in joint_order
                for (size_t i = 0; i < state.name.size(); ++i)
                {
                    const std::string &joint_name = state.name[i];
                    if (std::find(joint_order.begin(), joint_order.end(), joint_name) != joint_order.end())
                    {
                        pos_map[joint_name] = state.position[i];
                        if (i < state.velocity.size())
                            vel_map[joint_name] = state.velocity[i];
                    }
                }

                std::vector<double> ordered_pos, ordered_vel;
                for (const auto &joint_name : joint_order)
                {
                    ordered_pos.push_back(pos_map.count(joint_name) ? pos_map[joint_name] : 0.0);
                    ordered_vel.push_back(vel_map.count(joint_name) ? vel_map[joint_name] : 0.0);
                }

                out << YAML::BeginMap;
                out << YAML::Key << "positions" << YAML::Value << YAML::Flow << ordered_pos;
                out << YAML::Key << "velocities" << YAML::Value << YAML::Flow << ordered_vel;
                out << YAML::Key << "time_from_start" << YAML::Value
                    << (state.header.stamp.sec + state.header.stamp.nanosec / 1e9);
                out << YAML::EndMap;
            }

            out << YAML::EndSeq;
            out << YAML::EndMap;

            std::ofstream fout(filename);
            fout << out.c_str();
            RCLCPP_INFO(this->get_logger(), "Saved joint states to %s", filename.c_str());

            recorded_states_.clear();
        }

        std::string getTimestamp()
        {
            auto now = std::chrono::system_clock::now();
            auto now_c = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
            return ss.str();
        }
};




int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryRecorder>());
    rclcpp::shutdown();
    return 0;
}