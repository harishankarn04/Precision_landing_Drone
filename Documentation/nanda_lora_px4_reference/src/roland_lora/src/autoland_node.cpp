#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

using namespace std::chrono_literals;

enum MissionState {
  TAKEOFF,
  PATROL,
  RETURN_HOME,
  LAND
};

class AutolandNode : public rclcpp::Node {
public:
  AutolandNode() : Node("autoland_node") {
    sub_state_ = this->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", rclcpp::SensorDataQoS(), std::bind(&AutolandNode::stateCb, this, std::placeholders::_1));
    sub_target_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ekf/relative_platform_pose", 10, std::bind(&AutolandNode::targetCb, this, std::placeholders::_1));
    sub_local_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(), std::bind(&AutolandNode::localPoseCb, this, std::placeholders::_1));
      
    sub_vision_cmd_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/vision/vel_cmd", 10, std::bind(&AutolandNode::visionCb, this, std::placeholders::_1));
      
    pub_setpoint_vel_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/mavros/setpoint_velocity/cmd_vel", 10);
      
    client_arming_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    client_set_mode_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    timer_ = this->create_wall_timer(50ms, std::bind(&AutolandNode::controlLoop, this));
    last_request_ = this->now();
    last_target_time_ = this->now();
    last_vision_time_ = this->now();

    // Small 3x3 patrol box to prevent flying off the optical flow checkerboard
    patrol_waypoints_ = {
      {3.0, 0.0},
      {3.0, 3.0},
      {0.0, 3.0}
    };
  }

private:
  void stateCb(const mavros_msgs::msg::State::SharedPtr msg) { current_state_ = *msg; }
  void targetCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { 
    target_rel_pose_ = *msg; 
    last_target_time_ = this->now();
  }
  void localPoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_local_pose_ = *msg;
  }
  void visionCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    vision_cmd_ = *msg;
    last_vision_time_ = this->now();
  }
  
  void controlLoop() {
    geometry_msgs::msg::TwistStamped vel_cmd;
    vel_cmd.header.stamp = this->now();
    vel_cmd.header.frame_id = "map";

    // Smooth Altitude P-Controller
    double target_alt = 2.5;
    double z_err = target_alt - current_local_pose_.pose.position.z;
    double z_vel = std::max(-0.3, std::min(0.3, 0.8 * z_err));

    if (mission_state_ == TAKEOFF) {
      vel_cmd.twist.linear.x = 0.0;
      vel_cmd.twist.linear.y = 0.0;
      vel_cmd.twist.linear.z = z_vel; 

      if (current_local_pose_.pose.position.z > 2.3) {
        mission_state_ = PATROL;
        RCLCPP_INFO(this->get_logger(), "Takeoff complete! Starting Waypoint Patrol.");
      }
    } 
    else if (mission_state_ == PATROL) {
      double tx = patrol_waypoints_[wp_index_].first;
      double ty = patrol_waypoints_[wp_index_].second;
      
      double dx = tx - current_local_pose_.pose.position.x;
      double dy = ty - current_local_pose_.pose.position.y;
      double dist = std::sqrt(dx*dx + dy*dy);
      
      if (dist < 0.3) {
        RCLCPP_INFO(this->get_logger(), "Reached Waypoint %zu!", wp_index_);
        wp_index_++;
        if (wp_index_ >= patrol_waypoints_.size()) {
          mission_state_ = RETURN_HOME;
          RCLCPP_INFO(this->get_logger(), "Patrol complete! Using LoRa EKF to Return Home.");
        }
      } else {
        // STRICT VELOCITY LIMIT (0.4 m/s) to prevent aggressive pitch & optical flow loss
        vel_cmd.twist.linear.x = std::max(-0.4, std::min(0.4, 0.5 * dx));
        vel_cmd.twist.linear.y = std::max(-0.4, std::min(0.4, 0.5 * dy));
      }
      vel_cmd.twist.linear.z = z_vel; // Maintain 2.5m
    }
    else if (mission_state_ == RETURN_HOME || mission_state_ == LAND) {
      bool has_target = (this->now() - last_target_time_).seconds() < 2.0;

      if (has_target && !std::isnan(target_rel_pose_.pose.position.x)) {
        // STRICT VELOCITY LIMIT for homing too
        vel_cmd.twist.linear.x = std::max(-0.4, std::min(0.4, 0.8 * target_rel_pose_.pose.position.x));
        vel_cmd.twist.linear.y = std::max(-0.4, std::min(0.4, 0.8 * target_rel_pose_.pose.position.y));
        
        double target_dist = std::sqrt(std::pow(target_rel_pose_.pose.position.x, 2) + std::pow(target_rel_pose_.pose.position.y, 2));
        
        // When we are within 1.0m of the target, transition to LAND mode
        if (target_dist < 1.0) {
          if (mission_state_ != LAND) {
             RCLCPP_INFO(this->get_logger(), "LoRa localized target! Entering Precision Landing phase.");
          }
          mission_state_ = LAND;
          
          bool has_vision = (this->now() - last_vision_time_).seconds() < 0.5;
          if (has_vision) {
            vel_cmd.twist.linear.x = std::max(-0.4, std::min(0.4, vision_cmd_.twist.linear.x));
            vel_cmd.twist.linear.y = std::max(-0.4, std::min(0.4, vision_cmd_.twist.linear.y));
            vel_cmd.twist.linear.z = -0.4; // Descend confidently
          } else {
            // Fallback to LoRa homing if vision fails or hasn't seen the tag yet
            vel_cmd.twist.linear.x = std::max(-0.4, std::min(0.4, 0.8 * target_rel_pose_.pose.position.x));
            vel_cmd.twist.linear.y = std::max(-0.4, std::min(0.4, 0.8 * target_rel_pose_.pose.position.y));
            vel_cmd.twist.linear.z = -0.3; // Keep descending
          }
        } else {
          mission_state_ = RETURN_HOME;
          vel_cmd.twist.linear.z = z_vel; // Maintain 2.5m while homing
        }
      } else {
        // Target lost! Hover safely.
        vel_cmd.twist.linear.x = 0.0;
        vel_cmd.twist.linear.y = 0.0;
        vel_cmd.twist.linear.z = z_vel;
      }
    }

    pub_setpoint_vel_->publish(vel_cmd);

    if (mission_state_ == LAND && current_local_pose_.pose.position.z < 0.3) {
      if (current_state_.mode != "AUTO.LAND" && (this->now() - last_request_ > 1s)) {
        if (client_set_mode_->service_is_ready()) {
          auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
          req->custom_mode = "AUTO.LAND";
          client_set_mode_->async_send_request(req);
          RCLCPP_INFO(this->get_logger(), "Near ground! Switching to AUTO.LAND to kill motors.");
        }
        last_request_ = this->now();
      }
      return; // Skip normal offboard enforcement
    }

    // State machine to Arm and enter Offboard mode automatically
    if (current_state_.mode != "OFFBOARD" && (this->now() - last_request_ > 5s)) {
      if (client_set_mode_->service_is_ready()) {
        auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        req->custom_mode = "OFFBOARD";
        client_set_mode_->async_send_request(req);
        RCLCPP_INFO(this->get_logger(), "Requesting OFFBOARD mode");
      }
      last_request_ = this->now();
    } else if (!current_state_.armed && (this->now() - last_request_ > 5s)) {
      if (mission_state_ == LAND || mission_state_ == RETURN_HOME) {
        RCLCPP_INFO(this->get_logger(), "Mission Complete: Drone has successfully landed! Shutting down.");
        rclcpp::shutdown();
        return;
      }
      
      if (client_arming_->service_is_ready()) {
        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        req->value = true;
        client_arming_->async_send_request(req);
        RCLCPP_INFO(this->get_logger(), "Requesting ARMING");
      }
      last_request_ = this->now();
    }
  }

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr sub_state_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_target_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_local_pose_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_vision_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_setpoint_vel_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr client_arming_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr client_set_mode_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  mavros_msgs::msg::State current_state_;
  geometry_msgs::msg::PoseStamped target_rel_pose_;
  geometry_msgs::msg::PoseStamped current_local_pose_;
  geometry_msgs::msg::TwistStamped vision_cmd_;
  rclcpp::Time last_request_;
  rclcpp::Time last_target_time_;
  rclcpp::Time last_vision_time_;
  
  MissionState mission_state_ = TAKEOFF;
  std::vector<std::pair<double, double>> patrol_waypoints_;
  size_t wp_index_ = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutolandNode>());
  rclcpp::shutdown();
  return 0;
}
