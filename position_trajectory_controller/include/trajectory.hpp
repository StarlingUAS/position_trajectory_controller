#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

/*
 * Trajectory follower
 * Copyright (C) 2021 University of Bristol
 *
 * Author: Mickey Li <mickey.li@bristol.ac.uk> (University of Bristol)
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <stdexcept>
#include <thread>
#include <atomic>
#include <cmath>

#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <tf2/utils.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>

#include "simple_offboard_msgs/srv/submit_trajectory.hpp"

#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "mavros_msgs/msg/thrust.hpp"
#include "mavros_msgs/msg/status_text.hpp"
#include "mavros_msgs/msg/manual_control.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_long.hpp"

#include <libInterpolate/Interpolate.hpp>
#include <libInterpolate/AnyInterpolator.hpp>

using namespace std;

#define PX4_OFFBOARD_MODE "OFFBOARD"
#define PX4_LAND_MODE "AUTO.LAND"

class State{
    public:
        enum Value: uint8_t {
            INIT,
            TAKEOFF,
            GOTOSTART,
            LAND,
            STOP,
            EXECUTE,
            MAKESAFE,
            TERMINATE,
        };
    
    State() = default;
    constexpr State(Value state) : value(state) { }
    
    // Allow switch and comparisons.
    constexpr operator Value() const { return value; }

    // Prevent usage: if(fruit)
    explicit operator bool() = delete;        

    constexpr bool operator==(State a) const { return value == a.value; }
    constexpr bool operator!=(State a) const { return value != a.value; }

    constexpr bool operator==(State::Value a) const { return value == a; }
    constexpr bool operator!=(State::Value a) const { return value != a; }


    string to_string() const {
        switch(value){
            case INIT:
                return "STATE::INIT";
            case TAKEOFF:
                return "STATE::TAKEOFF";
            case GOTOSTART:
                return "STATE::GOTOSTART";
            case LAND:
                return "STATE::LAND";
            case STOP:
                return "STATE::STOP";
            case EXECUTE:
                return "STATE::EXECUTE";
            case TERMINATE:
                return "STATE::TERMINATE";
            case MAKESAFE:
                return "STATE::MAKESAFE";
            default:
                return "INVALID_STATE";
        };
    }

    private:
        Value value;
};

class TrajectoryHandler : public rclcpp::Node
{
    public:
        TrajectoryHandler();
        void submitTrajectory(std::shared_ptr<simple_offboard_msgs::srv::SubmitTrajectory::Request> req, std::shared_ptr<simple_offboard_msgs::srv::SubmitTrajectory::Response> res);

    private:
        void reset();
        void resetExecutionTimer(bool restart=true);
        void stateMachine(const rclcpp::Time& stamp);
        void emergency_stop();

        // State machine functions
        bool smChecks(const rclcpp::Time& stamp);  
        bool smOffboardArmed(const rclcpp::Time& stamp);      
        bool smTakeoffVehicle(const rclcpp::Time& stamp);
        bool smGoToStart(const rclcpp::Time& stamp);
        bool smLandVehicle(const rclcpp::Time& stamp);
        bool smMakeSafe(const rclcpp::Time& stamp);
        bool smExecuteTrajectory(const rclcpp::Time& stamp);
        void gotoTrajectoryPoint(const trajectory_msgs::msg::JointTrajectoryPoint& point);
        
        // Helper utility functions
        std::chrono::duration<double> get_timeout_parameter(string name, double default_param, bool invert = false);
        void sendSetModeRequest(string custom_mode);
        bool vehicleNearLocation(const geometry_msgs::msg::Pose& location);
        void printVehiclePosition();
        void sendSetpointPosition(const rclcpp::Time& stamp, const double x, const double y, const double z, const double yaw=0.0);
        void sendVehicleSetpointPosition(const rclcpp::Time& stamp);
        void handleLocalPosition(const geometry_msgs::msg::PoseStamped::SharedPtr s);
        double vehicleGetYaw();

        // Mission parameters
        std::shared_ptr<rclcpp::Time> mission_start_receive_time;
        bool mission_stop_received = false;

        // Execution parameters
        std::atomic<bool> executing_trajectory; // Is a trajectory being executed
        rclcpp::Time start_time;                // Trajectory Start time (after takeoff)
        double max_time_sec;                    // The end time of the trajectory              
        rclcpp::TimerBase::SharedPtr execution_timer;   // Timer which runs the execution state machine
        State execution_state;  // Keep Track of state

        // Interpolators
        std::vector<double> times;
        std::vector<std::vector<double>> demands;
        std::vector<_1D::AnyInterpolator<double>> interpolators;

        // External parameters
        std::string frame_id = "map";           // Default Frame of reference
        double execution_frequency;             // Setpoint frequency
        double transform_broadcast_frequency;  // Transform Broadcast frequency
        double end_extra_time;                  // Extra time to give after finishing trajectory
        double location_arrival_epsilon;        // Determine if vehicle is near setpoint
        double ground_threshold;                // Below ground threshold indicates that vehicle has landed
        double gotostart_duration;                // Time given for takeoff to occur
        double takeoff_height;                  // Initial takeoff height

        // Timeouts and timing
        std::chrono::duration<double> local_position_timeout;
        std::chrono::duration<double> state_timeout;
        
        std::shared_ptr<rclcpp::Time> offboard_attempt_start;
        std::chrono::duration<double> offboard_timeout;
        std::shared_ptr<rclcpp::Time> arming_attempt_start;
        std::chrono::duration<double> arming_timeout;

        std::shared_ptr<rclcpp::Time> takeoff_attempt_start;
        std::chrono::duration<double> takeoff_timeout;
        std::shared_ptr<rclcpp::Time> gotostart_attempt_start;
        std::chrono::duration<double> gotostart_timeout;
        std::shared_ptr<rclcpp::Time> land_attempt_start;
        std::chrono::duration<double> land_timeout;

        std::chrono::duration<double> mission_start_receive_timeout;

        // Takeoff and Landing parameters
        geometry_msgs::msg::Pose takeoff_location;
        geometry_msgs::msg::Pose start_location;
        std::vector<_1D::LinearInterpolator<double>> gotostart_interpolators;

        // Vehicle State
        rclcpp::Time last_received_vehicle_state;
        std::shared_ptr<mavros_msgs::msg::State> vehicle_state;
        std::shared_ptr<mavros_msgs::msg::State> prev_vehicle_state;
        rclcpp::Time last_received_vehicle_local_position;
        std::shared_ptr<geometry_msgs::msg::PoseStamped> vehicle_local_position;
        std::shared_ptr<geometry_msgs::msg::PoseStamped> vehicle_setpoint;

        // TF Broadcasting
        string vehicle_frame_id;
        string setpoint_frame_id;
        rclcpp::Time local_position_last_broadcast_time;
        std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster;

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr       setpoint_position_pub;

        // Subscriptions
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr               mission_start_sub;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr               mission_abort_sub;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr               estop_sub;
        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr         state_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr                     local_position_sub;

        // Clients
        rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr        mavros_arming_srv;
        rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr            mavros_set_mode_srv;
        rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr        mavros_command_srv;       

        // Service
        rclcpp::Service<simple_offboard_msgs::srv::SubmitTrajectory>::SharedPtr      traj_serv;

        // Multithreaded callback groups
        rclcpp::CallbackGroup::SharedPtr callback_group_timers_;
        rclcpp::CallbackGroup::SharedPtr callback_group_subscribers_;
        rclcpp::CallbackGroup::SharedPtr callback_group_clients_;
};


#endif
