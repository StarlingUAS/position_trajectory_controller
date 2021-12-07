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
#include "trajectory.hpp"

TrajectoryHandler::TrajectoryHandler() :
	Node("trajectory_handler",
		 "",
		 rclcpp::NodeOptions()
			.allow_undeclared_parameters(true)
			.automatically_declare_parameters_from_overrides(true)
	)
{
    this->callback_group_subscribers_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    this->callback_group_timers_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    this->callback_group_clients_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

    // Get Parameters
    this->get_parameter_or("execution_frequency", this->execution_frequency, 10.0); // hz
    this->get_parameter_or("transform_broadcast_frequency", this->transform_broadcast_frequency, 10.0); // hz
    this->get_parameter_or("end_extra_time", this->end_extra_time, 5.0); // seconds
    this->get_parameter_or("location_arrival_epsilon", this->location_arrival_epsilon, 0.1); // meters
    this->get_parameter_or("ground_threshold", this->ground_threshold, 0.2); // meters
    this->get_parameter_or("gotostart_duration", this->gotostart_duration, 15.0); // seconds
    this->get_parameter_or("takeoff_height", this->takeoff_height, 1.0); // seconds

    this->get_parameter_or("frame_id", this->frame_id, string("map")); 
    this->get_parameter_or("vehicle_frame_id", this->vehicle_frame_id, string("vehicle")); // meters
    this->get_parameter_or("setpoint_frame_id", this->setpoint_frame_id, string("setpoint")); // meters

    // Get Timeout Parameters
    this->state_timeout = this->get_timeout_parameter("state_timeout", 3.0);
    this->local_position_timeout = this->get_timeout_parameter("local_position_timeout", 2.0);
    this->arming_timeout = this->get_timeout_parameter("arming_timeout", 20.0);
    this->offboard_timeout = this->get_timeout_parameter("offboard_timeout", 20.0);
	this->land_timeout = this->get_timeout_parameter("land_timeout", 60.0);
    this->takeoff_timeout = this->get_timeout_parameter("takeoff_timeout", 60.0);
    this->gotostart_timeout = this->get_timeout_parameter("gotostart_timeout", 120.0);
	
    this->mission_start_receive_timeout = this->get_timeout_parameter("mission_start_receive_timeout", 3.0);

    // Initialise tf2
    this->local_position_last_broadcast_time = this->now();
    this->transform_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Initialise Service Clients
    this->mavros_arming_srv = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming", rmw_qos_profile_services_default, this->callback_group_clients_);
    this->mavros_set_mode_srv = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode", rmw_qos_profile_services_default, this->callback_group_clients_);
    this->mavros_command_srv = this->create_client<mavros_msgs::srv::CommandLong>("mavros/cmd/command", rmw_qos_profile_services_default, this->callback_group_clients_);

    // Safety Subscribers
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = this->callback_group_subscribers_;
    this->mission_start_sub = this->create_subscription<std_msgs::msg::Empty>(
        "/mission_start", 1, [this](const std_msgs::msg::Empty::SharedPtr s){(void)s; 
        this->mission_start_receive_time = std::make_shared<rclcpp::Time>(this->now());
        RCLCPP_INFO(this->get_logger(), "Mission Start Received");}, sub_opt);
    this->mission_abort_sub = this->create_subscription<std_msgs::msg::Empty>(
        "/mission_abort", 1, [this](const std_msgs::msg::Empty::SharedPtr s){(void)s; 
        if(!mission_stop_received){
            this->execution_state = State::STOP; 
            mission_stop_received = true;
            RCLCPP_INFO(this->get_logger(), "Mission Abort Received, Stopping");
        }}, sub_opt);
    this->estop_sub = this->create_subscription<std_msgs::msg::Empty>(
        "/emergency_stop", 1, [this](const std_msgs::msg::Empty::SharedPtr s){(void)s; 
        if(!mission_stop_received){
            this->execution_state = State::STOP; 
            mission_stop_received = true;
            RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP RECIEVED, Stopping");
        }
        this->emergency_stop();}, sub_opt);
    
    // Mavros Subscribers
    rclcpp::SensorDataQoS qos;
    qos.keep_last(10);
    this->state_sub =   this->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", qos, [this](const mavros_msgs::msg::State::SharedPtr s){
            this->last_received_vehicle_state = this->now(); 
            if(!this->vehicle_state){RCLCPP_INFO(this->get_logger(), "Initial mavros state received"); this->prev_vehicle_state = this->vehicle_state;}
            this->vehicle_state = s;}, sub_opt);
    this->local_position_sub =  this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "mavros/local_position/pose", qos, 
        std::bind(&TrajectoryHandler::handleLocalPosition, this, std::placeholders::_1), sub_opt);

    // Initialise Publishers
    this->setpoint_position_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 1);

    // Initialise Trajectory Service
    this->traj_serv = this->create_service<simple_offboard_msgs::srv::SubmitTrajectory>("submit_trajectory",
        std::bind(&TrajectoryHandler::submitTrajectory, this, std::placeholders::_1, std::placeholders::_2));

    this->reset();
    RCLCPP_INFO(this->get_logger(), "Controller initialised, waiting for requests on 'submit_trajectory' service.");
}

void TrajectoryHandler::emergency_stop() {
    RCLCPP_WARN(this->get_logger(), "Emergency Stop activated, sending Kill");
    // Kill Drone Rotors
    auto commandCall = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    commandCall->broadcast = false;
    commandCall->command = 400;
    commandCall->param2 = 21196.0;
    while (!this->mavros_command_srv->wait_for_service(std::chrono::duration<float>(0.1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for command service. Exiting.");
            throw std::runtime_error("Interrupted while waiting for command service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    auto result_future = this->mavros_command_srv->async_send_request(commandCall);
}

inline std::chrono::duration<double> TrajectoryHandler::get_timeout_parameter(string name, double default_param, bool invert) {
    double t;
    this->get_parameter_or(name, t, default_param);
    if(invert) {return std::chrono::duration<double>(1.0/t);}
    return std::chrono::duration<double>(t);
}

void TrajectoryHandler::handleLocalPosition(const geometry_msgs::msg::PoseStamped::SharedPtr s) {
    auto stamp = this->now();
    this->last_received_vehicle_local_position = this->now(); 

    if(!this->vehicle_local_position){
        RCLCPP_INFO(this->get_logger(), "Initial vehicle local position received");
    }

    this->vehicle_local_position = s;

    // Rate limit to rate specified
    if(stamp - this->local_position_last_broadcast_time > std::chrono::duration<double>(1.0/this->transform_broadcast_frequency)) {
        // Broadcast local position
        geometry_msgs::msg::TransformStamped tf;
        tf.child_frame_id = this->vehicle_frame_id;
        tf.transform.rotation = this->vehicle_local_position->pose.orientation;
        tf.transform.translation.x = this->vehicle_local_position->pose.position.x;
        tf.transform.translation.y = this->vehicle_local_position->pose.position.y;
        tf.transform.translation.z = this->vehicle_local_position->pose.position.z;
        tf.header.frame_id = this->frame_id;
        tf.header.stamp = stamp;
        this->transform_broadcaster->sendTransform(tf);

        this->local_position_last_broadcast_time = stamp;
    }
}

void TrajectoryHandler::reset(){
    // Clear Parameters
    this->mission_stop_received = false;
    this->executing_trajectory = false;
    this->execution_state = State::INIT;
    this->mission_start_receive_time = nullptr;
    this->prev_vehicle_state = nullptr;
    this->offboard_attempt_start = nullptr;
    this->arming_attempt_start = nullptr;
    this->takeoff_attempt_start = nullptr;
    this->land_attempt_start = nullptr;
    this->gotostart_attempt_start = nullptr;

    // Reset vehicle setpoint to zero
    this->vehicle_setpoint = std::make_shared<geometry_msgs::msg::PoseStamped>();
    this->frame_id = "map";
    this->start_time = this->now();

    // Clear Interpolators
    this->times.clear();
    for(auto d: this->demands) {
        d.clear();
    }
    this->demands.clear();
    this->interpolators.clear();
    this->gotostart_interpolators.clear();

    // Stop Execution Timer
    this->resetExecutionTimer(false);
   
    RCLCPP_INFO(this->get_logger(), "Reset Internal Parameters and Trajectory Executor");
}

void TrajectoryHandler::resetExecutionTimer(bool restart) {
    if (this->execution_timer) {
        this->execution_timer->cancel();
    }
    if(restart){
        auto execution_rate = std::chrono::duration<double>(1.0/this->execution_frequency);
        this->execution_timer = this->create_wall_timer(execution_rate, [this](){this->stateMachine(this->now());});
        RCLCPP_DEBUG(this->get_logger(), "Reset Trajectory Execution timer");
    } else {
        this->execution_timer = nullptr;
    }
}

void TrajectoryHandler::submitTrajectory(std::shared_ptr<simple_offboard_msgs::srv::SubmitTrajectory::Request> req, std::shared_ptr<simple_offboard_msgs::srv::SubmitTrajectory::Response> res) {
    RCLCPP_INFO(this->get_logger(), "Received Trajectory Execution Request");

    if(req->frame_id.empty()) {
        RCLCPP_INFO(this->get_logger(), "Frame_id not received, setting to default 'map'");
        req->frame_id = "map";
    }
    this->frame_id = req->frame_id;

    if(req->interpolation_method.empty()) {
        RCLCPP_INFO(this->get_logger(), "Interpolation method not received, setting to default 'linear'");
        req->interpolation_method == "linear";
    }

    // Check if already executing a trajectory
    if (this->executing_trajectory) {
        res->message = "Existing Trajectory Mission in Progress";
        res->success = false;
		RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
		return;
    }

    try {
        // Check if trajectory type has been given
        if (req->type != "position")
            throw std::runtime_error("Trajectory type not given or not recognised, select 'position'");


        // Check trajectory is populated
        if (req->trajectory.points.size() == 0)
            throw std::runtime_error("Trajectory does not contain any points, no execution");

    } catch (const std::exception& e) {
		res->message = e.what();
        res->success = false;
		RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
		this->reset();
		return;
	}


    // Check if valid positions:
    int num_demands = 0;
    auto tjp0 = req->trajectory.points[0];
    if(tjp0.positions.size() >= 3) {
        num_demands = tjp0.positions.size();
    } else {
        // Invalid demand? Add next one? Don't do anything?
        res->message = "Demand type not Position, Trajectory entry 0 contains not enough position";
        res->success = false;
        RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
        this->reset();
        return;
    }

    // Reset parameters
    this->reset();

    // Set parameters
    this->executing_trajectory = true;

    // Setup data
    this->max_time_sec = 0.0;
    for (int i=0; i < num_demands; i++) {
        std::vector<double> demand;
        this->demands.push_back(demand);
    }
    for (int i = 0; i < req->trajectory.points.size(); i++) {
        auto tjp = req->trajectory.points[i];

        double time = rclcpp::Duration(tjp.time_from_start).seconds();
        this->times.push_back(time);
        if (time > this->max_time_sec) {this->max_time_sec = time;}


        if(tjp.positions.size() >= 3) {
            for (int j= 0; j< tjp.positions.size(); j++) {
                this->demands[j].push_back(tjp.positions[j]);
            }
        } else {
            // Invalid demand? Add next one? Don't do anything?
            res->message = "Trajectory entry with not enough position values, t = " + std::to_string(time);
            res->success = false;
            RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
            this->reset();
            return;
        }
    }

    // Set Interpolators (https://github.com/CD3/libInterpolate)
    for (std::vector<double> demand: this->demands) {
        _1D::AnyInterpolator<double> interp;
        if (req->interpolation_method == "linear") {
            interp = _1D::LinearInterpolator<double>();
        } else if (req->interpolation_method == "cubic") {
            interp = _1D::CubicSplineInterpolator<double>();
        } else if (req->interpolation_method == "monotonic") {
            interp = _1D::MonotonicInterpolator<double>();
        } else {
            // Invalid demand? Add next one? Don't do anything?
            res->message = "Invalid Interpolation Method: " + req->interpolation_method;
            res->success = false;
            RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
            this->reset();
            return;
        }
        interp.setData(this->times.size(), this->times.data(), demand.data());
        this->interpolators.push_back(interp);
    }

    RCLCPP_INFO(this->get_logger(), "Interpolators initiated");

    // Set Takeoff Location
    this->takeoff_location = geometry_msgs::msg::Pose();
    this->takeoff_location.position.x = this->vehicle_local_position->pose.position.x;
    this->takeoff_location.position.y = this->vehicle_local_position->pose.position.y;
    this->takeoff_location.position.z = this->takeoff_height;
    this->takeoff_location.orientation = this->vehicle_local_position->pose.orientation;

    // Set Start Location and interpolator
    this->start_location = geometry_msgs::msg::Pose();
    this->start_location.position.x = req->trajectory.points[0].positions[0];
    this->start_location.position.y = req->trajectory.points[0].positions[1];
    this->start_location.position.z = req->trajectory.points[0].positions[2];
    
    double takeoff_yaw;
    if(req->trajectory.points[0].positions.size() > 3) {
        takeoff_yaw = req->trajectory.points[0].positions[3];
        tf2::Quaternion quat; 
        quat.setRPY(0.0, 0.0, takeoff_yaw);
        this->start_location.orientation = tf2::toMsg(quat);
    } else {
        takeoff_yaw = this->vehicleGetYaw();
        this->start_location.orientation = this->vehicle_local_position->pose.orientation;
    }
    std::vector<double> ttime{0.0, this->gotostart_duration};
    std::vector<double> tx{this->vehicle_local_position->pose.position.x, this->start_location.position.x};
    std::vector<double> ty{this->vehicle_local_position->pose.position.y, this->start_location.position.y};
    std::vector<double> tz{this->takeoff_height, this->start_location.position.z};
    std::vector<double> tyaw{this->vehicleGetYaw(), takeoff_yaw};
    for(int i=0; i<4;i++) {this->gotostart_interpolators.push_back(_1D::LinearInterpolator<double>());}
    this->gotostart_interpolators[0].setData(ttime, tx);
    this->gotostart_interpolators[1].setData(ttime, ty);
    this->gotostart_interpolators[2].setData(ttime, tz);    
    this->gotostart_interpolators[3].setData(ttime, tyaw);    

    // Start trajectory timer state loop (stateMachine)
    this->resetExecutionTimer();

    res->message = "Trajectory initialisation successful, executing submitted trajectory";
    res->success = true;
    RCLCPP_INFO(this->get_logger(), res->message);
}

void TrajectoryHandler::stateMachine(const rclcpp::Time& stamp){

        auto previous_state = this->execution_state;

        // Check if terminating
        if(this->execution_state == State::TERMINATE) {
            this->reset();
            return;
        }
    
    try{
        // Core saftey checks
        // Will trigger early failure if not in initialisation phase.
        bool checks = this->smChecks(stamp);
        if(!checks && this->execution_state!=State::INIT) {
            this->execution_state = State::STOP;
            RCLCPP_INFO(this->get_logger(), "State machine checks failed, switching to STOP State");
        }
    
        switch(this->execution_state) {
            case State::INIT:
                // Initialisation steps before takeoff and execution
                if(!checks) {
                    RCLCPP_INFO(this->get_logger(), "Initialisation Waiting on System Checks");
                }
                else if (!this->mission_start_receive_time || stamp - *this->mission_start_receive_time > this->mission_start_receive_timeout) {
                    RCLCPP_INFO(this->get_logger(), "Initialisation Waiting on Mission Start");
                } else {
                    this->execution_state = State::TAKEOFF;
                }
                break;
            case State::TAKEOFF:
                this->smOffboardArmed(stamp);
                if(this->smTakeoffVehicle(stamp)) {
                    this->start_time = this->now(); // Execute gotostart trajectory start time
                    this->execution_state = State::GOTOSTART;
                }
                break;
            case State::GOTOSTART:
                this->smOffboardArmed(stamp);
                if(this->smGoToStart(stamp)) {
                    this->start_time = this->now(); // Execute trajectory start time
                    this->execution_state = State::EXECUTE;
                }
                break;
            case State::EXECUTE:
                this->smOffboardArmed(stamp);
                if(this->smExecuteTrajectory(stamp)) {
                    this->execution_state = State::LAND;
                }
                break;
            
            case State::STOP:
                // Perform some actions related to early stopping due to abort or estop, then go to land
                this->execution_state = State::LAND;
                break;

            case State::LAND:
                if(this->smLandVehicle(stamp)) {
                    this->execution_state = State::MAKESAFE;
                }
                break;

            case State::MAKESAFE:
                // Make vehicle safe, i.e. disarm, then go to terminate
                if(this->smMakeSafe(stamp)) {
                    this->execution_state = State::TERMINATE;
                }
                break;

            default:
                RCLCPP_ERROR(this->get_logger(), "State machine should never get here");
        };


    }
    catch (const std::exception& e) {
		string message = e.what();
		RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
		this->execution_state = State::STOP;
	}

    if(this->execution_state != previous_state) {
        RCLCPP_INFO(this->get_logger(), "State Machine change from %s to %s", previous_state.to_string().c_str(), this->execution_state.to_string().c_str());
    }

    this->prev_vehicle_state = this->vehicle_state;
}

bool TrajectoryHandler::smChecks(const rclcpp::Time& stamp) {
    if(!vehicle_state || stamp - this->last_received_vehicle_state > this->state_timeout) {
        // State timeout
        return false;
    }
    if(!vehicle_local_position || stamp - this->last_received_vehicle_local_position > this->local_position_timeout) {
        // Local Position Timeout
        return false;
    }
    return true;
}

bool TrajectoryHandler::smOffboardArmed(const rclcpp::Time& stamp) {
    if(this->vehicle_state->mode != PX4_OFFBOARD_MODE) {
        // Not in offboard mode, send offboard message
        this->sendSetModeRequest(PX4_OFFBOARD_MODE);

        if(!this->offboard_attempt_start) {
            this->offboard_attempt_start = std::make_shared<rclcpp::Time>(stamp);
        } else if (stamp - *this->offboard_attempt_start > this->offboard_timeout) {
            throw std::runtime_error("Changing to OFFBOARD Mode timeout");
        }

        return false;
    } else {
        this->offboard_attempt_start = nullptr;
        if(this->prev_vehicle_state->mode != PX4_OFFBOARD_MODE) {
            RCLCPP_INFO(this->get_logger(), "Confirmed in OFFBOARD mode");
        }
    }

    if(!this->vehicle_state->armed) {
        // Not armed, send armed message
        RCLCPP_INFO(this->get_logger(), "Not ARMED, Sending ARMING Request");
        auto sm = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        sm->value = true;
        this->mavros_arming_srv->async_send_request(sm);

        if(!this->arming_attempt_start) {
            this->arming_attempt_start = std::make_shared<rclcpp::Time>(stamp);
        } else if (stamp - *this->arming_attempt_start > this->arming_timeout) {
            throw std::runtime_error("ARMING timeout");
        }

        return false;
    } else {
        this->arming_attempt_start = nullptr;
        if(!this->prev_vehicle_state->armed) {
            RCLCPP_INFO(this->get_logger(), "Confirmed ARMED");
        }
    }

    return true;
}

bool TrajectoryHandler::smMakeSafe(const rclcpp::Time& stamp) {
            
    // DISARM vehicle
    if(this->vehicle_state->armed) {
        // Not armed, send armed message
        RCLCPP_INFO(this->get_logger(), "ARMED, Sending DISARMING Request");
        auto sm = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        sm->value = false;
        this->mavros_arming_srv->async_send_request(sm);

        if(!this->arming_attempt_start) {
            this->arming_attempt_start = std::make_shared<rclcpp::Time>(stamp);
        } else if (stamp - *this->arming_attempt_start > this->arming_timeout) {
            throw std::runtime_error("DISARMING timeout");
        }

        return false;
    } else {
        this->arming_attempt_start = nullptr;
        if(this->prev_vehicle_state->armed) {
            RCLCPP_INFO(this->get_logger(), "Confirmed DISARMED");
        }
    }

    return true;
}

bool TrajectoryHandler::smTakeoffVehicle(const rclcpp::Time& stamp) {
    
     // Publish takeoff position as a setpoint
    this->vehicle_setpoint->pose = this->takeoff_location;
    this->sendVehicleSetpointPosition(stamp);

    // If vehicle is near setpoint then continue otherwise keep checking
    if(!this->vehicleNearLocation(this->takeoff_location)) {
        if(!this->takeoff_attempt_start) {
            this->takeoff_attempt_start = std::make_shared<rclcpp::Time>(stamp);
            RCLCPP_INFO(this->get_logger(), "Waiting for TAKEOFF to (%f, %f, %f)",
                this->vehicle_setpoint->pose.position.x,
                this->vehicle_setpoint->pose.position.y,
                this->vehicle_setpoint->pose.position.z
            );
        } else if (stamp - *this->takeoff_attempt_start > this->takeoff_timeout) {
            throw std::runtime_error("TAKEOFF timeout");
        } 
        return false;
    } else {
        this->takeoff_attempt_start = nullptr;
    }

    return true;
}

bool TrajectoryHandler::smLandVehicle(const rclcpp::Time& stamp) {
    this->sendSetModeRequest(PX4_LAND_MODE);

    if(this->vehicle_state->mode != PX4_LAND_MODE || this->vehicle_local_position->pose.position.z >= this->ground_threshold) {
        if(!this->land_attempt_start) {
            this->land_attempt_start = std::make_shared<rclcpp::Time>(stamp);
        } else if (stamp - *this->land_attempt_start > this->land_timeout) {
            throw std::runtime_error("LAND timeout");
        }
        return false;
    } else {
        this->land_attempt_start = nullptr;
    }
    return true;
}

bool TrajectoryHandler::smGoToStart(const rclcpp::Time& stamp) {
// Get time elapsed since start of trajectory
    rclcpp::Duration time_elapsed = stamp - this->start_time;
    double time_elapsed_sec = time_elapsed.seconds();

    // Is time elapsed after the maximum time in the trajectory
    if (time_elapsed_sec > this->gotostart_duration) {
        // Then stop update loop and exit
        time_elapsed_sec = this->gotostart_duration;
    }

    // Publish takeoff position as a setpoint
    this->sendSetpointPosition(stamp,
        this->gotostart_interpolators[0](time_elapsed_sec),
        this->gotostart_interpolators[1](time_elapsed_sec),
        this->gotostart_interpolators[2](time_elapsed_sec),
        this->gotostart_interpolators[3](time_elapsed_sec)
    );

    RCLCPP_INFO(this->get_logger(), "GOTOSTART setpoints (t=%f) (%f, %f, %f)", 
        time_elapsed_sec, 
        this->vehicle_setpoint->pose.position.x,
        this->vehicle_setpoint->pose.position.y,
        this->vehicle_setpoint->pose.position.z
    );

    // If vehicle is near setpoint then continue otherwise keep checking
    if(!this->vehicleNearLocation(this->start_location)) {
        if(!this->gotostart_attempt_start) {
            this->gotostart_attempt_start = std::make_shared<rclcpp::Time>(stamp);
            RCLCPP_INFO(this->get_logger(), "Waiting for GOTOSTART to (%f, %f, %f)",
                this->vehicle_setpoint->pose.position.x,
                this->vehicle_setpoint->pose.position.y,
                this->vehicle_setpoint->pose.position.z
            );
        } else if (stamp - *this->gotostart_attempt_start > this->gotostart_timeout) {
            throw std::runtime_error("GOTOSTART timeout");
        } 
        return false;
    } else {
        this->gotostart_attempt_start = nullptr;
    }

    return true;
}

bool TrajectoryHandler::smExecuteTrajectory(const rclcpp::Time& stamp) {

    // Get time elapsed since start of trajectory
    rclcpp::Duration time_elapsed = stamp - this->start_time;
    double time_elapsed_sec = time_elapsed.seconds();
    bool completed = false;

    // Is time elapsed after the maximum time in the trajectory
    if (time_elapsed_sec > this->max_time_sec) {
        // Then stop update loop and exit
        RCLCPP_INFO(this->get_logger(), "Reached final trajectory time");
        time_elapsed_sec = this->max_time_sec;
        completed = true;
    }

    // Publish position as a setpoint
    this->sendSetpointPosition(stamp,
        this->interpolators[0](time_elapsed_sec),
        this->interpolators[1](time_elapsed_sec),
        this->interpolators[2](time_elapsed_sec),
        this->interpolators.size()>3?this->interpolators[3](time_elapsed_sec):0.0
    );

    RCLCPP_INFO(this->get_logger(), "Sent request (t=%f) (%f, %f, %f)", 
        time_elapsed_sec, 
        this->vehicle_setpoint->pose.position.x,
        this->vehicle_setpoint->pose.position.y,
        this->vehicle_setpoint->pose.position.z
    );

    // If Reached Maximum time (completed), and drone is on the final location, then continue
    // May need a timeout just in case 
    if(completed && this->vehicleNearLocation(this->vehicle_setpoint->pose)) {
        RCLCPP_INFO(this->get_logger(), "Trajectory Completed");
        return true;
    }

    return false;
}

bool TrajectoryHandler::vehicleNearLocation(const geometry_msgs::msg::Pose& location) {
    if(!this->vehicle_local_position) {
        throw std::runtime_error("Vehicle location not received");
    }
    auto pose = this->vehicle_local_position->pose;

    // Right now only match coordinate and not orientation
    bool zcomp = fabs(pose.position.z - location.position.z) < this->location_arrival_epsilon;
    bool xcomp = fabs(pose.position.x - location.position.x) < this->location_arrival_epsilon;
    bool ycomp = fabs(pose.position.y - location.position.y) < this->location_arrival_epsilon;
    return xcomp && ycomp && zcomp;
}

void TrajectoryHandler::sendSetModeRequest(string custom_mode) {
    auto sm = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    sm->custom_mode = custom_mode;

    while (!this->mavros_set_mode_srv->wait_for_service(std::chrono::duration<int>(2))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for set mode service. Exiting.");
            throw std::runtime_error("Interrupted while waiting for set mode service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    this->mavros_set_mode_srv->async_send_request(sm);
    RCLCPP_INFO(this->get_logger(), "Current mode is %s, Sending %s Set Mode Request", this->vehicle_state->mode.c_str(), custom_mode.c_str());
}

void TrajectoryHandler::sendSetpointPosition(const rclcpp::Time& stamp, const double x, const double y, const double z, const double yaw) {
    this->vehicle_setpoint->pose.position.x = x;
    this->vehicle_setpoint->pose.position.y = y;
    this->vehicle_setpoint->pose.position.z = z;
    
    tf2::Quaternion quat; 
    quat.setRPY(0.0, 0.0, yaw);
    this->vehicle_setpoint->pose.orientation = tf2::toMsg(quat);

    this->sendVehicleSetpointPosition(stamp);
}

void TrajectoryHandler::sendVehicleSetpointPosition(const rclcpp::Time& stamp) {
    // Publish position as a setpoint
    this->setpoint_position_pub->publish(*this->vehicle_setpoint);

    // Publish as transform
    geometry_msgs::msg::TransformStamped tf;
    tf.child_frame_id = this->setpoint_frame_id;
    tf.transform.rotation = this->vehicle_setpoint->pose.orientation;
    tf.transform.translation.x = this->vehicle_setpoint->pose.position.x;
    tf.transform.translation.y = this->vehicle_setpoint->pose.position.y;
    tf.transform.translation.z = this->vehicle_setpoint->pose.position.z;
    tf.header.frame_id = this->frame_id;
    tf.header.stamp = stamp;
    this->transform_broadcaster->sendTransform(tf);
}


double TrajectoryHandler::vehicleGetYaw() {
    // see https://stackoverflow.com/a/37560411
    auto q = this->vehicle_local_position->pose.orientation;
    return atan2(2.0 * (q.z * q.w + q.x * q.y) , - 1.0 + 2.0 * (q.w * q.w + q.x * q.x));
}

void TrajectoryHandler::printVehiclePosition() {
    RCLCPP_INFO(this->get_logger(),
        "Vehicle Position is (%f, %f, %f)",
        vehicle_local_position->pose.position.x,
        vehicle_local_position->pose.position.y,
        vehicle_local_position->pose.position.z
    );
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto handler = std::make_shared<TrajectoryHandler>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(handler);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}