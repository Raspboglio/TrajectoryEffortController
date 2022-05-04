#include <trajectory_effort_controller/trajectory_effort_controller.hpp>
#include <realtime_tools/realtime_buffer.h>


#include <iostream>

namespace trajectory_effort_controller{

TrajectoryEffortController::TrajectoryEffortController(){


}

controller_interface::return_type TrajectoryEffortController::init(const std::string &controller_name){
	controller_interface::return_type res = ControllerInterface::init(controller_name);
	RCLCPP_INFO(node_->get_logger(), "Init TrajectoryEffortController");
	
	if(res != controller_interface::return_type::OK){
		return res;
	}

	_sub = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>("/trajectory_effort_controller/trajectory", 10, std::bind(&TrajectoryEffortController::subCallback, this, std::placeholders::_1));

	auto_declare<std::vector<std::string>>("joints", _joint_names);
	auto_declare<std::vector<std::string>>("commands", _command_interfaces);
	auto_declare<std::vector<std::string>>("states", _state_interfaces);
	auto_declare<double>("p", _p);
	auto_declare<double>("i",_i);
	auto_declare<double>("d",_d);
	auto_declare<double>("toll", _toll);
	auto_declare<int>("resolution", _resolution);

	return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration TrajectoryEffortController::command_interface_configuration() const {
	controller_interface::InterfaceConfiguration config;
	RCLCPP_INFO(node_->get_logger(), "number of command interfaces: %d", _command_interfaces.size());
	for(auto name: _joint_names){
		for(auto comm: _command_interfaces){
			std::string comm_name;
			comm_name = name;
			comm_name.append("/");
			comm_name.append(comm);
			RCLCPP_INFO(node_->get_logger(), "adding command interface %s", comm_name.c_str());
			config.names.push_back(comm_name);
		}
	}
	config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
	return config;
}

controller_interface::InterfaceConfiguration TrajectoryEffortController::state_interface_configuration() const{
	controller_interface::InterfaceConfiguration config;
	config.names = _state_interfaces;
	config.type = controller_interface::interface_configuration_type::ALL;
	return config;
}

controller_interface::return_type TrajectoryEffortController::update(){
	// Get current interface values;
	// RCLCPP_INFO(node_->get_logger(), "Updating");
	std::vector<double> current_pos = std::vector<double>(_joint_names.size());
	std::vector<double> current_vel = std::vector<double>(_joint_names.size());
	std::vector<double> current_eff = std::vector<double>(_joint_names.size());

	std::vector<hardware_interface::LoanedStateInterface>::iterator state_it = this->state_interfaces_.begin();

	for(; state_it != state_interfaces_.end(); state_it++){
		if(state_it->get_interface_name() == "position"){
			
			auto name_it =  std::find(_joint_names.begin(), _joint_names.end(), state_it->get_name());
			int index = name_it - _joint_names.begin();
			// RCLCPP_INFO(node_->get_logger(), "Adding interface: full_name: %s at %d ", state_it->get_full_name().c_str(), index);
			current_pos[index] = state_it->get_value();

		}else if(state_it->get_interface_name() == "velocity"){
			auto name_it = std::find(_joint_names.begin(), _joint_names.end(), state_it->get_name());
			int index = name_it - _joint_names.begin();
			current_vel[index] = state_it->get_value();
		}else if(state_it->get_interface_name() == "effort"){
			auto name_it = std::find(_joint_names.begin(), _joint_names.end(), state_it->get_name());
			int index = name_it -_joint_names.begin();
			current_eff[index] = state_it->get_value();
		} 
	}

	
	std::vector<hardware_interface::LoanedCommandInterface>::iterator command_it = this->command_interfaces_.begin();
	int index = 0;


	int64_t curr_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();	
	for(; command_it != command_interfaces_.end(); command_it++){
		// RCLCPP_INFO(node_->get_logger(), "command interface name: %s", command_it->get_full_name().c_str());
		if(command_it->get_interface_name() == "effort"){
			auto name_it = std::find(_joint_names.begin(), _joint_names.end(), command_it->get_name());
			int index = name_it - _joint_names.begin();
			if(index > current_pos.size()){
				RCLCPP_ERROR(node_->get_logger(), "can't find command joint");
				return controller_interface::return_type::ERROR;
			}
			_curr_state = current_pos[index];
			// Compute the expected current value due to trajectory
			double next_target = _targets.begin()->velocities[index] * (curr_time - _last_update) + _last_point.positions[index]; 
			_pid[index].setTarget(next_target);
			_last_point.positions[index] = next_target;
			
			_pid[index].tick();
			// RCLCPP_INFO(node_->get_logger(), "Ticking pid %d  current state: %.4f    next command: %.4f", index, _curr_state, _next_command);
			command_it->set_value(_next_command);
			
			// double error = _pid[index].getError();
			// if(error > (-_toll) && error < _toll && reached){
			// 	reached = false;
			// }

			// RCLCPP_INFO(node_->get_logger(), "Setting effort at joint %s  error %.2f  value %.2f",command_it->get_full_name().c_str(),pos_err.at(index), pos_err.at(index) * p);
			
			// _curr_command_it->set_value(pos_err.at(index) * _p);
		}

		// if(reached && _targets.size() > 1){
		// 	//TODO: check that the time of current target is reached
		// 	int64_t target_time = _targets.begin()->time_from_start.sec * 1000 + _targets.begin()->time_from_start.nanosec / 1000000;
		// 	int64_t curr_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		// 	if(curr_time - _start_time >= target_time){
		// 		_targets.erase(_targets.begin());
		// 		for(size_t i = 0; i < _targets.begin()->positions.size() && i < _pid.size(); i++){
		// 			_pid[i].setTarget(_targets.begin()->positions[i]);
		// 		}
		// 	}
		// }
		
		
		
	}
	int64_t target_time = _targets.begin()->time_from_start.sec * 1000 + _targets.begin()->time_from_start.nanosec / 1000000;
	if(curr_time - _start_time > target_time ){
		// RCLCPP_INFO(node_->get_logger(), "out of time!!!");
		if(_targets.size() > 1){
			_targets.erase(_targets.begin());
		}else{
			// Stop joint and set position as pid target
			
			for(size_t joint_index = 0; joint_index < _pid.size(); joint_index++){
				// RCLCPP_INFO(node_->get_logger(),"stopping joint");
				_targets.begin()->velocities[joint_index] = 0;
				_pid[joint_index].setTarget(_targets.begin()->positions[joint_index]);
			}
		}	
	}
	_last_update = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		
	

	return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TrajectoryEffortController::on_configure(const rclcpp_lifecycle::State & previous_state){
	
	// TODO: add check on validity of commands and states
	_joint_names = node_->get_parameter("joints").as_string_array();
	_command_interfaces = node_->get_parameter("commands").as_string_array();
	for(auto comm : _command_interfaces){
		if(comm != "effort" && comm != "position" && comm != "velocity"){
			RCLCPP_ERROR(node_->get_logger(), "Invalid command interface %s", comm);
			return CallbackReturn::ERROR;
		}
	}
	_state_interfaces = node_->get_parameter("states").as_string_array();
	for(auto state : _state_interfaces){
		if(state != "effort" && state != "position" && state != "velocity"){
			RCLCPP_ERROR(node_->get_logger(), "Invalid state interface %s", state);
			return CallbackReturn::ERROR;
		}
	}
	// TODO: turn p, i, d into vectors?
	_p = node_->get_parameter("p").as_double();
	_i = node_->get_parameter("i").as_double();
	_d = node_->get_parameter("d").as_double();
	_resolution = node_->get_parameter("resolution").as_int();

	// for(auto elem : _command_interfaces){
	// 	RCLCPP_INFO(node_->get_logger(), "command: %s", elem.c_str());
	// }
	std::sort(_joint_names.begin(),_joint_names.end());
	RCLCPP_INFO(node_->get_logger(), "Trajectory Effort Controller configured");
	// for(auto elem : _joint_names){
	// 	RCLCPP_INFO(node_->get_logger(),elem);
	// }

	for(auto elem : _joint_names){
		RCLCPP_INFO(node_->get_logger(), "adding pid controller %s", elem.c_str());
		PIDController<double> tmp( _p, _i, _d, pidFeedback, pidOutput);
		_pid.push_back(tmp);
	}
	// RCLCPP_INFO(node_->get_logger(), "created %d pid", _pid.size());
	return CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TrajectoryEffortController::on_activate(const rclcpp_lifecycle::State & previous_state){
	std::vector<hardware_interface::LoanedStateInterface>::iterator state_it = this->state_interfaces_.begin();
	trajectory_msgs::msg::JointTrajectoryPoint cur_point;
	std::vector<double> cur_pos = std::vector<double>(_joint_names.size());
	for(; state_it != state_interfaces_.end(); state_it++){
		
		if(state_it->get_interface_name() == "position"){
			
			auto name_it =  std::find(_joint_names.begin(), _joint_names.end(), state_it->get_name());
			int index = name_it - _joint_names.begin();
			// RCLCPP_INFO(node_->get_logger(), "Adding interface: full_name: %s at %d ", state_it->get_full_name().c_str(), index);
			cur_pos[index] = state_it->get_value();
			_pid[index].setTarget(state_it->get_value());
			// RCLCPP_INFO(node_->get_logger(), " enabling pid");
			_pid[index].setEnabled(true);
		}
	}

	cur_point.positions = cur_pos;
	cur_point.velocities = std::vector<double>(_joint_names.size());
	cur_point.accelerations = std::vector<double>(_joint_names.size());
	cur_point.effort = std::vector<double>(_joint_names.size());

	_targets.push_back(cur_point);
	_last_point = cur_point;

	_start_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	_last_update = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	
	return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TrajectoryEffortController::on_deactivate(const rclcpp_lifecycle::State & previous_state){
	_targets.clear();
	RCLCPP_INFO(node_->get_logger(), "Trajectory Effort Controller deactivated");
	auto pid_it = _pid.begin();
	for(; pid_it != _pid.end(); pid_it++){
		pid_it->setEnabled(false);
		
	}
	return CallbackReturn::SUCCESS;
}


void TrajectoryEffortController::subCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg){
	
	//RCLCPP_INFO(node_->get_logger(),"New message received");

	// Check message size
	if(msg->joint_names.size() != this->_joint_names.size()){
		RCLCPP_ERROR(node_->get_logger(), "Error in subCallback, message number of joint doesn't match interface number of joint");
		return;
	}
	////////////////////////////////// Erase
	// Each time we obtain a new message we ignore the current trajectory
	trajectory_msgs::msg::JointTrajectoryPoint current = _targets.front();
	_targets.clear();
	_last_point = current;
	double last_time = 0;
	for(auto elem: msg->points){
		///////////////////////////// Sorting
		RCLCPP_INFO(node_->get_logger(), "Got new Message");
		std::vector<std::string>::iterator names_it = _joint_names.begin();
		//sort 1 element
		trajectory_msgs::msg::JointTrajectoryPoint sorted_elem;
		for(; names_it != _joint_names.end(); names_it++){
			std::vector<std::string>::iterator msg_it = std::find(msg->joint_names.begin(), msg->joint_names.end(),*names_it);
			
			if(msg_it == msg->joint_names.end()){
				RCLCPP_ERROR(node_->get_logger(),"ERROR reading new message, all the joint must be specified");
				return ;
			}else{
			
				int index = msg_it - msg->joint_names.begin();
				// RCLCPP_INFO(node_->get_logger(), "inserting node %s", msg_it->c_str());
				if(elem.positions.size() > index){
					sorted_elem.positions.push_back(elem.positions[index]);	
				}else{
					RCLCPP_ERROR(node_->get_logger(), "Position number is too small, all positions must be specified. The point is ignored");
					return;
				}

				if(elem.accelerations.size() > index){
					sorted_elem.accelerations.push_back(elem.accelerations[index]);
				}else{
					sorted_elem.accelerations.push_back(0);
				}

				if(elem.effort.size() > index){
					sorted_elem.effort.push_back(elem.effort[index]);
				}else{
					sorted_elem.effort.push_back(0);
				}

				if(elem.velocities.size() > index){
					sorted_elem.velocities.push_back(elem.velocities[index]);
				}else{
					sorted_elem.velocities.push_back(0);
				}
			
			}
			
		}
		sorted_elem.time_from_start = elem.time_from_start;
		int64_t target_time = sorted_elem.time_from_start.sec*1000 - sorted_elem.time_from_start.nanosec / 1000000;
		
		// Speed evaluation
		for(size_t joint_index = 0; joint_index < _joint_names.size(); joint_index++){
			
			sorted_elem.velocities[joint_index] = (sorted_elem.positions[joint_index] - current.positions[joint_index]) / (target_time - last_time); 
			// RCLCPP_INFO(node_->get_logger(),"%d velocity : %.9f", joint_index, sorted_elem.velocities[joint_index]);
		}

		last_time = target_time;
		current = sorted_elem;
		_targets.push_back(sorted_elem);
	}

	_start_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	_last_update= std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	
}

double TrajectoryEffortController::pidFeedback(){
	return _curr_state;
}

void TrajectoryEffortController::pidOutput(double output){
	_next_command = output;
}


double TrajectoryEffortController::_curr_state = 0.0;
double TrajectoryEffortController::_next_command = 0.0;



} //namespace trajectory_effort_controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(trajectory_effort_controller::TrajectoryEffortController, controller_interface::ControllerInterface)