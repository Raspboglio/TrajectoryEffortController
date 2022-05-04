#ifndef TRAJECTORY_EFFORT_HPP
#define TRAJECTORY_EFFORT_HPP

#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <external/PID/cpp/PID.h>
#include <chrono>

namespace trajectory_effort_controller{

    // TODO: move the trajectory interpolation inside this plugin? 

    // @brief this controller provides the same functionalities as the joint trajectory controller but 
    // uses effort as interface.
    class TrajectoryEffortController : public controller_interface::ControllerInterface{
    public:
        
        TrajectoryEffortController();

        // @brief initialize the parameters
        // @param joints: vector of joint name
        // @param command: list of command interfaces
        // @param states: list of states
        controller_interface::return_type init(const std::string &controller_name) override;
        
        /**
         * @brief command_interface_configuration This controller requires the position command
         * interfaces for the controlled joints
         */
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        /**
         * @brief command_interface_configuration This controller requires the position and velocity
         * state interfaces for the controlled joints
         */
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        
        controller_interface::return_type update() override;

        
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;

        
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;

        
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;
        static double _next_command;
        static double _curr_state;

    private:

        void subCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
        static double pidFeedback();
        static void pidOutput(double output);

        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr _sub;
        
        std::vector<std::string> _joint_names;
        std::vector<std::string> _command_interfaces;
        std::vector<std::string> _state_interfaces;

        std::vector<trajectory_msgs::msg::JointTrajectoryPoint> _targets;
        std::vector<PIDController<double>> _pid;

        // TODO: maybe switch to gazebo time
        int64_t _start_time, _last_update;
        trajectory_msgs::msg::JointTrajectoryPoint _last_point;
        
        int _resolution;
        double _p,_i,_d;
        double _toll;

    };

}


#endif //TRAJECTORY_EFFORT_HPP
