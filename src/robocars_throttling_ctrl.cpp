/**
 * @file robocars_throttling_ctrl.cpp
 * @brief drive ESC speed accoridingly to current mode and orders received.
 * 
 * Copyright (c) 2020 Benoit TRINITE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * Topic subscribed : 
 *  - /radio_channels : current throttling value from radio
 *  - /robocars_brain_state : current car state
 *  - /robocars_actuator_ctrl_mode : nominal/calibration mode
 *  - /autopilot/throttling : current throttling decision from autopilot
 * Topic published :
 *  - /robocars_throttling_ctrl/output : value sent actuator 
 * 
 * Parameters :
 *  - command_input_min : expected lowest value from radio channel : used to map value to -1
 *  - command_input_max : expected highest value from radio channel : used to map value to 1
 *  - command_output_min : expected lowest value for actuators : usually : 1000
 *  - command_output_max : expected highest value for actuators : usually : 2000
 *  - use_brake : use brake cycle to actively slow down car, ESC must support it
 *  - brake_cycle_ms : duration for brake cycle
 *  - loop_hz : main loop refresh freq.
 */


#include <tinyfsm.hpp>
#include <ros/ros.h>
#include <stdio.h>
#include <algorithm> 
#include <cmath>

#include <robocars_msgs/robocars_actuator_output.h>
#include <robocars_msgs/robocars_actuator_ctrl_mode.h>
#include <robocars_msgs/robocars_radio_channels.h>
#include <robocars_msgs/robocars_brain_state.h>
#include <robocars_msgs/robocars_autopilot_output.h>

#include <robocars_throttling_ctrl.hpp>

RosInterface * ri;

static int command_input_min;
static int command_input_max;
static int command_output_min;
static int command_output_max;
static int brake_cycle_ms;
static int use_brake;
bool discrete_throttling = false;
bool reverse = false;
static int loop_hz;
static int discrete_throttling_thres1;
static int discrete_throttling_thres2;
static int discrete_throttling_level1;
static int discrete_throttling_level2;
u_int32_t thres1level;
u_int32_t thres2level;
u_int32_t out1Level;
u_int32_t out2Level;

class onRunningMode;
class onIdle;
class onManualDriving;
class onAutonomousDriving;
class onQualibtrateMode;
class onStopDriving;

class onRunningMode
: public RobocarsStateMachine
{
    public:
        onRunningMode() : RobocarsStateMachine("onRunningMode"),__tick_count(0) {};
        onRunningMode(const char * subStateName) : RobocarsStateMachine(subStateName),__tick_count(0) {};


    protected:

        uint32_t __tick_count;
        
        void entry(void) override {
            RobocarsStateMachine::entry();
        };

        void react(ManualDrivingEvent const & e) override { 
            RobocarsStateMachine::react(e);
        };

        void react( AutonomousDrivingEvent const & e) override { 
            RobocarsStateMachine::react(e);
        };

        void react( IdleStatusEvent const & e) override { 
            RobocarsStateMachine::react(e);
        };

        void react(EnterQualibrateModeEvent const & e) override { 
            RobocarsStateMachine::react(e);
        };

        void react(TickEvent const & e) override {
            __tick_count++;
            if ((__tick_count%loop_hz)==0) {
                //Update param each second
                ri->updateParam(); 
            }
        };

};

class onQualibtrateMode
: public RobocarsStateMachine
{
    public:
        onQualibtrateMode() : RobocarsStateMachine("onQualibtrateMode") {};

    private:

        void entry(void) override {
            RobocarsStateMachine::entry();
            ri->initQualibration();
        };

        void react (LeaveQualibrateModeEvent const & e) override {
            RobocarsStateMachine::react(e);
            transit<onIdle>();
        }

        void react (RadioChannelEvent const & e) override {
            ri->qualibrate(e.radio_channel_value); 
        };

        void react (TickEvent const & e) override {
            RobocarsStateMachine::react(e);
        };

};

class onIdle
: public onRunningMode
{
    public:
        onIdle() : onRunningMode("onArm") {};

    private:

        void entry(void) override {
            onRunningMode::entry();
        };
  
        void react(ManualDrivingEvent const & e) override { 
            onRunningMode::react(e);
            transit<onManualDriving>();
        };

        void react(TickEvent const & e) override {
            ri->maintainIdleActuator(); 
            onRunningMode::react(e);
        };

};

class onManualDriving
: public onRunningMode
{
    public:
        onManualDriving() : onRunningMode("onManualDriving") {};

    private:

        void entry(void) override {
            onRunningMode::entry();
            if (discrete_throttling) {
                ROS_INFO("Throttling Ctrl: threshold level 1 set to %d", thres1level);
                ROS_INFO("Throttling Ctrl: threshold level 2 set to %d", thres2level);
                ROS_INFO("Throttling Ctrl: Output level 1 set to %d", out1Level);
                ROS_INFO("Throttling Ctrl: Output level 2 set to %d", out2Level);
            }
        };

        void react (AutonomousDrivingEvent const & e) override {
            onRunningMode::react(e);
            transit<onAutonomousDriving>();
        }

        void react(IdleStatusEvent const & e) override { 
            onRunningMode::react(e);
            transit<onIdle>();
        };

        void react (RadioChannelEvent const & e) override {
            ri->controlActuatorFromRadio(e.radio_channel_value); 
        }

        void react (TickEvent const & e) override {
            onRunningMode::react(e);
        };

};

class onAutonomousDriving
: public onRunningMode
{
    public:
        onAutonomousDriving() : onRunningMode("onAutonomousDriving") {};

    protected:

        virtual void entry(void) { 
            onRunningMode::entry();
        };  

        virtual void react(TickEvent                      const & e) override { 
            onRunningMode::react(e);
        };

        virtual void react(IdleStatusEvent                 const & e) override { 
            onRunningMode::react(e);
            transit<onIdle>();
        };

        void react (AutopilotEvent const & e) override {
            ri->controlActuatorFromAutopilot(e.autopilot_value, e.carId); 
        };

        virtual void react(ManualDrivingEvent                     const & e) override { 
            onRunningMode::react(e);
            if (use_brake == 1) {
                transit<onStopDriving>();
            } else {
                transit<onManualDriving>();
            }
        };

};

class onStopDriving
: public onRunningMode
{
    public:
        onStopDriving() : onRunningMode("onStopDriving") {};

    protected:

        uint32_t __tick_count;

        virtual void entry(void) { 
            onRunningMode::entry();
            __tick_count=0;
        };  

        virtual void react(IdleStatusEvent                 const & e) override { 
            onRunningMode::react(e);
            transit<onIdle>();
        };

        virtual void react(TickEvent                      const & e) override { 
            onRunningMode::react(e);
            ri->brakeActuator(); 
            __tick_count++;
            if (__tick_count%(brake_cycle_ms/loop_hz)==0) {
                transit<onManualDriving>();
            }
        };

};

FSM_INITIAL_STATE(RobocarsStateMachine, onIdle);


uint32_t discretizeValue(uint32_t out1,uint32_t out2,uint32_t value)
{
  if (value<thres1level) {return 0;}
  if (value>=thres1level && value<thres2level) {return out1;}
  if (value>=thres2level) {return out2;}
  return 0;
}

uint32_t mapRange(uint32_t in1,uint32_t in2,uint32_t out1,uint32_t out2,uint32_t value)
{
  if (value<in1) {value=in1;}
  if (value>in2) {value=in2;}
  return out1 + ((value-in1)*(out2-out1))/(in2-in1);
}

_Float32 mapRange(_Float32 in1,_Float32 in2,_Float32 out1,_Float32 out2,_Float32 value)
{
  if (value<in1) {value=in1;}
  if (value>in2) {value=in2;}
  return out1 + ((value-in1)*(out2-out1))/(in2-in1);
}

void RosInterface::initParam() {
    if (!nh.hasParam("command_input_min")) {
        nh.setParam ("command_input_min", 363);       
    }
    if (!nh.hasParam("command_input_max")) {
        nh.setParam ("command_input_max", 1641);       
    }
    if (!nh.hasParam("command_output_min")) {
        nh.setParam ("command_output_min", 1000);       
    }
    if (!nh.hasParam("command_output_max")) {
        nh.setParam ("command_output_max", 2000);       
    }
    if (!nh.hasParam("use_brake")) {
        nh.setParam("use_brake", 0);
    }
    if (!nh.hasParam("brake_cycle_ms")) {
        nh.setParam("brake_cycle_ms", 1000);
    }
    if (!nh.hasParam("discrete_throttling")) {
        nh.setParam ("discrete_throttling", false);       
    }
    if (!nh.hasParam("discrete_throttling_thres1")) {
        nh.setParam ("discrete_throttling_thres1", 20);       
    }
    if (!nh.hasParam("discrete_throttling_thres2")) {
        nh.setParam ("discrete_throttling_thres2", 70);       
    }
    if (!nh.hasParam("discrete_throttling_level1")) {
        nh.setParam ("discrete_throttling_level1", 40);       
    }
    if (!nh.hasParam("discrete_throttling_level2")) {
        nh.setParam ("discrete_throttling_level2", 100);       
    }
    if (!nh.hasParam("loop_hz")) {
        nh.setParam ("loop_hz", 30);       
    }
    if (!nh.hasParam("reverse")) {
        nh.setParam ("reverse", false);       
    }
}
void RosInterface::updateParam() {
    nh.getParam("command_input_min", command_input_min);
    nh.getParam("command_input_max", command_input_max);
    nh.getParam("command_output_min", command_output_min);
    nh.getParam("command_output_max", command_output_max);
    nh.getParam("use_brake", use_brake);
    nh.getParam("brake_cycle_ms", brake_cycle_ms);
    nh.getParam("discrete_throttling", discrete_throttling);
    nh.getParam("discrete_throttling_thres1", discrete_throttling_thres1);
    nh.getParam("discrete_throttling_thres2", discrete_throttling_thres2);
    nh.getParam("discrete_throttling_level1", discrete_throttling_level1);
    nh.getParam("discrete_throttling_level2", discrete_throttling_level2);
    nh.getParam("loop_hz", loop_hz);
    nh.getParam("reverse", reverse);

    u_int32_t idleThrottlingIn = command_input_min+((command_input_max-command_input_min)/2);
    thres1level = idleThrottlingIn+((command_input_max-idleThrottlingIn)*discrete_throttling_thres1/100);
    thres2level = idleThrottlingIn+((command_input_max-idleThrottlingIn)*discrete_throttling_thres2/100);

    out1Level = idleThrottlingIn+((command_input_max-idleThrottlingIn)*discrete_throttling_level1)/100;
    out2Level = idleThrottlingIn+((command_input_max-idleThrottlingIn)*discrete_throttling_level2)/100;

}

void RosInterface::initPub () {
    act_throttling_pub = nh.advertise<robocars_msgs::robocars_actuator_output>("output", 10);
}

void RosInterface::initSub () {
    channels_sub = nh.subscribe<robocars_msgs::robocars_radio_channels>("/radio_channels", 1, &RosInterface::channels_msg_cb, this);
    state_sub = nh.subscribe<robocars_msgs::robocars_brain_state>("/robocars_brain_state", 1, &RosInterface::state_msg_cb, this);
    mode_sub = nh.subscribe<robocars_msgs::robocars_actuator_ctrl_mode>("/robocars_actuator_ctrl_mode", 1, &RosInterface::mode_msg_cb, this);
    autopilot_sub = nh.subscribe<robocars_msgs::robocars_autopilot_output>("/autopilot/throttling", 1, &RosInterface::autopilot_msg_cb, this);

}

void RosInterface::channels_msg_cb(const robocars_msgs::robocars_radio_channels::ConstPtr& msg){    
    send_event(RadioChannelEvent(msg->channels[2]));
}

void RosInterface::autopilot_msg_cb(const robocars_msgs::robocars_autopilot_output::ConstPtr& msg) {
        unsigned int carId = stoi(msg->header.frame_id);
        send_event(AutopilotEvent(msg->norm, carId));
}

void RosInterface::state_msg_cb(const robocars_msgs::robocars_brain_state::ConstPtr& msg) {
    static u_int32_t last_state = -1;
    if (msg->state != last_state) {
        switch (msg->state) {
            case robocars_msgs::robocars_brain_state::BRAIN_STATE_IDLE:
                send_event(IdleStatusEvent());        
            break;
            case robocars_msgs::robocars_brain_state::BRAIN_STATE_MANUAL_DRIVING:
                send_event(ManualDrivingEvent());        
            break;
            case robocars_msgs::robocars_brain_state::BRAIN_STATE_AUTONOMOUS_DRIVING:
                send_event(AutonomousDrivingEvent());        
            break;
        }
        last_state=msg->state;
    }
    
}

void RosInterface::mode_msg_cb(const robocars_msgs::robocars_actuator_ctrl_mode::ConstPtr& msg) {
    if (msg->mode == robocars_msgs::robocars_actuator_ctrl_mode::ACTUATOR_MODE_QUALIBRATE) {
        send_event(EnterQualibrateModeEvent());        
    } else {
        send_event(LeaveQualibrateModeEvent());        
    }
}


void RosInterface::controlActuatorFromRadio (uint32_t throttling_value) {

    robocars_msgs::robocars_actuator_output throttlingMsg;

    throttlingMsg.header.stamp = ros::Time::now();
    throttlingMsg.header.seq=1;
    throttlingMsg.header.frame_id = "0";
    if (discrete_throttling) {
        throttling_value = discretizeValue(out1Level,out2Level,throttling_value);
    }
    if (reverse==false) {
        throttlingMsg.pwm = std::max((uint32_t)1500,mapRange(command_input_min,command_input_max,command_output_min,command_output_max,throttling_value));
        throttlingMsg.norm = std::fmax((_Float32)0.0,mapRange((_Float32)command_input_min,(_Float32)command_input_max,-1.0,1.0,(_Float32)throttling_value));
    } else {
        throttlingMsg.pwm = std::max((uint32_t)1000,mapRange(command_input_min,command_input_max,command_output_min,command_output_max,throttling_value));
        throttlingMsg.norm = std::fmax((_Float32)-1.0,mapRange((_Float32)command_input_min,(_Float32)command_input_max,-1.0,1.0,(_Float32)throttling_value));
    }

    act_throttling_pub.publish(throttlingMsg);
}

void RosInterface::controlActuatorFromAutopilot (_Float32 throttling_value, __uint32_t carId) {

    robocars_msgs::robocars_actuator_output throttlingMsg;
    char frame_id[100];
    snprintf(frame_id, sizeof(frame_id), "%d", carId);

    throttlingMsg.header.stamp = ros::Time::now();
    throttlingMsg.header.seq=1;
    throttlingMsg.header.frame_id = frame_id;
    throttlingMsg.pwm = std::max((uint32_t)1500,(uint32_t)mapRange(-1.0,1.0,(_Float32)command_output_min,(_Float32)command_output_max,throttling_value));
    throttlingMsg.norm = throttling_value;

    act_throttling_pub.publish(throttlingMsg);
}

void RosInterface::maintainIdleActuator () {

    robocars_msgs::robocars_actuator_output throttlingMsg;

    throttlingMsg.header.stamp = ros::Time::now();
    throttlingMsg.header.seq=1;
    throttlingMsg.header.frame_id = "0";
    throttlingMsg.pwm = 1500;
    throttlingMsg.norm = 0.0;

    act_throttling_pub.publish(throttlingMsg);
}

void RosInterface::brakeActuator () {

    robocars_msgs::robocars_actuator_output throttlingMsg;

    throttlingMsg.header.stamp = ros::Time::now();
    throttlingMsg.header.seq=1;
    throttlingMsg.header.frame_id = "0";
    throttlingMsg.pwm = 1000;
    throttlingMsg.norm = -1.0;

    act_throttling_pub.publish(throttlingMsg);
}

void RosInterface::initQualibration() {
    command_input_min = 1500;
    command_input_max = 1500;
}

void RosInterface::qualibrate (uint32_t throttling_value) {
    if (throttling_value < command_input_min) {
        nh.setParam ("command_input_min", (int)throttling_value);
        command_input_min = throttling_value;
    }
    if (throttling_value > command_input_max) {
        nh.setParam ("command_input_max", (int)throttling_value);      
        command_input_max = throttling_value;   
    }
}

int main(int argc, char **argv)
{
    int loopCnt=0;
    ros::init(argc, argv, "robocars_throttling_ctrl_fsm");

    ri = new RosInterface;

    ri->initPub();
    fsm_list::start();
    ri->initSub();
    ROS_INFO("Throttling Ctrl: Starting");

    // wait for FCU connection
    ros::Rate rate(loop_hz);
    while(ros::ok()){
        ros::spinOnce();
        send_event (TickEvent());
        rate.sleep();
    }
}

