/**
 * @file robocars_throttling_ctrl.hpp
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
 */
#include <tinyfsm.hpp>
#include <ros/ros.h>
#include <stdio.h>

struct BaseEvent : tinyfsm::Event
{
    public:
        BaseEvent(const char * evtName) : _evtName(evtName) {};
        const char * getEvtName() const { return _evtName; };
    private:
        const char *  _evtName;
};

struct TickEvent                    : BaseEvent { public: TickEvent() : BaseEvent("TickEvent") {}; };
struct IdleStatusEvent              : BaseEvent { public: IdleStatusEvent() : BaseEvent("IdleStatusEvent") {}; };
struct ManualDrivingEvent           : BaseEvent { public: ManualDrivingEvent() : BaseEvent("ManualDrivingEvent") {}; };
struct AutonomousDrivingEvent       : BaseEvent { public: AutonomousDrivingEvent() : BaseEvent("AutonomousDrivingEvent") {}; };
struct EnterQualibrateModeEvent     : BaseEvent { public: EnterQualibrateModeEvent() : BaseEvent("EnterQualibrateModeEvent") {}; };
struct LeaveQualibrateModeEvent     : BaseEvent { public: LeaveQualibrateModeEvent() : BaseEvent("LeaveQualibrateModeEvent") {}; };
struct RadioChannelEvent            : BaseEvent { public: 
    RadioChannelEvent(const uint32_t value) : radio_channel_value(value), BaseEvent("RadioChannelEvent") {};
    uint32_t radio_channel_value; 
    };
struct AutopilotEvent            : BaseEvent { public: 
    AutopilotEvent(const _Float32 value, const __uint32_t carId) : autopilot_value(value), carId(carId), BaseEvent("AutopilotEvent") {};
    _Float32 autopilot_value; 
    __uint32_t carId;
    };

class RobocarsStateMachine
: public tinyfsm::Fsm<RobocarsStateMachine>
{
    public:
        RobocarsStateMachine(const char * stateName) : _stateName(stateName), tinyfsm::Fsm<RobocarsStateMachine>::Fsm() { 
            ROS_INFO("Throttling Ctrl StateMachine: State created: %s", _stateName);      
        };
        const char * getStateName() const { return _stateName; };

    public:
        /* default reaction for unhandled events */
        void react(BaseEvent const & ev) { 
            ROS_INFO("state %s: unexpected event %s reveived", getStateName(), ev.getEvtName());      
        };

        virtual void react(TickEvent                      const & e) { /*logEvent(e);*/ };
        virtual void react(IdleStatusEvent                const & e) { logEvent(e); };
        virtual void react(ManualDrivingEvent             const & e) { logEvent(e); };
        virtual void react(AutonomousDrivingEvent         const & e) { logEvent(e); };
        virtual void react(EnterQualibrateModeEvent       const & e) { logEvent(e); };
        virtual void react(LeaveQualibrateModeEvent       const & e) { logEvent(e); };
        virtual void react(RadioChannelEvent              const & e) {  };
        virtual void react(AutopilotEvent                 const & e) {  };

        virtual void entry(void) { 
            ROS_INFO("Throttling Ctrl : State %s: entering", getStateName()); 
        };  
        void         exit(void)  { };  /* no exit actions */

    private:
        const char *  _stateName ="NoName";
        void logEvent(BaseEvent const & e) {
            ROS_INFO("State %s: event %s", getStateName(), e.getEvtName());
        }
};

typedef tinyfsm::FsmList<RobocarsStateMachine> fsm_list;

template<typename E>
void send_event(E const & event)
{
  fsm_list::template dispatch<E>(event);
}

class RosInterface
{
    public :
        RosInterface() {
            initParam();
            updateParam();
        };


        void initParam();
        void updateParam();
        void initPub();
        void initSub();

        void maintainIdleActuator();
        void brakeActuator();
        void controlActuatorFromRadio (uint32_t steering_value);
        void controlActuatorFromAutopilot (_Float32 steering_value, __uint32_t carId);
        void initQualibration();
        void qualibrate (uint32_t throttling_value);

    private:

        void channels_msg_cb(const std_msgs::Int16MultiArray::ConstPtr& msg);
        void state_msg_cb(const robocars_msgs::robocars_brain_state::ConstPtr& msg);
        void mode_msg_cb(const robocars_msgs::robocars_actuator_ctrl_mode::ConstPtr& msg);
        void autopilot_msg_cb(const robocars_msgs::robocars_autopilot_output::ConstPtr& msg);

        ros::NodeHandle nh;
        ros::Publisher act_throttling_output_pub;
        ros::Publisher act_throttling_norm_pub;
        ros::Subscriber channels_sub;
        ros::Subscriber state_sub;
        ros::Subscriber mode_sub;
        ros::Subscriber autopilot_sub;

};

