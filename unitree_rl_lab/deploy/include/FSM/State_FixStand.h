// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSMState.h"
#include "LinearInterpolator.h"
#include "AISignal.h"

class State_FixStand : public FSMState
{
public:
    State_FixStand(int state, std::string state_string = "FixStand") 
    : FSMState(state, state_string) 
    {
        ts_ = param::config["FSM"]["FixStand"]["ts"].as<std::vector<float>>();
        qs_ = param::config["FSM"]["FixStand"]["qs"].as<std::vector<std::vector<float>>>();
        assert(ts_.size() == qs_.size());

        // Auto transition to Velocity after delay
        auto auto_velocity_delay = param::config["FSM"]["FixStand"]["auto_velocity_delay"];
        if(auto_velocity_delay.IsDefined())
        {
            auto_velocity_delay_sec_ = auto_velocity_delay.as<float>();
        }
        
        if(auto_velocity_delay_sec_ > 0 && FSMStringMap.right.count("Velocity"))
        {
            int velocity_id = FSMStringMap.right.at("Velocity");
            registered_checks.emplace_back(
                std::make_pair(
                    [this]() -> bool {
                        auto now = std::chrono::steady_clock::now();
                        auto elapsed = std::chrono::duration<float>(now - enter_time_).count();
                        return elapsed >= auto_velocity_delay_sec_;
                    },
                    velocity_id
                )
            );
        }

        // Dance102 trigger from web / ZMQ — removed: dance should only trigger from Velocity state
    }

    void enter()
    {
        enter_time_ = std::chrono::steady_clock::now();
        // set gain
        static auto kp = param::config["FSM"]["FixStand"]["kp"].as<std::vector<float>>();
        static auto kd = param::config["FSM"]["FixStand"]["kd"].as<std::vector<float>>();
        for(int i(0); i < kp.size(); ++i)
        {
            auto & motor = lowcmd->msg_.motor_cmd()[i];
            motor.kp() = kp[i];
            motor.kd() = kd[i];
            motor.dq() = motor.tau() = 0;
        }


        // set initial position from actual robot state (not command)
        std::vector<float> q0;
        for(int i(0); i < kp.size(); ++i) {
            q0.push_back(lowstate->msg_.motor_state()[i].q());
        }
        qs_[0] = q0;
        t0_ = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3;
    }

    void run()
    {
        float t = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3 - t0_;
        auto q = linear_interpolate(t, ts_, qs_);
        
        for(int i(0); i < q.size(); ++i) {
            lowcmd->msg_.motor_cmd()[i].q() = q[i];
        }
    }

private:
    double t0_;
    std::vector<float> ts_;
    std::vector<std::vector<float>> qs_;
    std::chrono::steady_clock::time_point enter_time_;
    float auto_velocity_delay_sec_ = 0.0f;  // Default disabled, set in config
};

REGISTER_FSM(State_FixStand)