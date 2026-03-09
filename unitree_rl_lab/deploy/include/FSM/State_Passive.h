// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSMState.h"
#include <chrono>

class State_Passive : public FSMState
{
public:
    State_Passive(int state, std::string state_string = "Passive") 
    : FSMState(state, state_string) 
    {
        auto motor_mode = param::config["FSM"]["Passive"]["mode"];
        if(motor_mode.IsDefined())
        {
            auto values = motor_mode.as<std::vector<int>>();
            for(int i(0); i<values.size(); ++i)
            {
                lowcmd->msg_.motor_cmd()[i].mode() = values[i];
            }
        }

        // Auto transition to FixStand after 3 seconds
        auto auto_stand_delay = param::config["FSM"]["Passive"]["auto_stand_delay"];
        if(auto_stand_delay.IsDefined())
        {
            auto_stand_delay_sec_ = auto_stand_delay.as<float>();
        }
        
        if(auto_stand_delay_sec_ > 0 && FSMStringMap.right.count("FixStand"))
        {
            int fixstand_id = FSMStringMap.right.at("FixStand");
            registered_checks.emplace_back(
                std::make_pair(
                    [this]() -> bool {
                        auto now = std::chrono::steady_clock::now();
                        auto elapsed = std::chrono::duration<float>(now - enter_time_).count();
                        return elapsed >= auto_stand_delay_sec_;
                    },
                    fixstand_id
                )
            );
        }
    } 

    void enter()
    {
        enter_time_ = std::chrono::steady_clock::now();
        // set gain
        static auto kd = param::config["FSM"]["Passive"]["kd"].as<std::vector<float>>();
        for(int i(0); i < kd.size(); ++i)
        {
            auto & motor = lowcmd->msg_.motor_cmd()[i];
            motor.kp() = 0;
            motor.kd() = kd[i];
            motor.dq() = 0;
            motor.tau() = 0;
        }
    }

    void run()
    {
        for(int i(0); i < lowcmd->msg_.motor_cmd().size(); ++i)
        {
            lowcmd->msg_.motor_cmd()[i].q() = lowstate->msg_.motor_state()[i].q();
        }
    }

private:
    std::chrono::steady_clock::time_point enter_time_;
    float auto_stand_delay_sec_ = 3.0f;  // Default 3 seconds
};

REGISTER_FSM(State_Passive)
