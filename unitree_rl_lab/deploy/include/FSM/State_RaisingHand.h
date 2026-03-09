// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSM/State_RLBase.h"
#include "LinearInterpolator.h"
#include "AISignal.h"
#include <spdlog/spdlog.h>

/**
 * G1 29dof Joint Index:
 * Legs (0-11):
 *   Left: 0-5 (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll)
 *   Right: 6-11
 * Waist (12-14): yaw, pitch, roll
 * Left Arm (15-21): shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw
 * Right Arm (22-28): shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw
 */

// Arm joint indices for G1 29dof
enum G1ArmJointIndex {
    LeftShoulderPitch = 15,
    LeftShoulderRoll = 16,
    LeftShoulderYaw = 17,
    LeftElbow = 18,
    LeftWristRoll = 19,
    LeftWristPitch = 20,
    LeftWristYaw = 21,
    RightShoulderPitch = 22,
    RightShoulderRoll = 23,
    RightShoulderYaw = 24,
    RightElbow = 25,
    RightWristRoll = 26,
    RightWristPitch = 27,
    RightWristYaw = 28
};

// Phase enum for RaisingHand state machine
enum class RaisingHandPhase {
    RAISING,      // Đang giơ tay lên
    HOLDING,      // Giữ tay trên cao
    LOWERING,     // Hạ tay xuống
    TURNING,  
    MOVING_FORWARD, // Di chuyển về phía trước
    DONE          // Hoàn thành, sẵn sàng chuyển state
};


class State_RaisingHand : public State_RLBase
{
public:
    State_RaisingHand(int state, std::string state_string = "RaisingHand") 
    : State_RLBase(state, state_string)
    {
        auto cfg = param::config["FSM"]["RaisingHand"];
        if(cfg["raise_duration"].IsDefined()) {
            raise_duration_ = cfg["raise_duration"].as<float>();
        }
        if(cfg["hold_duration"].IsDefined()) {
            hold_duration_ = cfg["hold_duration"].as<float>();
        }
        if(cfg["lower_duration"].IsDefined()) {
            lower_duration_ = cfg["lower_duration"].as<float>();
        }
        if(cfg["turn_duration"].IsDefined()) {
            turn_duration_ = cfg["turn_duration"].as<float>();
        }
        if(cfg["turn_velocity"].IsDefined()) {
            turn_velocity_ = cfg["turn_velocity"].as<float>();
        }
        
        // Auto transition to Velocity after lowering hand
        if(FSMStringMap.right.count("Velocity"))
        {
            int velocity_id = FSMStringMap.right.at("Velocity");
            registered_checks.emplace_back(
                std::make_pair(
                    [this]() -> bool {
                        return phase_ == RaisingHandPhase::DONE;
                    },
                    velocity_id
                )
            );
        }
    }

    void enter()
    {
        // Gọi enter của State_RLBase để khởi động policy cho chân
        State_RLBase::enter();
        AISignal::getInstance().stop();
        // Reset phase
        AISignal::getInstance().setVelocity(0.0f, 0.0f, 0.0f);
        phase_ = RaisingHandPhase::RAISING;
        
        // Store initial arm positions
        for(int i = LeftShoulderPitch; i <= RightWristYaw; ++i) {
            q0_arm_[i - LeftShoulderPitch] = lowstate->msg_.motor_state()[i].q();
        }
        
        // Calculate raised arm target positions
        q_raised_ = {
            // Left arm (giữ nguyên)
            q0_arm_[0],   // LeftShoulderPitch
            q0_arm_[1],   // LeftShoulderRoll
            q0_arm_[2],   // LeftShoulderYaw
            q0_arm_[3],   // LeftElbow
            q0_arm_[4],   // LeftWristRoll
            q0_arm_[5],   // LeftWristPitch
            q0_arm_[6],   // LeftWristYaw
            
            // Right arm (giơ lên)
            -0.65f,        // RightShoulderPitch (raise up)
            -0.15f,       // RightShoulderRoll (slightly out)
            q0_arm_[9],   // RightShoulderYaw
            0.7f,         // RightElbow
            q0_arm_[11],  // RightWristRoll
            q0_arm_[12],  // RightWristPitch
            q0_arm_[13]   // RightWristYaw
        };

        t0_arm_ = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3;
        spdlog::warn("RaisingHand: Thread ZMQ da dung. Robot dang thuc hien hanh dong...");
    }

  void run()
{
    State_RLBase::run();
    
    float t = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3 - t0_arm_;
    std::array<float, 14> q_current;
    
    // Tham số làm mượt
    const float ramp_time = 0.8f; // Thời gian tăng/giảm tốc (giây)
    const float target_v = 0.3f;  // Vận tốc đi thẳng tối đa
    const float move_duration = 4.0f; // Tổng thời gian đi thẳng

    switch(phase_)
    {
        case RaisingHandPhase::RAISING:
        case RaisingHandPhase::HOLDING:
        case RaisingHandPhase::LOWERING:
        {
            // Ép đứng yên tuyệt đối trong khi làm việc với tay
            AISignal::getInstance().setVelocity(0.0f, 0.0f, 0.0f);

            if (phase_ == RaisingHandPhase::RAISING) {
                float ratio = std::clamp(t / raise_duration_, 0.0f, 1.0f);
                for(int i = 0; i < 14; ++i) q_current[i] = q0_arm_[i] + (q_raised_[i] - q0_arm_[i]) * ratio;
                if(t >= raise_duration_) { phase_ = RaisingHandPhase::HOLDING; t0_arm_ = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3; }
            } 
            else if (phase_ == RaisingHandPhase::HOLDING) {
                q_current = q_raised_;
                if(t >= hold_duration_) { phase_ = RaisingHandPhase::LOWERING; t0_arm_ = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3; 
                for(int i = 15; i <= 28; ++i) q_lower_start_[i-15] = lowstate->msg_.motor_state()[i].q(); }
            }
            else { // LOWERING
                float ratio = std::clamp(t / lower_duration_, 0.0f, 1.0f);
                for(int i = 0; i < 14; ++i) q_current[i] = q_lower_start_[i] + (q0_arm_[i] - q_lower_start_[i]) * ratio;
                if(t >= lower_duration_) { phase_ = RaisingHandPhase::TURNING; t0_arm_ = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3; }
            }
            break;
        }
        
       case RaisingHandPhase::TURNING:
        {
            // Thiết lập vận tốc quay cố định 0.5 rad/s
            // Lưu ý: Đối với quay tại chỗ, giá trị x và y phải là 0.0
            float fixed_turn_vel_y = 0.0f;
            float fixed_turn_vel_Z = 0.5f; 
            AISignal::getInstance().setVelocity(0.01f, fixed_turn_vel_y, fixed_turn_vel_Z);
            
            q_current = q0_arm_; // Giữ tay ở vị trí mặc định

            // In log cảnh báo mỗi giây một lần để tránh tràn màn hình (spam)

            if(t >= turn_duration_) {
                phase_ = RaisingHandPhase::MOVING_FORWARD;
                // RESET mốc thời gian cực kỳ quan trọng để phase sau không bị bỏ qua
                t0_arm_ = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3;
                
                // Dừng quay trước khi bắt đầu đi thẳng
                AISignal::getInstance().setVelocity(0.0f, 0.0f, 0.0f);
                spdlog::info("RaisingHand: Quay xong, chuyen sang di thang.");
            }
            break;
        }

        case RaisingHandPhase::MOVING_FORWARD:
        {
            // HÌNH THANG VẬN TỐC TIẾN (Trapezoidal Profile)
            float current_v = target_v;
            if (t < ramp_time) {
                current_v = target_v * (t / ramp_time); // Ramp up
            } else if (t > move_duration - ramp_time) {
                current_v = target_v * (move_duration - t) / ramp_time; // Ramp down
            }

            AISignal::getInstance().setVelocity(std::max(0.0f, current_v), 0.0f, 0.0f);
            q_current = q0_arm_;
            spdlog::warn("Di thang sau khi quay dau");

            if(t >= move_duration) { 
                phase_ = RaisingHandPhase::DONE;
                AISignal::getInstance().setVelocity(0.0f, 0.0f, 0.0f);
                
            }
            break;
        }
        
        case RaisingHandPhase::DONE:
        {
            AISignal::getInstance().setVelocity(0.0f, 0.0f, 0.0f);
            q_current = q0_arm_;
            break;
        }
    }
    
    // Gửi lệnh khớp tay
    for(int i = LeftShoulderPitch; i <= RightWristYaw; ++i) {
        lowcmd->msg_.motor_cmd()[i].q() = q_current[i - LeftShoulderPitch];
        lowcmd->msg_.motor_cmd()[i].kp() = 40;
        lowcmd->msg_.motor_cmd()[i].kd() = 1;
    }
}
    void exit()
    {
        
        // Gọi exit của State_RLBase để dừng policy thread
        State_RLBase::exit();
        spdlog::info("RaisingHand: Exited");
    }

private:
    double t0_arm_;
    float raise_duration_ = 2.0f;   // Thời gian giơ tay lên
    float hold_duration_ = 5.0f;    // Thời gian giữ tay trên cao
    float lower_duration_ = 2.0f;   // Thời gian hạ tay xuống
    float turn_duration_ = 2.0f;    // Thời gian quay đầu
    float turn_velocity_ = 0.8f;    // Tốc độ quay (rad/s)
    
    RaisingHandPhase phase_ = RaisingHandPhase::RAISING;
    
    std::array<float, 14> q0_arm_;        // Vị trí ban đầu của tay
    std::array<float, 14> q_raised_;      // Vị trí tay giơ lên
    std::array<float, 14> q_lower_start_; // Vị trí bắt đầu hạ tay
};

REGISTER_FSM(State_RaisingHand)
