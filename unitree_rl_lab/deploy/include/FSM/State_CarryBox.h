// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSM/State_RLBase.h"
#include "LinearInterpolator.h"

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
namespace CarryBoxJoint {
    constexpr int LeftShoulderPitch = 15;
    constexpr int LeftShoulderRoll = 16;
    constexpr int LeftShoulderYaw = 17;
    constexpr int LeftElbow = 18;
    constexpr int LeftWristRoll = 19;
    constexpr int LeftWristPitch = 20;
    constexpr int LeftWristYaw = 21;
    constexpr int RightShoulderPitch = 22;
    constexpr int RightShoulderRoll = 23;
    constexpr int RightShoulderYaw = 24;
    constexpr int RightElbow = 25;
    constexpr int RightWristRoll = 26;
    constexpr int RightWristPitch = 27;
    constexpr int RightWristYaw = 28;
}

/**
 * State_CarryBox: Kế thừa từ State_RLBase
 * - Policy vẫn chạy để điều khiển chân giữ cân bằng
 * - Phase 1: Đi thẳng 5 giây
 * - Phase 2: Mở hai tay ra (spread arms)
 * - Phase 3: Giơ CẢ HAI tay lên (như đang bê thùng)
 * - Phase 4: Quay đầu 180 độ
 * - Phase 5: Đi thẳng về
 */
class State_CarryBox : public State_RLBase
{
public:
    enum class Phase { WALK_FIRST, OPEN_ARMS, RAISE_ARMS, TURN_AROUND, WALK_BACK, FINISHED };
    
    // Static velocity for observation to read (inline for C++17)
    inline static std::atomic<float> target_vel_x{0.0f};
    inline static std::atomic<float> target_vel_yaw{0.0f};
    
    State_CarryBox(int state, std::string state_string = "CarryBox") 
    : State_RLBase(state, state_string)
    {
        auto cfg = param::config["FSM"]["CarryBox"];
        if(cfg["walk_first_duration"].IsDefined()) {
            walk_first_duration_ = cfg["walk_first_duration"].as<float>();
        }
        if(cfg["open_duration"].IsDefined()) {
            open_duration_ = cfg["open_duration"].as<float>();
        }
        if(cfg["raise_duration"].IsDefined()) {
            raise_duration_ = cfg["raise_duration"].as<float>();
        }
        if(cfg["turn_duration"].IsDefined()) {
            turn_duration_ = cfg["turn_duration"].as<float>();
        }
        if(cfg["walk_back_duration"].IsDefined()) {
            walk_back_duration_ = cfg["walk_back_duration"].as<float>();
        }
    }

    void enter()
    {
        State_RLBase::enter();
        
        phase_ = Phase::WALK_FIRST;
        target_vel_x = 0.3f;  // Start walking immediately
        target_vel_yaw = 0.0f;
        
        // Lưu vị trí ban đầu của tay
        for(int i = CarryBoxJoint::LeftShoulderPitch; i <= CarryBoxJoint::RightWristYaw; ++i) {
            q0_arm_[i - CarryBoxJoint::LeftShoulderPitch] = lowstate->msg_.motor_state()[i].q();
        }

        t0_arm_ = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3;
    }

    void run()
    {
        State_RLBase::run();
        
        float t = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3 - t0_arm_;
        
        std::array<float, 14> q_target;
        float ratio;
        
        if (phase_ == Phase::WALK_FIRST) {
            // Phase 1: Đi thẳng 5 giây đầu
            target_vel_x = 0.3f;
            target_vel_yaw = 0.0f;
            ratio = 1.0f;  // Giữ tay ở vị trí mặc định
            
            // Giữ tay ở vị trí ban đầu
            for(int i = 0; i < 14; ++i) {
                q_target[i] = q0_arm_[i];
            }
            
            if (t >= walk_first_duration_) {
                phase_ = Phase::OPEN_ARMS;
                target_vel_x = 0.0f;
                saveArmPositions();
                t0_arm_ = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3;
            }
        }
        else if (phase_ == Phase::OPEN_ARMS) {
            target_vel_x = 0.0f;
            target_vel_yaw = 0.0f;
            ratio = std::clamp(t / open_duration_, 0.0f, 1.0f);
            
            // Target: Mở hai tay ra ngang (spread arms)
            q_target = {
                // Left arm (mở ra ngang)
                0.0f,         // LeftShoulderPitch (ngang)
                0.8f,         // LeftShoulderRoll (mở ra ngoài)
                q0_arm_[2],   // LeftShoulderYaw
                0.3f,         // LeftElbow (hơi gập)
                q0_arm_[4],   // LeftWristRoll
                q0_arm_[5],   // LeftWristPitch
                q0_arm_[6],   // LeftWristYaw
                
                // Right arm (mở ra ngang)
                0.0f,         // RightShoulderPitch (ngang)
                -0.8f,        // RightShoulderRoll (mở ra ngoài)
                q0_arm_[9],   // RightShoulderYaw
                0.3f,         // RightElbow (hơi gập)
                q0_arm_[11],  // RightWristRoll
                q0_arm_[12],  // RightWristPitch
                q0_arm_[13]   // RightWristYaw
            };
            
            // Chuyển sang phase tiếp theo khi hoàn thành
            if (ratio >= 1.0f) {
                phase_ = Phase::RAISE_ARMS;
                saveArmPositions();
                t0_arm_ = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3;
            }
        }
        else if (phase_ == Phase::RAISE_ARMS) {
            target_vel_x = 0.0f;
            target_vel_yaw = 0.0f;
            ratio = std::clamp(t / raise_duration_, 0.0f, 1.0f);
            
            // Target: Giơ CẢ HAI tay lên
            q_target = {
                // Left arm (giơ lên)
                -0.5f,        // LeftShoulderPitch (raise up)
                0.15f,        // LeftShoulderRoll (slightly out)
                q0_arm_[2],   // LeftShoulderYaw
                0.7f,         // LeftElbow
                q0_arm_[4],   // LeftWristRoll
                q0_arm_[5],   // LeftWristPitch
                q0_arm_[6],   // LeftWristYaw
                
                // Right arm (giơ lên)
                -0.5f,        // RightShoulderPitch (raise up)
                -0.15f,       // RightShoulderRoll (slightly out)
                q0_arm_[9],   // RightShoulderYaw
                0.7f,         // RightElbow
                q0_arm_[11],  // RightWristRoll
                q0_arm_[12],  // RightWristPitch
                q0_arm_[13]   // RightWristYaw
            };
            
            if (ratio >= 1.0f) {
                phase_ = Phase::TURN_AROUND;
                saveArmPositions();
                t0_arm_ = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3;
            }
        }
        else if (phase_ == Phase::TURN_AROUND) {
            // Quay đầu 180 độ
            target_vel_x = 0.2f;
            target_vel_yaw = 0.6f;  // Quay với tốc độ 0.8 rad/s
            ratio = 1.0f;  // Giữ nguyên tay
            
            // Giữ tay ở vị trí hiện tại
            q_target = {
                -0.5f, 0.15f, q0_arm_[2], 0.7f, q0_arm_[4], q0_arm_[5], q0_arm_[6],
                -0.5f, -0.15f, q0_arm_[9], 0.7f, q0_arm_[11], q0_arm_[12], q0_arm_[13]
            };
            
            if (t >= turn_duration_) {
                phase_ = Phase::WALK_BACK;
                t0_arm_ = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3;
            }
        }
        else if (phase_ == Phase::WALK_BACK) {
            // Đi thẳng về (sau khi quay đầu)
            target_vel_x = 0.3f;
            target_vel_yaw = 0.0f;
            ratio = 1.0f;
            
            // Giữ tay ở vị trí hiện tại
            q_target = {
                -0.5f, 0.15f, q0_arm_[2], 0.7f, q0_arm_[4], q0_arm_[5], q0_arm_[6],
                -0.5f, -0.15f, q0_arm_[9], 0.7f, q0_arm_[11], q0_arm_[12], q0_arm_[13]
            };
            
            if (t >= walk_back_duration_) {
                phase_ = Phase::FINISHED;
                target_vel_x = 0.0f;
                target_vel_yaw = 0.0f;
            }
        }
        else { // FINISHED
            target_vel_x = 0.0f;
            target_vel_yaw = 0.0f;
            ratio = 1.0f;
            q_target = {
                -0.5f, 0.15f, q0_arm_[2], 0.7f, q0_arm_[4], q0_arm_[5], q0_arm_[6],
                -0.5f, -0.15f, q0_arm_[9], 0.7f, q0_arm_[11], q0_arm_[12], q0_arm_[13]
            };
        }

        // Interpolate và set arm positions
        for(int i = CarryBoxJoint::LeftShoulderPitch; i <= CarryBoxJoint::RightWristYaw; ++i)
        {
            int idx = i - CarryBoxJoint::LeftShoulderPitch;
            float q_des = q0_arm_[idx] + (q_target[idx] - q0_arm_[idx]) * ratio;
            lowcmd->msg_.motor_cmd()[i].q() = q_des;
            lowcmd->msg_.motor_cmd()[i].kp() = 40;
            lowcmd->msg_.motor_cmd()[i].kd() = 1;
        }
    }

    void exit()
    {
        target_vel_x = 0.0f;
        target_vel_yaw = 0.0f;
        State_RLBase::exit();
    }

private:
    void saveArmPositions() {
        for(int i = CarryBoxJoint::LeftShoulderPitch; i <= CarryBoxJoint::RightWristYaw; ++i) {
            q0_arm_[i - CarryBoxJoint::LeftShoulderPitch] = lowstate->msg_.motor_state()[i].q();
        }
    }
    
    Phase phase_ = Phase::WALK_FIRST;
    double t0_arm_;
    float walk_first_duration_ = 5.0f;  // Phase 1: Đi thẳng 5s đầu
    float open_duration_ = 2.0f;         // Phase 2: Mở tay ra
    float raise_duration_ = 2.0f;        // Phase 3: Giơ tay lên
    float turn_duration_ = 5.0;         // Phase 4: Quay đầu
    float walk_back_duration_ = 7.0f;    // Phase 5: Đi thẳng về
    std::array<float, 14> q0_arm_;
};

REGISTER_FSM(State_CarryBox)
