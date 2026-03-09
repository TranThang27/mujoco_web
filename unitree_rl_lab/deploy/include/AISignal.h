// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <atomic>
#include <thread>
#include <zmq.hpp>
#include <string>
#include <spdlog/spdlog.h>

// Dance command enum (0 = none)
enum class DanceCmd { NONE = 0, DANCE_102 = 1, GANGNAM_STYLE = 2 };

class AISignal
{
public:
    static AISignal& getInstance()
    {
        static AISignal instance;
        return instance;
    }

    void start(const std::string& address = "tcp://localhost:5555")
    {
        if (running_.load()) return;

        running_ = true;
        receiver_thread_ = std::thread([this, address]() {
            try {
                zmq::context_t context(1);
                zmq::socket_t subscriber(context, ZMQ_SUB);

                subscriber.set(zmq::sockopt::rcvtimeo, 100);
                subscriber.connect(address);
                subscriber.set(zmq::sockopt::subscribe, "dance");

                spdlog::info("AISignal: Connected to {}", address);

                while (running_.load()) {
                    zmq::message_t msg;
                    auto res = subscriber.recv(msg, zmq::recv_flags::none);

                    if (!running_.load()) break;

                    if (res.has_value()) {
                        std::string received = std::string(static_cast<char*>(msg.data()), msg.size());

                        // Dance commands from web via g1_ws_bridge
                        if (received.find("dance ") == 0) {
                            std::string cmd = received.substr(6);
                            if (cmd.find("dance102") != std::string::npos) {
                                dance_cmd_ = static_cast<int>(DanceCmd::DANCE_102);
                                spdlog::info("AISignal: DANCE_102 triggered!");
                            } else if (cmd.find("gangnam") != std::string::npos) {
                                dance_cmd_ = static_cast<int>(DanceCmd::GANGNAM_STYLE);
                                spdlog::info("AISignal: GANGNAM_STYLE triggered!");
                            }
                        }
                    }
                }
                subscriber.close();
            } catch (const zmq::error_t& e) {
                if (running_.load()) spdlog::error("AISignal ZMQ error: {}", e.what());
            }
        });

        spdlog::info("AISignal: Receiver thread started");
    }

    void stop()
    {
        running_ = false;
        if (receiver_thread_.joinable()) {
            receiver_thread_.join();
        }
        spdlog::warn("AISignal: Port 5555 CLOSED.");
    }

    // Dance command API
    DanceCmd getDanceCmd() const { return static_cast<DanceCmd>(dance_cmd_.load()); }
    void clearDanceCmd() { dance_cmd_ = static_cast<int>(DanceCmd::NONE); }
    bool isDance102Triggered() const { return getDanceCmd() == DanceCmd::DANCE_102; }
    bool isGangnamTriggered()  const { return getDanceCmd() == DanceCmd::GANGNAM_STYLE; }

private:
    AISignal() : running_(false), dance_cmd_(static_cast<int>(DanceCmd::NONE)) {}
    ~AISignal() { stop(); }

    AISignal(const AISignal&) = delete;
    AISignal& operator=(const AISignal&) = delete;

    std::atomic<bool> running_;
    std::atomic<int>  dance_cmd_;
    std::thread       receiver_thread_;
};
