// g1_ws_bridge: Bridges browser (mujoco_wasm) ↔ DDS ↔ g1_ctrl
// Browser runs G1 physics locally, bridge relays LowState/LowCmd via WebSocket.
//
// Flow:
//   Browser (mujoco_wasm physics) --[WS JSON LowState]--> bridge --[DDS]--> g1_ctrl
//   g1_ctrl --[DDS LowCmd]--> bridge --[WS JSON LowCmd]--> Browser (apply ctrl[])

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

#include <zmq.hpp>

#include <iostream>
#include <sstream>
#include <thread>
#include <mutex>
#include <set>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>

using namespace unitree::robot;
namespace beast     = boost::beast;
namespace websocket = beast::websocket;
namespace net       = boost::asio;
using     tcp       = net::ip::tcp;

static constexpr int NUM_MOTOR = 29;
static constexpr unsigned short WS_PORT = 8765;

// ─── ZMQ publisher (bridge → g1_ctrl AISignal) ────────────────────────────────
static zmq::context_t* g_zmq_ctx  = nullptr;
static zmq::socket_t*  g_zmq_pub  = nullptr;

void zmq_publish(const std::string& msg) {
    if (!g_zmq_pub) return;
    try {
        zmq::message_t m(msg.data(), msg.size());
        g_zmq_pub->send(m, zmq::send_flags::dontwait);
    } catch(...) {}
}

// ─── Global DDS state ────────────────────────────────────────────────────────
std::mutex cmd_mtx;
struct MotorCmdCache {
    float tau[NUM_MOTOR]   = {};
    float kp [NUM_MOTOR]   = {};
    float kd [NUM_MOTOR]   = {};
    float q  [NUM_MOTOR]   = {};
    float dq [NUM_MOTOR]   = {};
} g_cmd;

std::shared_ptr<ChannelPublisher<unitree_hg::msg::dds_::LowState_>> lowstate_pub;

// ─── WebSocket Session Set ────────────────────────────────────────────────────
std::mutex sessions_mtx;
std::set<std::shared_ptr<websocket::stream<tcp::socket>>> g_sessions;

// ─── Build JSON from latest LowCmd ───────────────────────────────────────────
std::string build_cmd_json() {
    std::lock_guard<std::mutex> lk(cmd_mtx);
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(5);
    ss << "{\"tau\":[";
    for (int i = 0; i < NUM_MOTOR; ++i) { if (i) ss << ','; ss << g_cmd.tau[i]; }
    ss << "],\"kp\":[";
    for (int i = 0; i < NUM_MOTOR; ++i) { if (i) ss << ','; ss << g_cmd.kp[i]; }
    ss << "],\"kd\":[";
    for (int i = 0; i < NUM_MOTOR; ++i) { if (i) ss << ','; ss << g_cmd.kd[i]; }
    ss << "],\"q_des\":[";
    for (int i = 0; i < NUM_MOTOR; ++i) { if (i) ss << ','; ss << g_cmd.q[i]; }
    ss << "],\"dq_des\":[";
    for (int i = 0; i < NUM_MOTOR; ++i) { if (i) ss << ','; ss << g_cmd.dq[i]; }
    ss << "]}";
    return ss.str();
}

// ─── Parse browser JSON → publish LowState to DDS / forward dance cmd to ZMQ ──
// Expected: {"q":[29],"dq":[29],"tau_est":[29],"imu_quat":[4],"imu_gyro":[3],"imu_acc":[3]}
//       OR: {"cmd":"dance102"} / {"cmd":"gangnam"} / {"cmd":"raisinghand"}
void parse_and_publish(const std::string& json) {
    // --- Dance command shortcut from web UI ---
    auto cmd_pos = json.find("\"cmd\"");
    if (cmd_pos != std::string::npos) {
        if (json.find("dance102") != std::string::npos) {
            zmq_publish("dance dance102");
            std::cout << "[ZMQ] Sent: dance dance102\n";
        } else if (json.find("gangnam") != std::string::npos) {
            zmq_publish("dance gangnam");
            std::cout << "[ZMQ] Sent: dance gangnam\n";
        }
        return;
    }

    if (!lowstate_pub) return;

    unitree_hg::msg::dds_::LowState_ msg;

    // Simple inline JSON number extractor
    auto extract_array = [&](const char* key, float* out, int n) {
        std::string tag = '"' + std::string(key) + '"';
        auto pos = json.find(tag);
        if (pos == std::string::npos) return;
        pos = json.find('[', pos);
        if (pos == std::string::npos) return;
        ++pos;
        for (int i = 0; i < n; ++i) {
            while (pos < json.size() && (json[pos] == ',' || json[pos] == ' ')) ++pos;
            if (pos >= json.size() || json[pos] == ']') break;
            out[i] = std::stof(json.substr(pos), nullptr);
            pos = json.find_first_of(",]", pos);
        }
    };

    float q[35]={}, dq[35]={}, tau_est[35]={};
    float imu_quat[4]={1,0,0,0}, imu_gyro[3]={}, imu_acc[3]={};

    extract_array("q",       q,        NUM_MOTOR);
    extract_array("dq",      dq,       NUM_MOTOR);
    extract_array("tau_est", tau_est,  NUM_MOTOR);
    extract_array("imu_quat", imu_quat, 4);
    extract_array("imu_gyro", imu_gyro, 3);
    extract_array("imu_acc",  imu_acc,  3);

    for (int i = 0; i < NUM_MOTOR; ++i) {
        msg.motor_state()[i].q()       = q[i];
        msg.motor_state()[i].dq()      = dq[i];
        msg.motor_state()[i].tau_est() = tau_est[i];
    }
    msg.imu_state().quaternion()[0] = imu_quat[0];
    msg.imu_state().quaternion()[1] = imu_quat[1];
    msg.imu_state().quaternion()[2] = imu_quat[2];
    msg.imu_state().quaternion()[3] = imu_quat[3];
    msg.imu_state().gyroscope()[0]  = imu_gyro[0];
    msg.imu_state().gyroscope()[1]  = imu_gyro[1];
    msg.imu_state().gyroscope()[2]  = imu_gyro[2];
    msg.imu_state().accelerometer()[0] = imu_acc[0];
    msg.imu_state().accelerometer()[1] = imu_acc[1];
    msg.imu_state().accelerometer()[2] = imu_acc[2];

    lowstate_pub->Write(msg);
}

// ─── Broadcast latest LowCmd to all WS clients ───────────────────────────────
void broadcast_loop() {
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(8)); // ~120Hz
        std::string msg = build_cmd_json();
        std::lock_guard<std::mutex> lk(sessions_mtx);
        for (auto it = g_sessions.begin(); it != g_sessions.end(); ) {
            try {
                (*it)->text(true);
                (*it)->write(net::buffer(msg));
                ++it;
            } catch (...) {
                it = g_sessions.erase(it);
            }
        }
    }
}

// ─── WebSocket session handler (one per client) ───────────────────────────────
void do_session(tcp::socket socket) {
    auto ws = std::make_shared<websocket::stream<tcp::socket>>(std::move(socket));
    try {
        ws->set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
        ws->set_option(websocket::stream_base::decorator([](websocket::response_type& res) {
            res.set("Access-Control-Allow-Origin", "*");
        }));
        ws->accept();
        { std::lock_guard<std::mutex> lk(sessions_mtx); g_sessions.insert(ws); }
        std::cout << "[WS] Client connected. total=" << g_sessions.size() << "\n";

        beast::flat_buffer buf;
        boost::system::error_code ec;
        while (true) {
            buf.consume(buf.size());
            ws->read(buf, ec);
            if (ec) break;
            // Incoming: LowState from browser simulation
            parse_and_publish(beast::buffers_to_string(buf.data()));
        }
    } catch (...) {}
    { std::lock_guard<std::mutex> lk(sessions_mtx); g_sessions.erase(ws); }
    std::cout << "[WS] Client disconnected.\n";
}

// ─── WebSocket accept loop ────────────────────────────────────────────────────
void ws_accept_loop(unsigned short port) {
    net::io_context ioc(1);
    tcp::acceptor acceptor(ioc, {tcp::v4(), port});
    std::cout << "[WS] Listening on ws://0.0.0.0:" << port << "\n";
    // Broadcast thread
    std::thread(broadcast_loop).detach();
    while (true) {
        tcp::socket socket(ioc);
        acceptor.accept(socket);
        std::thread([s = std::move(socket)]() mutable { do_session(std::move(s)); }).detach();
    }
}

// ─── DDS LowCmd subscriber callback ──────────────────────────────────────────
void on_lowcmd(const void* msg_ptr) {
    const auto& msg = *static_cast<const unitree_hg::msg::dds_::LowCmd_*>(msg_ptr);
    std::lock_guard<std::mutex> lk(cmd_mtx);
    for (int i = 0; i < NUM_MOTOR; ++i) {
        const auto& m = msg.motor_cmd()[i];
        g_cmd.tau[i] = m.tau();
        g_cmd.kp [i] = m.kp();
        g_cmd.kd [i] = m.kd();
        g_cmd.q  [i] = m.q();
        g_cmd.dq [i] = m.dq();
    }
}

// ─── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    std::string interface = "lo";
    int domain_id = 0;
    for (int i = 1; i < argc - 1; ++i) {
        if (std::string(argv[i]) == "--network") interface = argv[i+1];
        if (std::string(argv[i]) == "--domain")  domain_id = std::stoi(argv[i+1]);
    }

   
    std::cout << "Interface: " << interface << "  domain_id: " << domain_id << "\n";

    // Init DDS
    ChannelFactory::Instance()->Init(domain_id, interface);
    std::cout << "[DDS] Initialized on interface=" << interface << "\n";

    // Init ZMQ publisher — BIND so g1_ctrl AISignal can connect as subscriber
    g_zmq_ctx = new zmq::context_t(1);
    g_zmq_pub = new zmq::socket_t(*g_zmq_ctx, ZMQ_PUB);
    g_zmq_pub->bind("tcp://127.0.0.1:5555");
    std::cout << "[ZMQ] Publisher bound to tcp://127.0.0.1:5555\n";

    // Publisher: LowState (browser simulation state → g1_ctrl)
    lowstate_pub = std::make_shared<ChannelPublisher<unitree_hg::msg::dds_::LowState_>>("rt/lowstate");
    lowstate_pub->InitChannel();
    std::cout << "[DDS] Publishing  rt/lowstate\n";

    // Subscriber: LowCmd (g1_ctrl policy output → browser)
    ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_> lowcmd_sub("rt/lowcmd", on_lowcmd);
    lowcmd_sub.InitChannel();
    std::cout << "[DDS] Subscribing rt/lowcmd\n";
    std::cout << "\nWaiting for g1_ctrl to connect...\n";
    std::cout << "Open http://localhost:5500 in your browser to start simulation.\n\n";

    // Start WebSocket server (blocks)
    ws_accept_loop(WS_PORT);
    return 0;
}
