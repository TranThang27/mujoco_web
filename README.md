<p align="center">
  <a href="https://zalo.github.io/mujoco_wasm/"><img src="./assets/MuJoCoWasmLogo.png" href></a>
</p>
<p align="left">
  <a href="https://github.com/zalo/mujoco_wasm/deployments/activity_log?environment=github-pages">
      <img src="https://img.shields.io/github/deployments/zalo/mujoco_wasm/github-pages?label=Github%20Pages%20Deployment" title="Github Pages Deployment"></a>
  <!--<a href="https://github.com/zalo/mujoco_wasm/deployments/activity_log?environment=Production">
      <img src="https://img.shields.io/github/deployments/zalo/mujoco_wasm/Production?label=Vercel%20Deployment" title="Vercel Deployment"></a> -->
  <!--<a href="https://lgtm.com/projects/g/zalo/mujoco_wasm/context:javascript">
      <img alt="Language grade: JavaScript" src="https://img.shields.io/lgtm/grade/javascript/g/zalo/mujoco_wasm.svg?logo=lgtm&logoWidth=18"/></a> -->
  <a href="https://github.com/zalo/mujoco_wasm/commits/main">
      <img src="https://img.shields.io/github/last-commit/zalo/mujoco_wasm" title="Last Commit Date"></a>
  <a href="https://github.com/zalo/mujoco_wasm/blob/main/LICENSE">
      <img src="https://img.shields.io/badge/license-MIT-brightgreen" title="License: MIT"></a>
</p>

## The Power of MuJoCo in your Browser.

Load and Run MuJoCo 3.3.8 Models using JavaScript and the official MuJoCo WebAssembly Bindings.

This project used to be a WASM compilation and set of javascript bindings for MuJoCo, but since Deepmind completed the official MuJoCo bindings, this project is now just a small demo suite in the `examples` folder.

### [See the Live Demo Here](https://zalo.github.io/mujoco_wasm/)

### [See a more Advanced Example Here](https://kzakka.com/robopianist/)

## Build

Simply ensure `npm` is installed and run `npm install` to pull three.js and MuJoCo's Official WASM bindings.

To serve and run the index.html page while developing, use an HTTP Server.  I like to use [five-server](https://github.com/yandeu/five-server).

## How to Run with Docker (Recommended)

By using Docker, you can instantly build and launch the server, websocket bridge, and controller without installing dependencies on your host.

1) Build the image
```bash
docker build -t mujoco_wasm_env .
```

2) Run the container
```bash
docker run -p 5500:5500 -p 8765:8765 -it mujoco_wasm_env
```

3) Open `http://localhost:5500` in your web browser.

## How to Run Manually (Linux)

1) Install unitree rl lab and set up the environment

```bash
sudo apt install -y libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev
# Install unitree_sdk2
# Note: from the root mujoco_wasm folder
git clone https://github.com/unitreerobotics/unitree_sdk2.git unitree_rl_lab/unitree_sdk2
cd unitree_rl_lab/unitree_sdk2
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=OFF # Install on the /usr/local directory
sudo make install

# Build g1_29dof controller
cd ../deploy/robots/g1_29dof
mkdir build && cd build
cmake .. && make
```

2) Build g1_ws_bridge

```bash
# from the root mujoco_wasm folder
cd unitree_rl_lab/deploy/g1_ws_bridge
mkdir build && cd build
cmake .. && make
```

3) Run with 3 terminals (from the root folder):

**Terminal 1 (Controller):**
```bash
cd unitree_rl_lab/deploy/robots/g1_29dof/build
./g1_ctrl --network lo
```

**Terminal 2 (WebSocket Bridge):**
```bash
cd unitree_rl_lab/deploy/g1_ws_bridge/build
./g1_ws_bridge
```

**Terminal 3 (Web Server):**
```bash
npm install
five-server . 
```

