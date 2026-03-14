FROM ubuntu:22.04

# Prevent interactive prompts during apt install
ENV DEBIAN_FRONTEND=noninteractive

# Update and install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    libyaml-cpp-dev \
    libboost-all-dev \
    libeigen3-dev \
    libspdlog-dev \
    libfmt-dev \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Node.js (Version 20.x)
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
    apt-get install -y nodejs && \
    rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app/mujoco_wasm

# Copy the entire project into the container
COPY . /app/mujoco_wasm

# Ensure unitree_sdk2 is present (in case the directory is empty or missing)
# and build/install it. It installs to /usr/local/ by default.
RUN if [ ! -f "unitree_rl_lab/unitree_sdk2/CMakeLists.txt" ]; then \
        rm -rf unitree_rl_lab/unitree_sdk2 && \
        git clone https://github.com/unitreerobotics/unitree_sdk2.git unitree_rl_lab/unitree_sdk2; \
    fi && \
    cd unitree_rl_lab/unitree_sdk2 && \
    mkdir -p build && cd build && \
    cmake .. -DBUILD_EXAMPLES=OFF && \
    make -j$(nproc) install

# Build g1_29dof controller
RUN cd unitree_rl_lab/deploy/robots/g1_29dof && \
    mkdir -p build && cd build && \
    cmake .. && \
    make -j$(nproc)

# Build g1_ws_bridge web socket bridge
RUN cd unitree_rl_lab/deploy/g1_ws_bridge && \
    mkdir -p build && cd build && \
    cmake .. && \
    make -j$(nproc)

# Install Node dependencies and global server package
RUN npm install && \
    npm install -g five-server pm2

# Expose required ports
# 5500: Web frontend
# 8765: WebSocket port used by g1_ws_bridge
# 5555: ZMQ port
EXPOSE 5500 8765 5555

# Create a startup script to run all 3 processes concurrently
RUN echo '#!/bin/bash\n\
echo "Starting g1_ctrl..."\n\
cd /app/mujoco_wasm/unitree_rl_lab/deploy/robots/g1_29dof/build\n\
./g1_ctrl --network lo &\n\
\n\
echo "Starting g1_ws_bridge..."\n\
sleep 2\n\
cd /app/mujoco_wasm/unitree_rl_lab/deploy/g1_ws_bridge/build\n\
./g1_ws_bridge &\n\
\n\
echo "Starting web server..."\n\
sleep 2\n\
cd /app/mujoco_wasm\n\
npx five-server . --port 5500 --host 0.0.0.0\n\
' > /app/start.sh && chmod +x /app/start.sh

# Run the startup script
CMD ["/app/start.sh"]
