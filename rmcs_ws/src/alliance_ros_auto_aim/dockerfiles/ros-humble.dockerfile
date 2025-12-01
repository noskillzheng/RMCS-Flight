FROM ros:humble

# 设置工作目录
WORKDIR /ros_ws

# 拷贝源码到容器中
COPY . /ros_ws

RUN /ros_ws/env/common_install

# 安装 colcon 和依赖工具
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    build-essential \
    git \
    && rm -rf /var/lib/apt/lists/*

# 构建指令（可选，通常由 GitHub Actions 触发）
# RUN . /opt/ros/humble/setup.sh && colcon build
