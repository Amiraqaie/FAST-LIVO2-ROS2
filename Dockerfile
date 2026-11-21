# syntax=docker/dockerfile:1.7
# Use the official OSRF ROS image
ARG BUILDER_TAG=latest
FROM amiraqaie1376/builder:${BUILDER_TAG}

# Set environment variables for ROS
ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/$ROS_DISTRO
ENV PATH=$ROS_ROOT/bin:$PATH
ENV LD_LIBRARY_PATH=$ROS_ROOT/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=$ROS_ROOT/lib/python3/dist-packages:$PYTHONPATH
ARG WS=/fast_livo_ws
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    WS=${WS} \
    CPATH=/opt/ros/${ROS_DISTRO}/include
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    ca-certificates \
    cmake \
    git \
    libapr1-dev \
    libboost-thread-dev \
    libeigen3-dev \
    libfmt-dev \
    libopencv-dev \
    libpcl-dev \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-cmake-auto \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-rcl-interfaces \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclcpp-components \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-rcutils \
    ros-${ROS_DISTRO}-ros2bag \
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-rosbag2-storage-default-plugins \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-sophus \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

# livox_ros_driver2 links against Livox-SDK2 from /usr/local.
RUN git clone --depth=1 https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 \
    && cmake -S /tmp/Livox-SDK2 -B /tmp/Livox-SDK2/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/Livox-SDK2/build --parallel "$(nproc)" \
    && cmake --install /tmp/Livox-SDK2/build \
    && ldconfig \
    && rm -rf /tmp/Livox-SDK2

WORKDIR ${WS}

RUN mkdir -p ${WS}/src \
    && git clone --depth=1 https://github.com/Livox-SDK/livox_ros_driver2.git ${WS}/src/livox_ros_driver2

# Clone and patch rpg_vikit camera_loader.cpp
RUN git clone --depth=1 \
    https://github.com/U-AMC/rpg_vikit_rational_polynomial.git \
    ${WS}/src/rpg_vikit \
    && python3 - <<'PY'
from pathlib import Path

p = Path("/fast_livo_ws/src/rpg_vikit/vikit_ros/src/camera_loader.cpp")
s = p.read_text()

start = s.index("/// Load from ROS Namespace")
end = s.index("} // namespace camera_loader", start)

replacement = r'''
/// Load a single camera from a remote ROS parameter namespace.
bool loadFromRosNs(
    const rclcpp::Node::SharedPtr &nh,
    const std::string &ns,
    vk::AbstractCamera *&cam)
{
    cam = nullptr;

    const std::string cam_model =
        getRemoteParam<std::string>(nh, ns, "cam_model", "");

    if (cam_model == "Ocam")
    {
        cam = new vk::OmniCamera(
            getRemoteParam<std::string>(nh, ns, "cam_calib_file", ""));
    }
    else if (cam_model == "Pinhole")
    {
        cam = new vk::PinholeCamera(
            getRemoteParam<int>(nh, ns, "cam_width", 0),
            getRemoteParam<int>(nh, ns, "cam_height", 0),
            getRemoteParam<double>(nh, ns, "scale", 1.0),
            getRemoteParam<double>(nh, ns, "cam_fx", 0.0),
            getRemoteParam<double>(nh, ns, "cam_fy", 0.0),
            getRemoteParam<double>(nh, ns, "cam_cx", 0.0),
            getRemoteParam<double>(nh, ns, "cam_cy", 0.0),
            getRemoteParam<double>(nh, ns, "cam_d0", 0.0),
            getRemoteParam<double>(nh, ns, "cam_d1", 0.0),
            getRemoteParam<double>(nh, ns, "cam_d2", 0.0),
            getRemoteParam<double>(nh, ns, "cam_d3", 0.0));
    }
    else if (cam_model == "EquidistantCamera")
    {
        cam = new vk::EquidistantCamera(
            getRemoteParam<int>(nh, ns, "cam_width", 0),
            getRemoteParam<int>(nh, ns, "cam_height", 0),
            getRemoteParam<double>(nh, ns, "scale", 1.0),
            getRemoteParam<double>(nh, ns, "cam_fx", 0.0),
            getRemoteParam<double>(nh, ns, "cam_fy", 0.0),
            getRemoteParam<double>(nh, ns, "cam_cx", 0.0),
            getRemoteParam<double>(nh, ns, "cam_cy", 0.0),
            getRemoteParam<double>(nh, ns, "k1", 0.0),
            getRemoteParam<double>(nh, ns, "k2", 0.0),
            getRemoteParam<double>(nh, ns, "k3", 0.0),
            getRemoteParam<double>(nh, ns, "k4", 0.0));
    }
    else if (cam_model == "PolynomialCamera")
    {
        cam = new vk::PolynomialCamera(
            getRemoteParam<int>(nh, ns, "cam_width", 0),
            getRemoteParam<int>(nh, ns, "cam_height", 0),
            getRemoteParam<double>(nh, ns, "cam_fx", 0.0),
            getRemoteParam<double>(nh, ns, "cam_fy", 0.0),
            getRemoteParam<double>(nh, ns, "cam_cx", 0.0),
            getRemoteParam<double>(nh, ns, "cam_cy", 0.0),
            getRemoteParam<double>(nh, ns, "cam_skew", 0.0),
            getRemoteParam<double>(nh, ns, "k2", 0.0),
            getRemoteParam<double>(nh, ns, "k3", 0.0),
            getRemoteParam<double>(nh, ns, "k4", 0.0),
            getRemoteParam<double>(nh, ns, "k5", 0.0),
            getRemoteParam<double>(nh, ns, "k6", 0.0),
            getRemoteParam<double>(nh, ns, "k7", 0.0));
    }
    else if (cam_model == "RationalPolynomial")
    {
        cam = new vk::RationalPolynomialCamera(
            getRemoteParam<int>(nh, ns, "cam_width", 0),
            getRemoteParam<int>(nh, ns, "cam_height", 0),
            getRemoteParam<double>(nh, ns, "cam_fx", 0.0),
            getRemoteParam<double>(nh, ns, "cam_fy", 0.0),
            getRemoteParam<double>(nh, ns, "cam_cx", 0.0),
            getRemoteParam<double>(nh, ns, "cam_cy", 0.0),
            getRemoteParam<double>(nh, ns, "k1", 0.0),
            getRemoteParam<double>(nh, ns, "k2", 0.0),
            getRemoteParam<double>(nh, ns, "k3", 0.0),
            getRemoteParam<double>(nh, ns, "k4", 0.0),
            getRemoteParam<double>(nh, ns, "k5", 0.0),
            getRemoteParam<double>(nh, ns, "k6", 0.0),
            getRemoteParam<double>(nh, ns, "p1", 0.0),
            getRemoteParam<double>(nh, ns, "p2", 0.0));
    }
    else if (cam_model == "ATAN")
    {
        cam = new vk::ATANCamera(
            getRemoteParam<int>(nh, ns, "cam_width", 0),
            getRemoteParam<int>(nh, ns, "cam_height", 0),
            getRemoteParam<double>(nh, ns, "cam_fx", 0.0),
            getRemoteParam<double>(nh, ns, "cam_fy", 0.0),
            getRemoteParam<double>(nh, ns, "cam_cx", 0.0),
            getRemoteParam<double>(nh, ns, "cam_cy", 0.0),
            getRemoteParam<double>(nh, ns, "cam_d0", 0.0));
    }
    else
    {
        RCLCPP_ERROR(
            nh->get_logger(),
            "Unsupported camera model '%s' in namespace '%s'",
            cam_model.c_str(), ns.c_str());
        return false;
    }

    return cam != nullptr;
}


/// Load multiple cameras from a remote ROS parameter namespace.
bool loadFromRosNs(
    const rclcpp::Node::SharedPtr &nh,
    const std::string &ns,
    std::vector<vk::AbstractCamera *> &cam_list)
{
    cam_list.clear();

    const int cam_num =
        getRemoteParam<int>(nh, ns, "cam_num", 0);

    if (cam_num <= 0)
    {
        RCLCPP_ERROR(
            nh->get_logger(),
            "Invalid cam_num=%d in namespace '%s'",
            cam_num, ns.c_str());
        return false;
    }

    for (int i = 0; i < cam_num; ++i)
    {
        const std::string cam_ns =
            ns + "/cam_" + std::to_string(i);

        const std::string cam_model =
            getRemoteParam<std::string>(nh, cam_ns, "cam_model", "");

        if (cam_model == "FishPoly")
        {
            cam_list.push_back(new vk::PolynomialCamera(
                getRemoteParam<int>(nh, cam_ns, "image_width", 0),
                getRemoteParam<int>(nh, cam_ns, "image_height", 0),
                getRemoteParam<double>(nh, cam_ns, "A11", 0.0),
                getRemoteParam<double>(nh, cam_ns, "A22", 0.0),
                getRemoteParam<double>(nh, cam_ns, "u0", 0.0),
                getRemoteParam<double>(nh, cam_ns, "v0", 0.0),
                getRemoteParam<double>(nh, cam_ns, "A12", 0.0),
                getRemoteParam<double>(nh, cam_ns, "k2", 0.0),
                getRemoteParam<double>(nh, cam_ns, "k3", 0.0),
                getRemoteParam<double>(nh, cam_ns, "k4", 0.0),
                getRemoteParam<double>(nh, cam_ns, "k5", 0.0),
                getRemoteParam<double>(nh, cam_ns, "k6", 0.0),
                getRemoteParam<double>(nh, cam_ns, "k7", 0.0)));
        }
        else if (cam_model == "Pinhole")
        {
            cam_list.push_back(new vk::PinholeCamera(
                getRemoteParam<int>(nh, cam_ns, "cam_width", 0),
                getRemoteParam<int>(nh, cam_ns, "cam_height", 0),
                getRemoteParam<double>(nh, cam_ns, "scale", 1.0),
                getRemoteParam<double>(nh, cam_ns, "cam_fx", 0.0),
                getRemoteParam<double>(nh, cam_ns, "cam_fy", 0.0),
                getRemoteParam<double>(nh, cam_ns, "cam_cx", 0.0),
                getRemoteParam<double>(nh, cam_ns, "cam_cy", 0.0),
                getRemoteParam<double>(nh, cam_ns, "cam_d0", 0.0),
                getRemoteParam<double>(nh, cam_ns, "cam_d1", 0.0),
                getRemoteParam<double>(nh, cam_ns, "cam_d2", 0.0),
                getRemoteParam<double>(nh, cam_ns, "cam_d3", 0.0)));
        }
        else
        {
            RCLCPP_ERROR(
                nh->get_logger(),
                "Unsupported camera model '%s' in namespace '%s'",
                cam_model.c_str(), cam_ns.c_str());

            for (auto *camera : cam_list)
                delete camera;

            cam_list.clear();
            return false;
        }
    }

    return true;
}

'''

p.write_text(s[:start] + replacement + s[end:])
print("Successfully patched all camera_loader.cpp parameter-loading branches.")
PY


# vikit_common is a plain CMake package; install it globally and let colcon build vikit_ros.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cmake -S ${WS}/src/rpg_vikit/vikit_common -B /tmp/vikit_common_build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build /tmp/vikit_common_build --parallel "$(nproc)" \
    && cmake --install /tmp/vikit_common_build \
    && mkdir -p /usr/local/lib/cmake/vikit_common \
    && printf '%s\n' \
      'set(vikit_common_SOURCE_DIR "/usr/local")' \
      'set(vikit_common_DIR "/usr/local/lib/cmake/vikit_common")' \
      'set(vikit_common_INCLUDE_DIR "/usr/local/include")' \
      'set(vikit_common_INCLUDE_DIRS "/usr/local/include")' \
      'set(vikit_common_LIBRARIES "/usr/local/lib/libvikit_common.so")' \
      'set(vikit_common_LIBRARY "/usr/local/lib/libvikit_common.so")' \
      'set(vikit_common_LIBRARY_DIR "/usr/local/lib")' \
      'set(vikit_common_LIBRARY_DIRS "/usr/local/lib")' \
      > /usr/local/lib/cmake/vikit_common/vikit_commonConfig.cmake \
    && ldconfig \
    && touch ${WS}/src/rpg_vikit/vikit_common/COLCON_IGNORE \
    && touch ${WS}/src/rpg_vikit/vikit_py/COLCON_IGNORE \
    && rm -rf /tmp/vikit_common_build

# livox_ros_driver2 switches package metadata by ROS version via build.sh; do it explicitly
# so the whole workspace can be built once together.
RUN cp ${WS}/src/livox_ros_driver2/package_ROS2.xml ${WS}/src/livox_ros_driver2/package.xml \
    && cp -r ${WS}/src/livox_ros_driver2/launch_ROS2 ${WS}/src/livox_ros_driver2/launch

COPY . ${WS}/src/fast_livo

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install --continue-on-error \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DROS_EDITION=ROS2 \
        -DDISTRO_ROS=${ROS_DISTRO}

RUN printf '%s\n' \
    '#!/usr/bin/env bash' \
    'set -e' \
    'source "/opt/ros/${ROS_DISTRO}/setup.bash"' \
    'source "${WS}/install/setup.bash"' \
    'exec "$@"' \
    > /fast_livo_entrypoint.sh \
    && chmod +x /fast_livo_entrypoint.sh

ENTRYPOINT ["/fast_livo_entrypoint.sh"]
CMD ["bash"]
