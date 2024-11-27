# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Use the quay.io/qm-images/ros2:rolling base image
FROM quay.io/fedora-sig-robotics/ros2:jazzy-cs9

ARG ROS2_WS=/opt/ros2_ws

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN dnf update -y && \
    dnf install -y \
    python3-pip \
    && dnf clean all

RUN pip3 install -U colcon-common-extensions

COPY rootfs/ /

RUN mkdir -p ${ROS2_WS}

COPY ./src ${ROS2_WS}

# Set up the ROS2 workspace
WORKDIR ${ROS2_WS}
# RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN . /opt/ros/jazzy/setup.bash && colcon build

ENTRYPOINT ["/usr/local/bin/roshell"]
