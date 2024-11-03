FROM osrf/ros:humble-desktop

WORKDIR "/workstation/src"
RUN git clone https://github.com/ros2/teleop_twist_keyboard.git

COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]