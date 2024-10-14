FROM osrf/ros:humble-desktop

WORKDIR "${HOME_DIR}/workstation/src"
RUN git clone https://github.com/ros2/teleop_twist_keyboard.git


WORKDIR "${HOME_DIR}"

COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]