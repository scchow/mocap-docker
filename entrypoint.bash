# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# setup mocap package
source "$WORK_DIR/devel/setup.bash"


## Appending source command to ~/.bashrc enables autocompletion (ENTRYPOINT alone does not support that)
grep -qxF '. "${WORK_DIR}/entrypoint.bash"' ${HOME}/.bashrc || echo '. "${WORK_DIR}/entrypoint.bash"' >>${HOME}/.bashrc