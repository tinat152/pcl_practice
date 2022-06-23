cd
export DISPLAY=:1
export LC_ALL=C
export PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64:/home/user/.catkin_ws_python3/src/TensorRT-7.0.0.11/lib:${LD_LIBRARY_PATH}

alias rosenv='export | grep ROS'
alias public_ip="curl -s checkip.dyndns.org | sed -e 's/.*Current IP Address: //' -e 's/<.*$//'"
alias instance_id="curl -s http://169.254.169.254/latest/meta-data/instance-id"
alias rosbridge_address='echo "wss://$(instance_id).robotigniteacademy.com/${SLOT_PREFIX}/rosbridge/"'
alias webpage_address='echo "https://$(instance_id).robotigniteacademy.com/${SLOT_PREFIX}/webpage/"'
alias webvideo_address='echo "https://$(instance_id).robotigniteacademy.com/${SLOT_PREFIX}/cameras/"'
alias tensorboard_address='echo "https://$(instance_id).robotigniteacademy.com/${SLOT_PREFIX}/tensorboard/"'
alias jenkins_prefix='echo "/${SLOT_PREFIX}/jenkins/"'
alias jenkins_address='echo "https://$(instance_id).robotigniteacademy.com$(jenkins_prefix)"'

source /usr/share/gazebo/setup.sh
source /opt/ros/${SELECTED_ROS_DISTRO}/setup.bash

if [ -n "${SOURCE_ROS1_WS}" ]; then
    source /home/simulations/public_sim_ws/devel/setup.bash &> /dev/null
    source ~/simulation_ws/devel/setup.bash &> /dev/null
    source ~/catkin_ws/devel/setup.bash &> /dev/null;
    if ! catkin_make -h &> /dev/null; then  # This may happen if somehow rosject_save fails
        export PATH=/opt/ros/${SELECTED_ROS_DISTRO}/bin:${PATH}
        export LD_LIBRARY_PATH=/home/user/catkin_ws/devel/lib:/home/simulations/public_sim_ws/devel/lib:/opt/ros/${SELECTED_ROS_DISTRO}/lib:/opt/ros/${SELECTED_ROS_DISTRO}/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}
        export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/opt/ros/${SELECTED_ROS_DISTRO}
        export PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:/opt/ros/${SELECTED_ROS_DISTRO}/lib/pkgconfig:/opt/ros/${SELECTED_ROS_DISTRO}/lib/x86_64-linux-gnu/pkgconfig
        rm -rf ~/catkin_ws/build/catkin ~/catkin_ws/build/CMakeCache.txt ~/catkin_ws/build/CMakeFiles/
    fi
fi

if [ -n "${SOURCE_ROS2_WS}" ]; then
    source /home/user/ros2_ws/install/local_setup.bash 2> /dev/null
    source /home/simulations/ros2_sims_ws/install/setup.bash
fi

# Alberto asked path below on Jun, 5, 2020.
# We check if it is kinetic because "rostopic list" fails on melodic/eloquent
if env | grep 'ROS_DISTRO=kinetic' &>/dev/null ; then
    export PYTHONPATH=/home/simulations/public_sim_ws/devel/lib/python2.7/dist-packages:/opt/ros/kinetic/lib/python2.7/dist-packages:/home/simulations/public_sim_ws/src/all/ros_basics_examples/python_course_class:/home/simulations/public_sim_ws/src/ros_basics_examples/python_course_class:/home/simulations/public_sim_ws/src/all/kinematics_course_utils/kinematics
fi

(rospack profile &> /dev/null &)
cd

# -----------------------------------------------------
#           Note from The Construct
# -----------------------------------------------------
# Please, do not modify this file.
# Changes made here will not be persisted.
# Add your custom exports to the end of ~/.bashrc instead.
# -----------------------------------------------------
