## ROS_NOETIC On Loongarch64

### Build

- [Ros_Noettic_Pack](deb_loongnix/ros_catkin_ws_loongarch.tar.gz)  use this package to resolving dependencies
- [Tutorials](https://wiki.ros.org/Installation/Source) follow this tutorials to install dependencies

##### additional cmd

```bash
#chech deb
rosdep check --from-path src --ignore-src -r -y
#install all deb, avoid missing
rosdep install --from-path src --ignore-src -r -y
```



### Failed_Deb

missing this deb, no relevant error yet.

```bash
System dependencies have not been satisfied:
apt	python3-catkin-pkg-modules
apt	python3-rosdep-modules
apt	sbcl
apt	python3-rospkg-modules
```



### Use

```bash
cd $Workspace
source install_isolated/setup.bash

roscore
rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key
```



### Problem

- $Workspace/install_isolated/lib/turtlesim/turtlesim_node: error while loading shared libraries: libGL.so.1: cannot open shared object file: No such file or directory

  ```bash
  #solve
  locale libGL.so.1 #find path
  export LD_LIBRARY_PATH=$Path:$LD_LIBRARY_PATH #Path = lib pre libGL.so.1
  ```

  
