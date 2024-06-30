# Reading Images and Poses from ROSbag

- (opt)Record ROSbag

  ```shell
  rosbag record --lz4 --repeat-latched /tf /tf_static /locobot/camera/aligned_depth_to_color/camera_info /locobot/camera/aligned_depth_to_color/image_raw /locobot/camera/color/image_raw
  ```

- Installing Compilation Tools

  ```shell
  sudo apt install -y python3-wstool python3-catkin-tools
  ```

- Creating a ROS Workspace

  ```shell
  mkdir catkin_ws
  cd catkin_ws
  wstool init src
  ```

- Fetching Source Code and Compiling

  ```shell
  cd src
  git clone https://github.com/SergeyuiL/rosbag_play.git
  cd ..
  catkin build
  source devel/setup.bash
  ```

- Getting ROSbag Data and Placing it in the ROSbag Folder

  ```shell
  mkdir -p ~/catkin_ws/src/rosbag_play/rosbag
  mv undist_data.bag ~/catkin_ws/src/rosbag_play/rosbag/
  ```

- Reading ROSbag to Convert into RGB Images, Depth Images, and Poses

  ```shell
  cd ~/catkin_ws/src/rosbag_play/
  ./scripts/getdata.sh
  # if permission denied
  sudo chmod +x getdata.py getdata.sh
  # if no tmux
  sudo apt install tmux
  ```

  After running, you will be prompted to enter the path for the ROSbag file and the target data folder path.

  To close the tmux session, enter the following in any tmux terminal:

  ```shell
  tmux kill-server
  ```
