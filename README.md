# 1、从rosbag中读取图像和位姿

- 安装编译工具

  ```shell
  sudo apt install -y python-wstool python3-catkin-tools
  ```

- 创建ROS工作空间

  ```shell
  mkdir catkin_ws
  cd catkin_ws
  wstool init src
  ```

- 获取源码并编译

  ```shell
  cd src
  git clone https://github.com/SergeyuiL/rosbag_play.git
  cd ..
  catkin build
  source devel/setup.bash
  ```

- 获取rosbag数据并放入rosbag文件夹

  ```shell
  mkdir -p ~/catkin_ws/src/rosbag_play/rosbag
  mv undist_data.bag ~/catkin_ws/src/rosbag_play/rosbag/
  ```

- 读取rosbag转化为RGB图、深度图和位姿

  ```shell
  cd ~/catkin_ws/src/rosbag_play/
  ./scripts/getdata.sh
  # 如果脚本运行提示没权限
  sudo chmod +x getdata.py getdata.sh
  # 如果系统没装tmux
  sudo apt install tmux
  ```

  运行完后会在同目录下生成data文件夹，包含处理后的文件

  要关掉tmux会话，在任一tmux终端中输入：

  ```shell
  tmux kill-server
  ```

# 2、vlmaps构建地图

- 参考https://github.com/SergeyuiL/vlmaps.git 中的README拉取源码和创建vlmaps虚拟环境

- 切换到demo分支

  ```shell
  git checkout demo
  ```

- 在```Mydemo```目录下：

  - ```topdown_color_map.py```：创建top-down color map

    - 记得调整环境和数据路径到对应位置

      ```python
      sys.path.insert(0, '/home/sg/Workspace/vlmaps')
      data_save_dir = "/home/sg/Workspace/catkin_ws/src/rosbag_play/data"
      ```

  - ```show_color_map.py```：显示top-down color map

    - 同理调整

      ```python
      sys.path.insert(0, '/home/sg/Workspace/vlmaps')
      data_dir = "/home/sg/Workspace/catkin_ws/src/rosbag_play/data"
      ```

- 可参考```examples/clip_mapping_lseg_from_scratch_batch.py```的逻辑进行语义分割，实际数据集的读取和点云转换见```Mydemo/topdown_color_map.py```下的```my_*```函数

