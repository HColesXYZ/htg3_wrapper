
### htg3_wrapper

This repo is designed to interface with the 'HT_G3_Test_rig' so make sure you have that and have set ${HTG3_COLLECTOR_DIR}.

#### PC Steps:

```
mkdir catkin_ws/src
cd catkin_ws/src
git clone https://github.com/HColesXYZ/htg3_wrapper.git
cd ..
mamba activate ros_env
catkin init
catkin build
source devel/setup.bash
```
Then in one terminal:
```
roscore
```
Then in the next terminal:
```
rosbag record -a
```
And finally in another terminal:
```
roslaunch htg3_wrapper main.launch 
```
or
```
roslaunch htg3_wrapper ros_out.launch 
```

#### RPi Steps:
In one terminal:
```
cd {RPI_ROOT}/main/release
sudo ./startup.sh
```
In another:
```
cd {RPI_ROOT}/scripts
python press_button.py
```

> **So far on the test rig I have needed to run**:

```
pip install empy
pip install catkin_tools
conda install gflags
```
> **So far for the RasPi I have needed to**:

```
sudo crontab -e
# out the @reboot ./startup.sh
sudo reboot
```