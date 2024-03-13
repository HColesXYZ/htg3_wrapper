
### htg3_wrapper

This repo is designed to interface with the 'HT_G3_Test_rig' repo so go build that first.

#### Steps:

```
mkdir catkin_ws/src
cd catkin_ws/src
git clone https://github.com/HColesXYZ/htg3_wrapper.git
cd ..
catkin init
catkin build
```
Then in once terminal:
```
roscore
```
And finally in another terminal:
```
roslaunch htg3_wrapper main.launch 
```

> **So far on the test rig I have needed to run**:

```
pip install empy
pip install catkin_tools

```