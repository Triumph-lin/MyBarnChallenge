<p align="center">
  <img width = "100%" src='the-barn-challenge/res/BARN_Challenge.png' />
  </p>

--------------------------------------------------------------------------------

# ICRA BARN Navigation Challenge

## Updates:
* 02/04/2024: Adding 60 [DynaBARN](https://github.com/aninair1905/DynaBARN) environments. DynaBARN environments can be accessed by world indexes from 300-359.

## Requirements
If you run it on a local machine without containers:
* ROS version at least Kinetic
* CMake version at least 3.0.2
* Python version at least 3.6
* Python packages: defusedxml, rospkg, netifaces, numpy

The requirements above are just suggestions. If you run into any issue, please contact organizers for help (zfxu@utexas.edu).

## Installation
Follow the instructions below to run simulations on your local machines. 

### 1. Create ROS workspace
```
mkdir -p /<YOUR_HOME_DIR>/jackal_ws/src
cd /<YOUR_HOME_DIR>/jackal_ws/src
```

### 2. Clone this repo
**Attention:This code is based on ROS Noetic.**
```
git clone https://github.com/Triumph-lin/MyBarnChallenge.git
```
### 3. Create a virtual environment (use barn.yaml )
```
cd MyBarnChallenge
conda env create -f barn.yaml
```
Please replace the path with your desired location. 
```
prefix: /home/YOUR_USERNAME/miniconda3/envs/barn
```

### 4. Install ROS package dependencies
```
cd ..
source /opt/ros/noetic/setup.bash
rosdep init; rosdep update
rosdep install -y --from-paths . --ignore-src --rosdistro=noetic
```

### 5. Build the workspace 
```
catkin_make
source devel/setup.bash
```


## Run Simulations
Navigate to the folder of this repo. 

If you run it on your local machines: 
```
cd jackal_ws/src/MyBarnChallenge/the-barn-challenge
python3 run.py --world_idx 0 --gui
```
The program accepts the following command-line arguments:
(1)The ```--world_idx <filename>``` parameter accepts values from 0 to 359, where worlds 0-299 are static environments and worlds 300-359 are dynamic environments with moving obstacles.
(2)The ```--gui``` parameter enables Gazebo visualization (GUI mode). If this flag is omitted, the simulation runs in headless mode without graphical interface.
(3)The ```--out <filename>``` parameter specifies the output file to save simulation results. Results will be written to this file for later analysis.

A successful run should print the episode status (collided/succeeded/timeout) and the time cost in second:
> \>>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<
>
> Navigation collided with time 27.2930 (s)

> \>>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<
>
> Navigation succeeded with time 29.4610 (s)


> \>>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<
>
>Navigation timeout with time 100.0000 (s)

## Test the navigation stack
We provide a bash script `test.sh` to run your navigation stack on 300 BARN worlds with 1 run for each world. 
```'
bash test.sh
python mean.py --file_name out.txt
```
You should see the report like this:
>Avg Success: 1.0
Avg Collision: 0.0
Avg Timeout: 0.0
Avf Time: 9.7917
Avg Score: 0.4802


