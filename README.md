# Pickachu
2018 Taiwan Pickathon Challenge

## Download and Build
```bash
$ mkdir pickachu_ws/src && cd pickachu_ws/src
$ git clone https://github.com/tkuiclab/Pickachu.git
$ cd ~/pickachu_ws
$ catkin_make
```

## Startup Process
```bash
$ source <WORKSPACE_OF_PICKACHU>/devel/setup.bash
$ roslaunch accupick3d accupick3d.launch
$ roslaunch core core.launch
```

## How to use
```bash
# Take a picture
$ rosservice call /take_picture "{}"

# Get & Save picture at <WORKSPACE_OF_PICKCHU>/src/core/images/
$ rosservice call /get_picture "{}"
```
