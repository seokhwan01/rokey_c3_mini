# rokey_c3_mini

ROKEY 지능-1 C-3조 파이널 프로젝트 결과물입니다. 

[IP-Webcam 다운로드(Android)](https://play.google.com/store/apps/details?id=com.pas.webcam&hl=ko&pli=1)


# YOLOv8 Finetuning 모델 다운받기
```
$ cd ~/rokey_c3_mini/src/mini_project/mini_project
$ wget --no-check-certificate 'https://drive.google.com/file/d/1JZEP8SVHQs3CQghculh4TJQS0eNJ2DHr/view?usp=sharing' -O model.zip
$ unzip model.zip
```

```
$ cd ~/rokey_c3_mini
$ source install/setup.bash
```

# 👀Detection 모듈 실행

```
$ ros2 launch mini_project detection_system.launch
```


# 🤖Navigaiton 모듈 실행 (순서 중요)

### (1) turtlebot4_navigation 실행 (robot name 각자 맞춰서 수정 필요)
```
# for robot8
$ ros2 launch turtlebot4_navigation localization.launch.py namespace:=robot8 map:=$HOME/rokey_c3_mini/src/mini_project/map/map.yaml
$ ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot8
$ ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot8

# for robot9
$ ros2 launch turtlebot4_navigation localization.launch.py namespace:=robot9 map:=$HOME/rokey_c3_mini/src/mini_project/map/map.yaml
$ ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot9
$ ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot9
```

### (2) 커스텀 ROS2 노드 실행
```
# Action Manager
$ ros2 run mini_project action_manager --ros-args -r __ns:=/robot8 -r /tf:=/robot8/tf -r /tf_static:=/robot8/tf_static
$ ros2 run mini_project action_manager --ros-args -r __ns:=/robot9 -r /tf:=/robot9/tf -r /tf_static:=/robot9/tf_static

# Following Car
$ ros2 run mini_project following_car --ros-args -r  __ns:=/robot8 -r /tf:=/robot8/tf -r /tf_static:=/robot8/tf_static

# Task Manager
$ ros2 run mini_project task_manager
```
