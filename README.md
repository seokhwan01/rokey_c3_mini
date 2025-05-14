# YOLOv8 Finetuning 모델 다운받기
```
$ cd ~/rokey_c3_mini/src/detection/detection
$ wget --no-check-certificate 'https://drive.google.com/uc?export=download&id=1JZEP8SVHQs3CQghculh4TJQS0eNJ2DHr' -O weights.zip
$ unzip weights.zip
```

# 👀Detection 모듈 실행 방법

```
$ source install/setup.bash
```

```
# Image 발행 (oak-d 용으로 변경 필요)
$ ros2 run detection test_image_publisher
```

```
$ ros2 run detection detection_manager
$ ros2 run detection tracking_manager
$ ros2 run detection display_manager
```
