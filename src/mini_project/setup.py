from setuptools import find_packages, setup

package_name = 'mini_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leejinwon',
    maintainer_email='leejinwon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_convertor = mini_project.detection.ImageConvertor:main',
            'detection_manager = mini_project.detection.DetectionManager:main',
            'tracking_manager = mini_project.detection.TrackingManager:main',
            'display_manager = mini_project.detection.DisplayManager:main',
            'qr_detection = mini_project.detection.QRDetection:main',

            'following_car = mini_project.navigation.FollowingCar:main',
            'goal_pose_extractor = mini_project.navigation.GoalPoseExtractor:main',
            'nav_to_pose = mini_project.navigation.NavToPose:main',
            'goal_manager = mini_project.navigation.GoalManager:main',
        ],
    },
)
