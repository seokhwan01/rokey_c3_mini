from setuptools import find_packages, setup

package_name = 'qr_detector_camera'

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
    maintainer='garuda',
    maintainer_email='garuda@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'qr_detector_camera_node = qr_detector_camera.qr_detector_camera_node:main',
        	'qr_detector_webcam_node = qr_detector_camera.qr_detector_webcam:main',
            'test_server_node = qr_detector_camera.test_server:main',

        ],
    },
)
