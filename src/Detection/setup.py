from setuptools import find_packages, setup

package_name = 'Detection'

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
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capture = Detection.image_capture:main',
            'detaction = Detection.detection_publisher:main',
            'tracking = Detection.tracking_pubilsher:main',
            'display = Detection.display:main',
        ],
    },
)