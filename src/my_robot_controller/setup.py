from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # فایل‌های launch با پسوند .launch.py
        (os.path.join('share', package_name, 'launch'), 
         ['launch/launch_simulation.launch.py', 
          'launch/launch_node.launch.py']),
        
        ('share/' + package_name + '/worlds', ['worlds/roadmap.world']),
        ('share/' + package_name + '/models/left_sign', 
         ['models/left_sign/model.sdf', 'models/left_sign/model.config']),
        ('share/' + package_name + '/models/right_sign', 
         ['models/right_sign/model.sdf', 'models/right_sign/model.config']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samane',
    maintainer_email='samane@todo.todo',
    description='Robot controller for traffic sign detection',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "move_node = my_robot_controller.node_controll_motor:main",
            "detect_node = my_robot_controller.node_detect:main",
            "camera_node = my_robot_controller.node_camera:main",
        ],
    },
)
