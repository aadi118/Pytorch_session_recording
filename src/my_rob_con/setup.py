from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'my_rob_con'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='my_rob_con'),
    package_dir={'': 'my_rob_con'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('my_rob_con/task4/launch/*.py')),
        (os.path.join('share', package_name, 'urdf'),
            glob('my_rob_con/task4/urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aadityaby',
    maintainer_email='aaditya.yeola@gmail.com',
    description='Dual robot project with gesture control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_to_cmd_vel = my_rob_con.task4.gesture_to_cmd_vel:main',
            'finger_detection = my_rob_con.task4.scripts.finger_detection:main',
        ],
    },
)
