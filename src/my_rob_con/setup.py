from setuptools import setup
import os
from glob import glob

package_name = 'my_rob_con'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        'my_rob_con',
        'my_rob_con.task4',
        'my_rob_con.task4.scripts',
    ],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'task4/urdf'), glob('my_rob_con/task4/urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aadityaby',
    maintainer_email='you@example.com',
    description='My robot control package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'finger_detection = my_rob_con.task4.scripts.finger_detection:main',
            'left_bot_controller = my_rob_con.task4.scripts.left_bot_controller:main',
            'right_bot_controller = my_rob_con.task4.scripts.right_bot_controller:main',
        ],
    },
)
