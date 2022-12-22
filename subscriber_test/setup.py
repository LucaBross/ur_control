from setuptools import setup
import os
from glob import glob

package_name = 'subscriber_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luca',
    maintainer_email='luca@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tool_current_subscriber=subscriber_test.tool_current_subscriber:main',
            'succesful_grasp_detect=subscriber_test.succesfull_grasp_detect:main',
            'encoder_state_detect=subscriber_test.encoder_state_detect:main',
            'gripper_close=subscriber_test.gripper_close:main',
            'gripper_open=subscriber_test.gripper_open:main',
            'Current_Writer=subscriber_test.Current_writer:main',
        ],
    },
)
