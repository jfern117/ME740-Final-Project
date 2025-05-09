from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'swarm_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jfern117',
    maintainer_email='jfern117@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim = swarm_gui.sim_node:main',
            'gui = swarm_gui.gui_node:main',
            'controller = swarm_gui.keyboard_control_node:main',
            'gazebo_interface = swarm_gui.gazebo_interface:main',
            'autonmous_control = swarm_gui.autonomous_agent_ctl:main'
        ],
    },
)
