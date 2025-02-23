from setuptools import find_packages, setup

package_name = 'swarm_gui'

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
    maintainer='jfern117',
    maintainer_email='jfern117@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = swarm_gui.test_node:main',
            'talker = swarm_gui.test_publisher:main',
            'listener = swarm_gui.gui_node:main'
        ],
    },
)
