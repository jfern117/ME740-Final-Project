from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tb_sim_custom'

# models = glob(os.path.join('models', '**', '*.sdf'), recursive=True)
# print(type(models))

#I got help from chatgpt for the concept of this setup but I've implemented it myself
#The model files are in subdirectories we'd like to preserve
model_files = []
for root, dirs, files in os.walk('models'):
    for file in files:

        _, ext = os.path.splitext(file)
        if ext == ".sdf":
            source_path = os.path.join(root, file)
            install_path = os.path.join('share', package_name, root)
            model_files.append((install_path, [source_path]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        *model_files,
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jpfern',
    maintainer_email='jfern117@gmail.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
