import os
from glob import glob
from setuptools import setup

package_name = 'graspgen_tutorials'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    zip_safe=True,
    author='Takuya Kiyokawa',
    author_email='taku8926@gmail.com',
    maintainer='Takuya Kiyokawa',
    maintainer_email='taku8926@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Tutorials for GraspGen',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'mesh_graspgen_service = graspgen_tutorials.mesh_graspgen_service:main',
            'pointcloud_graspgen_service = graspgen_tutorials.pointcloud_graspgen_service:main',
        ],
    },
    include_package_data=True,
)