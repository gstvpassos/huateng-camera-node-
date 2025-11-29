from setuptools import find_packages
from setuptools import setup

setup(
    name='huateng_camera_node',
    version='0.0.1',
    packages=find_packages(
        include=('huateng_camera_node', 'huateng_camera_node.*')),
)
