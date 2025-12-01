from setuptools import find_packages, setup
import glob # for file searching
import os   # for file path manipulation

package_name = 'uiabot_mini_bringup'

launch_files = glob.glob(os.path.join('launch', '*'))
config_files = glob.glob(os.path.join('config', '*'))

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), launch_files),
        (os.path.join('share', package_name, 'config'), config_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gruppe-6',
    maintainer_email='gruppe-6@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
