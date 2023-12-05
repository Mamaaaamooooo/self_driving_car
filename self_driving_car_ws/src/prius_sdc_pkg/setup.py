from setuptools import setup
import os
from glob import glob

package_name = 'prius_sdc_pkg'
config_mudule = 'prius_sdc_pkg/config'
det_module = 'prius_sdc_pkg/Detection'
det_l_module = 'prius_sdc_pkg/Detection/Lanes'


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,config_mudule,det_module, det_l_module],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name,'launch'), glob('launch/*')),
    (os.path.join('share', package_name,'worlds'), glob('worlds/*')),
    (os.path.join('lib', package_name), glob('scripts/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='goldminsu',
    maintainer_email='goldminsu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recorder_node = prius_sdc_pkg.video_recorder:main',
            'driver_node = prius_sdc_pkg.driving_node:main', 
            'spawner_node = prius_sdc_pkg.sdf_spawner:main',
            'computer_vision_node = prius_sdc_pkg.computer_vision_node:main',
        ],
    },
)
