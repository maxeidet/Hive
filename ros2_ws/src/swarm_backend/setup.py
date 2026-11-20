from glob import glob

from setuptools import find_packages, setup

package_name = 'swarm_backend'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test', 'tests']),
    package_data={'swarm_backend.ui': ['static/*']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hive',
    maintainer_email='max.eidet@gmail.com',
    description='Python backend for the drone swarm (AI/swarm logic, ROS 2 node)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swarm_shell = swarm_backend.shell:main',
            'fake_detection = swarm_backend.nodes.fake_detection:main',
            'attack_node = swarm_backend.nodes.attack_node:main',
            'base_station_node = swarm_backend.nodes.base_station_node:main',
            'patrol_node = swarm_backend.nodes.patrol_node:main',
            'operator_ui = swarm_backend.nodes.operator_ui:main',
            'hivemind_node = swarm_backend.nodes.hivemind_node:main',
        ],
    },
)
