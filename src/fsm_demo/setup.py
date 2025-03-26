from setuptools import setup

package_name = 'fsm_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 FSM Python package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'sensor_simulator = fsm_demo.sensor_simulator:main',
            'planner_fsm = fsm_demo.planner_fsm:main',
            'actuator_logger = fsm_demo.actuator_logger:main',
            'brake_controller = fsm_demo.brake_controller:main',
        ],
    },
)
