from setuptools import setup

setup(
    name='fsm_demo',
    version='0.1.0',
    packages=['fsm_demo', 'fsm_demo.states'],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Will',
    maintainer_email='will@example.com',
    description='FSM demo package',
    entry_points={
        'console_scripts': [
            'planner_fsm = fsm_demo.planner_fsm:main',
            'sensor_simulator = fsm_demo.sensor_simulator:main',
            'brake_controller = fsm_demo.brake_controller:main',
            'actuator_logger = fsm_demo.actuator_logger:main',
        ],
    },
)
