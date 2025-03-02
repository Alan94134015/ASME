from setuptools import find_packages, setup

package_name = 'robot_pkg'

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
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot = robot_pkg.robot_demo:main',
            'step_motor = robot_pkg.step_motor_demo:main',
            'bottom = robot_pkg.bottom_demo:main',
            'execute = robot_pkg.execute_demo:main',
            'servo = robot_pkg.servo_demo:main'
        ],
    },
)
