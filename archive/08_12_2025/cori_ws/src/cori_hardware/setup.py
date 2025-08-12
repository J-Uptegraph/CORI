from setuptools import find_packages, setup

package_name = 'cori_hardware'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/arduino', ['arduino/cori_head_controller/cori_head_controller.ino']),
        ('share/' + package_name + '/launch', ['launch/hardware_bridge.launch.py', 'launch/realtime_web_control.launch.py']),
        ('share/' + package_name + '/web', ['cori_hardware/index.html']),
    ],
    install_requires=['setuptools', 'websockets>=10.0'],
    zip_safe=True,
    maintainer='CORI Team',
    maintainer_email='cori@example.com',
    description='CORI Hardware Interface - Arduino bridge for physical head movement',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_bridge = cori_hardware.arduino_bridge:main',
            'realtime_web_control = cori_hardware.realtime_web_control:main',
            'hardware_test = cori_hardware.hardware_test:main',
            'hardware_integration = cori_hardware.hardware_integration:main',
        ],
    },
)