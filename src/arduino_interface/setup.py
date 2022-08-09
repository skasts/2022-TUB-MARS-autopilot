from setuptools import setup

package_name = 'arduino_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alicia S.',
    maintainer_email='todo@todo.todo',
    description='Implements a hardware interface for an Arduino that passes wheel velocities.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_interface = arduino_interface.arduino_interface:main',
        ],
    },
)
