from setuptools import setup

package_name = 'covariance_adder'

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
    maintainer='Simon Kast',
    maintainer_email='todo@todo.todo',
    description='Adds covariance information to a PoseStamped msg',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'covariance_adder = covariance_adder.covariance_adder:main',
        ],
    },
)
