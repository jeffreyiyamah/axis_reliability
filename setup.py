from setuptools import setup

package_name = 'axis_reliability'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/axis_demo.launch.py']),
        ('share/' + package_name + '/config', ['config/axis_reliability.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='Robust axis navigation with fallback and recovery state machine.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'axis_reliability = axis_reliability.axis_reliability_node:main',
        ],
    },
)
