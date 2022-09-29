from setuptools import setup

package_name = 'problem_3_5'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arjan Gupta',
    maintainer_email='agupta11@wpi.edu',
    description='HW 3 ROS assignment for RBE500',
    license='Property of Worcester Polytechnic Institute',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
