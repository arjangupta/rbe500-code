from setuptools import setup

package_name = 'homework1'

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
    description='HW 1 assignment for RBE500',
    license='Property of Worcester Polytechnic Institute',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = homework1.hw1_publisher:main',
            'listener = homework1.hw1_subscriber:main'
        ],
    },
)
