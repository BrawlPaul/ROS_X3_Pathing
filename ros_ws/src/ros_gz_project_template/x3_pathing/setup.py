from setuptools import find_packages, setup

package_name = 'x3_pathing'

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
    maintainer='benjamin',
    maintainer_email='benjamin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'draw_square = x3_pathing.draw_square:main',
            'keyboard_teleop = x3_pathing.keyboard_teleop:main',
            'altimeter_publisher = x3_pathing.altimeter_publisher:main',
            'altitude_controller = x3_pathing.altitude_controller:main'
        ],
    },
)
