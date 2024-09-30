from setuptools import setup

package_name = 'wall_following_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/wall_following_launch.py']),
       
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
     entry_points={
        'console_scripts': [
            'lidar_processing_node = wall_following_pkg.lidar_processing_node:main',
            'control_node = wall_following_pkg.control_node:main',
        ],
    },
)
