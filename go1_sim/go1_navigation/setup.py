from setuptools import find_packages, setup

package_name = 'go1_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/navigation.launch.py']),
        ('share/' + package_name + '/params', ['params/nav2_params.yaml']),
        ('share/' + package_name + '/xml' , ['xml/go1_bt.xml']),
        ('share/' + package_name + '/map' , ['map/map.yaml']),
        ('share/' + package_name + '/map' , ['map/map.pgm']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Atharva Ghotavadekar',
    maintainer_email='atharvagh1@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_tf_publisher = go1_navigation.tf_publisher:main',
            'static_map_publisher = go1_navigation.static_map_publisher:main'
        ],
    },
)
