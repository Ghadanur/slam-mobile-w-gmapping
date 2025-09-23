from setuptools import find_packages, setup

package_name = 'rover_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/rover.urdf']),
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),
        ('share/' + package_name + '/launch', ['launch/gazebo_launch.py']),
        ('share/' + package_name + '/launch', ['launch/slam_gmapping.launch.py']),
        ('share/' + package_name + '/launch', ['launch/main.launch.py']),
        ('share/' + package_name + '/launch', ['launch/warehouse_rover.launch.py']),
        #('share/' + package_name + '/worlds', ['worlds/moon.world']), 
        ('share/' + package_name + '/worlds', ['worlds/Warehouse.world']),
        #(os.path.join('share', package_name, 'models'), glob('models/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghada',
    maintainer_email='ghadnoor.el@gmail.com',
    description='A URDF robot for RViz and Gazebo',
    license='MIT',
    tests_require=['pytest'],
)
