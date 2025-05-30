from setuptools import setup
from glob import glob


package_name = 'choirjet_examples'
scripts = {
    'simple'   : ['control_panel'],
    }
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/' + package_name, glob('resource/*.rviz')),
        ('share/' + package_name, glob('resource/*.urdf')),
        ('share/' + package_name + '/worlds', glob('worlds/*.wbt')),
        ('share/' + package_name + '/worlds', glob('worlds/*.wbo')),
        ('share/' + package_name + '/worlds/meshes', glob('worlds/meshes/*.stl')),
        ('share/' + package_name + '/worlds/meshes', glob('worlds/meshes/*.obj')),
        ('share/' + package_name + '/config', glob('config/*.lua')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/map', glob('map/map.yaml')),
        ('share/' + package_name + '/map', glob('map/map.pgm')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools','choirbot'],
    zip_safe=True,
    maintainer='opt4smart',
    maintainer_email='opt4smart@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts':
                 [ 
                'goal_pose_gen = choirjet_examples.goal_pose_gen:main',
            ] + [ 
                'path_publisher_node = choirjet_examples.path_publisher_node:main',
            ] + [ 
                'odom_publisher_node = choirjet_examples.odom_publisher_node:main',
            ] + [ 
                'ackermann_driver = choirjet_examples.ackermann_driver:main',
            ] + [ 
                'lidar_converter_node = choirjet_examples.lidar_converter_node:main',
            ] + [ 
                'ackermann_controller = choirjet_examples.ackermann_controller:main',
            ] + [
                'choirjet_{0}_{1} = choirjet_examples.{0}.{1}:main'.format(package, file)
            for package, files in scripts.items() 
            for file in files
        ],
    },
)

