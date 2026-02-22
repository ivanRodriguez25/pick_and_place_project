from setuptools import find_packages, setup

package_name = 'panda_moveit_demo'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [            
            'joint_space_motion = panda_moveit_demo.joint_space_planning:main',
            'carteian_space_motion_joint = panda_moveit_demo.cartesian_space_planning_joint:main',
            'carteian_space_motion = panda_moveit_demo.cartesian_space_planning:main',
            'continuous_movement = panda_moveit_demo.continuous_movement:main',
        ],
    },
)
