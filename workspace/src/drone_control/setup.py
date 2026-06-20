from setuptools import find_packages, setup

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', [
            'launch/coverage_run.launch.py',
            'launch/maze_run.launch.py'
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='quanmh25',
    maintainer_email='maihoangquan250205@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'coverage_node = drone_control.test.coverage_node:main',
            'coverage_docking_node = drone_control.coverage_docking_node:main',
            'maze_docking_node = drone_control.maze_docking_node:main',
        ],
    },
)
