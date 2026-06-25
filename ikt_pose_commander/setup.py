from setuptools import find_packages, setup

package_name = 'ikt_pose_commander'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    # Ship the dashboard's static web assets (HTML/CSS/JS + the Three.js vendor
    # bundle for the 3D viewer) inside the package so dashboard_node.py can
    # resolve them via Path(__file__).parent / "static" whether installed
    # normally or via `colcon build --symlink-install`.
    package_data={
        package_name: [
            'static/*.html', 'static/*.css', 'static/*.js',
            'static/vendor/*.js',
            'static/vendor/addons/controls/*.js',
            'static/vendor/addons/loaders/*.js',
        ],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/commander_defaults.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/commander.launch.py',
            'launch/dashboard.launch.py',
        ]),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='yizhongzhang',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='Accepts a Cartesian target pose on a topic, solves it with the '
                'ikt_inverse_kinematics solver, and commands the arm via its '
                'JointTrajectoryController (safe, default) or forward_position_'
                'controller (streaming). Safety-gated: starts disabled, rejects '
                'unreachable / large-jump solutions, limits speed.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander_node = ikt_pose_commander.commander_node:main',
            'dashboard_node = ikt_pose_commander.dashboard_node:main',
            'send_pose = ikt_pose_commander.send_pose:main',
        ],
    },
)
