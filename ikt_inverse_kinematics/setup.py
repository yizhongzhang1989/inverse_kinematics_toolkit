from setuptools import find_packages, setup

package_name = 'ikt_inverse_kinematics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    # Ship the static web assets (HTML/CSS/JS) inside the Python package so
    # dashboard_node.py can resolve them via Path(__file__).parent / "static"
    # whether installed normally or via `colcon build --symlink-install`.
    package_data={
        package_name: ['static/*.html', 'static/*.css', 'static/*.js',
                       'static/vendor/*.js',
                       'static/vendor/addons/controls/*.js',
                       'static/vendor/addons/loaders/*.js'],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/ik_defaults.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/ik.launch.py',
            'launch/dashboard.launch.py',
            'launch/ik_with_dashboard.launch.py',
        ]),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='yizhongzhang',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='ROS 2 layer over the ikt_core inverse-kinematics solver: a '
                'headless advisory solver node (URDF from file/string/topic, '
                'JSON + typed API), an optional 3D web dashboard, and an RViz '
                'interactive-marker bridge. The solver never commands the robot '
                '(it publishes IK results only). Core math + the Python library '
                'API live in the ikt_core package.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik_node = ikt_inverse_kinematics.ik_node:main',
            'dashboard_node = ikt_inverse_kinematics.dashboard_node:main',
            'marker_node = ikt_inverse_kinematics.marker_node:main',
        ],
    },
)
