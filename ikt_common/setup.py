from setuptools import find_packages, setup

package_name = 'ikt_common'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Ship the toolkit's packaged default config so ConfigManager can
        # fall back to it when no workspace config / ROBOT_CONFIG_PATH is set.
        ('share/' + package_name + '/config', ['config/toolkit_defaults.yaml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='yizhongzhang',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='Centralized configuration loader and workspace utilities '
                'shared by all packages in the cartesian_controllers_toolkit.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
