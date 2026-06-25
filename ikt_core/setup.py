from setuptools import find_packages, setup

package_name = 'ikt_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    # Ship the bundled sample URDFs inside the Python package so
    # ``ikt_core.assets`` can resolve them via importlib.resources whether the
    # package is imported from source, pip-installed, or installed into an
    # ament/colcon prefix (incl. --symlink-install).
    package_data={package_name: ['urdf/*.urdf']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', [
            'ikt_core/urdf/planar_3r.urdf',
            'ikt_core/urdf/arm_6dof.urdf',
            'ikt_core/urdf/srs_7dof.urdf',
            'ikt_core/urdf/dual_arm.urdf',
        ]),
    ],
    install_requires=['setuptools', 'numpy'],
    # Pinocchio is the kinematics backend. It is commonly system-provided
    # (apt install python3-pinocchio / ros-<distro>-pinocchio) or conda, so it is
    # NOT a hard install_requires (that would clash with a system build). For a
    # pure pip install use the extra:  pip install "ikt_core[pinocchio]"
    extras_require={'pinocchio': ['pin']},
    zip_safe=True,
    maintainer='yizhongzhang',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='Robot-agnostic, ROS-free inverse-kinematics core (Pinocchio '
                'backend): multi-tip / arbitrary-link / tool-frame pose solving '
                'with per-DOF stiffness, joint limits, singularity robustness, '
                'rest-posture bias, arm-angle redundancy control, reachability '
                'verdict and a dual-arm relative-pose constraint. High-level IK '
                'class + solve_ik() one-liner + a CLI; usable as a plain Python '
                'library (no ROS) or a colcon/ament package.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ikt = ikt_core.cli:main',
        ],
    },
)
