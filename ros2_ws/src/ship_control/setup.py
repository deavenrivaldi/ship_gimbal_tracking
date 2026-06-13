from setuptools import find_packages, setup

package_name = 'ship_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/ship_control']),
        ('share/ship_control', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linzhensheng',
    maintainer_email='youngbillionairesid@gmail.com',
    description='Gimbal control — stabilization and RL training',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # ── Active nodes ──────────────────────────────────────────
            'imu_stabilizer_node    = ship_control.imu_stabilizer_node:main',
            'gimbal_stabilizer_node = ship_control.gimbal_stabilizer_node:main',
            'fire_node              = ship_control.fire_node:main',
        ],
    },
)