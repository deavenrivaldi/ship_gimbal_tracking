from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ship_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/ship_simulation']),
        ('share/ship_simulation', ['package.xml']),
        ('share/ship_simulation/worlds', glob('worlds/*.sdf')),
        ('share/ship_simulation/models', glob('models/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linzhensheng',
    maintainer_email='youngbillionairesid@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
