from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'projectile_gz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # models SDF + world files
        (os.path.join('share', package_name, 'models'),
            glob('models/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tom',
    maintainer_email='tom@todo.todo',
    description='Gazebo interface for projectile system',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          'gz_interface = projectile_gz.gz_interface:main',
        ],
    },
)
