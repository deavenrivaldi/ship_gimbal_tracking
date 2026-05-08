from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ship_simulation'

# Helper function to find all files in a directory tree
def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            # This creates a tuple: (destination_path, [source_file])
            paths.append((os.path.join('share', package_name, path), [os.path.join(path, filename)]))
    return paths

# Get all files from models and worlds
extra_files = package_files('models')
extra_files += package_files('worlds')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/ship_simulation']),
        ('share/ship_simulation', ['package.xml']),
    ] + extra_files,
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
