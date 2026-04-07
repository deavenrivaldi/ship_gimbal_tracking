from setuptools import find_packages, setup

package_name = 'ship_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/ship_vision']),
        ('share/ship_vision', ['package.xml']),
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
            'yolo_detection_node = ship_vision.yolo_detection_node:main',
            'pixel_to_angle_node = ship_vision.pixel_to_angle_node:main',
        ],
    },
)
