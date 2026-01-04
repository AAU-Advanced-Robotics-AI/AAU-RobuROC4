from setuptools import find_packages, setup

package_name = 'roburoc_imu_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roburoc',
    maintainer_email='tssa20@student.aau.dk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roburoc_imu_publisher = roburoc_imu_publisher.roburoc_imu_publisher_node:main'
        ],
    },
)
