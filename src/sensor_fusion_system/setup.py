from setuptools import find_packages, setup

package_name = 'sensor_fusion_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mtahr',
    maintainer_email='mtahr@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sensor_publisher = sensor_fusion_system.sensor_publisher:main',
            'data_logger = sensor_fusion_system.data_logger:main',
            'alert_system = sensor_fusion_system.alert_system:main',
            'sensor_fusion = sensor_fusion_system.sensor_fusion:main',
       
        ],
    },
)
