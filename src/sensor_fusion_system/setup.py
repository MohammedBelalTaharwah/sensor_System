import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sensor_fusion_system'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    
    # هذا القسم هو الذي يخبر colcon بنسخ ملفات الإعدادات والتشغيل
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # لنسخ جميع ملفات الـ launch
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.launch.py'))),
        
        # لنسخ جميع ملفات الإعدادات (config)
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.yaml'))),
    ],
    
    # الاعتماديات (المكتبات) التي تحتاجها النُوَد للعمل
    install_requires=[
        'setuptools',
        'rclpy',
        'numpy',
        'matplotlib',
        'sensor_msgs',
        'std_msgs',
        'geometry_msgs',
        'diagnostic_msgs',
    ],
    zip_safe=True,
    maintainer='mtahr',
    maintainer_email='mtahr@todo.todo',
    description='A ROS 2 package for sensor fusion simulation, logging, and alerting.',
    license='Apache 2.0', # تم تغييرها من "TODO"
    
    # متطلبات الاختبارات
    tests_require=['pytest'],
    
    # "نقاط الدخول" التي تربط الأوامر بملفات البايثون
    entry_points={
        'console_scripts': [
            # هذه الأسماء تطابق ملفاتك (مثل sensor_publisher.py)
            'sensor_publisher = sensor_fusion_system.sensor_publisher:main',
            'data_logger = sensor_fusion_system.data_logger:main',
            'alert_system = sensor_fusion_system.alert_system:main',
            'sensor_fusion = sensor_fusion_system.sensor_fusion:main',
            
            # هذا النود الخامس مطلوب بواسطة ملف التشغيل (launch file)
            # نفترض أن اسم الملف هو 'visualization.py'
            'visualizer = sensor_fusion_system.visualization:main',
        ],
    },
)