from setuptools import find_packages, setup

package_name = 'yolo_integration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'ultralytics'],
    zip_safe=True,
    maintainer='thomas',
    maintainer_email='thomas@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'yolo_node = yolo_integration.yolo_node:main',
        ],
    },
)
