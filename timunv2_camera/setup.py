from setuptools import find_packages, setup

package_name = 'timunv2_camera'

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
    maintainer='hasan',
    maintainer_email='hasan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "camera_publisher_node = timunv2_camera.camera_publisher_node:main",
            "camera_subscriber_node = timunv2_camera.camera_subscriber_node:main",
            "gstreamer_publisher_node = timunv2_camera.gstreamer_publisher_node:main"
        ],
    },
)
