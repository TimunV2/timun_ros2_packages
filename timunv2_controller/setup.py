from setuptools import find_packages, setup

package_name = 'timunv2_controller'

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
            "joy_sub_node = timunv2_controller.joy_sub_node:main",
            "master_controller = timunv2_controller.master_controller:main"
        ],
    },
)
