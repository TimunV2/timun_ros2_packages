from setuptools import find_packages, setup

package_name = 'timunv2_pipefollowing'

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
    maintainer_email='hasandzulfadli@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pipeline_detect_node = timunv2_pipefollowing.pipeline_detect:main",
            "pipeline_navigation_node = timunv2_pipefollowing.pipeline_navigation:main"
        ],
    },
)
