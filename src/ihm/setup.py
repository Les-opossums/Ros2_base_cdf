from setuptools import find_packages, setup

package_name = 'ihm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',         
        'rcl_interfaces',    
    ],
    zip_safe=True,
    maintainer='greg',
    maintainer_email='gregoire.husser17@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ihm_node = ihm.ihm_node:main', 
            'parameters_server = ihm.parameters_server:main', 
        ],
    },
)
