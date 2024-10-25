from setuptools import find_packages, setup

package_name = 'balise_detector'

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
    maintainer='little',
    maintainer_email='delgorguealexis@gmail.com',
    description='This package contains the file to detect balises and find the position of the robot thanks to the signal coming from the lidar',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_balises = balise_detector.detect_balises:main'
        ],
    },
)
