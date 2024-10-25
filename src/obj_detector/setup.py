from setuptools import find_packages, setup, SetuptoolsDeprecationWarning
import warnings

warnings.filterwarnings("ignore", category=SetuptoolsDeprecationWarning)

package_name = 'obj_detector'

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
    maintainer='greg',
    maintainer_email='gregoire.husser@etu.inp-n7.fr',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_cluster = obj_detector.node_cluster:main',
            'wall_localisation = obj_detector.wall_localisation:main',
        ],
    },
)
