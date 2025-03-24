from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'path_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vulcan',
    maintainer_email='jinxu.yu@directdrivetech.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_controller_node = path_controller.path_controller_node:main',
            'path_pub_node = path_controller.pub_path:main',
        ],
    },
)
