import os

from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'duo_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deadcat',
    maintainer_email='artem.nkfv@gmail.com',
    description='Package to visualize Duo robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = duo_description.state_publisher:main'
        ],
    },
)
