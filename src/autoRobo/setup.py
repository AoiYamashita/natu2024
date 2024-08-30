from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autoRobo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name),glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yamashita',
    maintainer_email='yamashitaaoi1230@icloud.com',
    description='auto robot in natuRobo 2024',
    license='BSD-3-Clouse',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ImgProcess = autoRobo.ImgProcess:main',
            'Localization = autoRobo.localization:main',
            'serial = autoRobo.serial:main',
            'operater = autoRobo.operater:main'
        ],
    },
)
