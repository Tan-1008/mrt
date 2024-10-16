from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sauron'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'pubsub.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tan',
    maintainer_email='tan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher1 = sauron.talker1:main',
            'publisher2 = sauron.talker2:main',
            'subscriber = sauron.listener:main',
        ],
    },
)
