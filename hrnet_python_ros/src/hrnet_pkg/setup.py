from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'hrnet_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib/python3.8/site-packages', package_name, 'hrnet'), glob('hrnet_pkg/hrnet/**')),
        (os.path.join('lib/python3.8/site-packages', package_name, 'lib'), glob('hrnet_pkg/lib/**')),
        (os.path.join('lib/python3.8/site-packages', package_name, 'model'), glob('hrnet_pkg/model/**'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edgeboard',
    maintainer_email='edgeboard@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
     entry_points={
        'console_scripts': [
        'Hrnet_pub = hrnet_pkg.infer_demo_vedio:main',
        'turtle_controller = hrnet_pkg.control_topic:main'

        ],
    },
)
