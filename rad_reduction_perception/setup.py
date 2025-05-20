from setuptools import find_packages, setup
import os
import glob

package_name = 'rad_reduction_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install *all* launch files automatically
        (os.path.join('share', package_name, 'launch'),
         glob.glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sophiewalters3',
    maintainer_email='smw59@uw.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    "console_scripts": [
        "my_subscriber   = rad_reduction_perception.my_subscriber:main",
        "publisher       = rad_reduction_perception.publisher:main",
        "test_publisher  = rad_reduction_perception.test_publisher:main",
        "marker_to_robot = rad_reduction_perception.marker_to_robot:main",
    ],
},

)
