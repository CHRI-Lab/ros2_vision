from setuptools import find_packages, setup

package_name = 'depth_estimator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name + '/realsense_depth.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jackson',
    maintainer_email='renweih@student.unimelb.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "realsense_depth_estimator = depth_estimator.realsense_depth_estimator:main",
        ],
    },
)
