from setuptools import find_packages, setup

package_name = 'rocket_launch_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
    ],
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: Apache Software License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6',
    zip_safe=True,
    maintainer='lrossett, dougsland, dwalsh',
    maintainer_email='lrossett@redhat.com, dougsland@redhat.com, dwalsh@redhat.com',
    description='A Rocket Launch Simulator',
    license='Apache License 2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
