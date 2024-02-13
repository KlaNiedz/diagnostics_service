from setuptools import find_packages, setup

package_name = 'diagnostics_analyzer'

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
    maintainer='klaudia',
    maintainer_email='klaudianiedzi@gmail.com',
    description='Battery analyzer',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'my_publisher = diagnostics_analyzer.my_publisher:main',
        ],
    },
)
