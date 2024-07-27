from setuptools import find_packages, setup

package_name = 'sas_robot_driver'

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
    maintainer='qlin',
    maintainer_email='qlin1806@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'sas_robot_driver_example_robot = sas_robot_driver.sas_robot_driver_example_robot:main'
        ],
    },
)
