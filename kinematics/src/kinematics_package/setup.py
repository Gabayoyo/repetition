from setuptools import find_packages, setup

package_name = 'kinematics_package'

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
    maintainer='currie04',
    maintainer_email='orlenamay2204@hotmail.com',
    description='Kinematics package for modified px100 arm',
    license='OSC License 1.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'my_node = kinematics_package.my_node:main'
        ],
    },
)
