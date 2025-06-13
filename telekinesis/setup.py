from setuptools import find_packages, setup

package_name = 'telekinesis'

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
    maintainer='keshaw',
    maintainer_email='lucky7chess@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leap_v2_ik = telekinesis.leap_v2_ik:main',
            'leap_ik = telekinesis.leap_ik:main',
            'mcp_test = telekinesis.mcp_test:main',
            'palm_test = telekinesis.palm_test:main',
            'finger_test = telekinesis.finger_test:main',
            'durability_test = telekinesis.durability_test:main'
        ],
    },
)
