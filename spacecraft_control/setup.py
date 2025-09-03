from setuptools import setup

package_name = 'spacecraft_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@domain.com',
    description='Spacecraft control package for SRB',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manual_controller = spacecraft_control.manual_controller:main',
            'simple_movement = spacecraft_control.simple_movement:main',
        ],
    },
)
