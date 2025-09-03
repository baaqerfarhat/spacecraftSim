
from setuptools import setup



package_name = 'openvins_bridge'



setup(

    name=package_name,

    version='0.0.1',

    packages=[package_name],

    data_files=[

        ('share/ament_index/resource_index/packages',

            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='root',

    maintainer_email='root@todo.todo',

    description='Bridge for VINS-Fusion integration',

    license='TODO: License declaration',

    tests_require=['pytest'],

    entry_points={

        'console_scripts': [

            'ov_sync_repub = openvins_bridge.sync_repub:main',

        ],

    },

)

