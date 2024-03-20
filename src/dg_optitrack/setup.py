from setuptools import setup

package_name = 'dg_optitrack'

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
    maintainer='sofia',
    maintainer_email='zazakwok@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'subscriber = dg_optitrack.subscriber:main',
                'publisher = dg_optitrack.publisher:main',
        ],
},
)
