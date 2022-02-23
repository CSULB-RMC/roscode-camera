from setuptools import setup

package_name = 'rmc_camera'

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
    maintainer='CSULB Lunabotics programming camera subteam',
    maintainer_email='user@todo.todo',
    description='Camera package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_feed = rmc_camera.publisher_feed:main',
            'publisher_object_detection = rmc_camera.publisher_object_detection:main',
            'subscriber_feed = rmc_camera.subscriber_feed:main',
        ],
    },
)
