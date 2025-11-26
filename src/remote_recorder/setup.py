from setuptools import find_packages, setup

package_name = 'remote_recorder'

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
    maintainer='gandalf',
    maintainer_email='felix-max-jakob.mayer@student.uni-tuebingen.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'recorder_server = remote_recorder.recorder_server:main',
        ],
    },
)
