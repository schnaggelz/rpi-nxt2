from setuptools import find_packages, setup

package_name = 'py_nxt_test'

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
    maintainer='Schnaggelz',
    maintainer_email='schnaggelz@gmx.de',
    description='NXT driver node tests in Python',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_command_test = py_nxt_test.motor_command_test:main',
        ],
    },
)
