from setuptools import find_packages, setup

package_name = 'smilei_state_machine'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/smilei_state_machine.launch.py']),
        ('share/' + package_name + '/config', ['config/robot_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luis Tabarez',
    maintainer_email='luistabarez96@gmail.com',
    description='Máquina de estados para el robot SMILEi usando py_trees',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'smilei_state_machine_node = smilei_state_machine.smilei_state_machine_node:main',
        ],
    },
)
