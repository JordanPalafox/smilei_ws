from setuptools import setup

package_name = 'smilei_debug_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rovestrada',
    maintainer_email='rovestrada@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'velocity_publisher = smilei_debug_tools.velocity_publisher:main',
            'motor_data_publisher = smilei_debug_tools.motor_data_publisher:main',
        ],
    },
)
