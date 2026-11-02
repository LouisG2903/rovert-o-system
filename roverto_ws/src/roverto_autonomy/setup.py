from setuptools import find_packages, setup

package_name = 'roverto_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='gomezluis2907',
    maintainer_email='gomezluis2907@gmail.com',
    description='Rovert-O autonomy nodes: UART bridge, wheel odometry and reactive safety guard.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_guard_node = roverto_autonomy.lidar_guard_node:main',
            'sim_esp32_emulator_node = roverto_autonomy.sim_esp32_emulator_node:main',
            'map_compressor_node = roverto_autonomy.map_compressor_node:main',
            'esp32_bridge_node = roverto_autonomy.esp32_bridge_node:main',
        ],
    },
)
