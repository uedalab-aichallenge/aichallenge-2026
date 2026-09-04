from setuptools import setup

package_name = 'race_debug_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tendesu',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'race_debug_gui_node = race_debug_gui.gui_node:main',
            'race_debug_viewer = race_debug_gui.viewer:main',
        ],
    },
)
