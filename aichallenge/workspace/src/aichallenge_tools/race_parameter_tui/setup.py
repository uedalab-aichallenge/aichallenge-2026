from setuptools import setup


package_name = 'race_parameter_tui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AI Challenge team',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'parameter_tui = race_parameter_tui.parameter_tui:main',
        ],
    },
)
