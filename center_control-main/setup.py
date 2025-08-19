from setuptools import setup

package_name = 'center_control'

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
    maintainer='mmr',
    maintainer_email='mmr@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'translate = center_control.translate:main',
		'control = center_control.control:main',
		'arm_auto = center_control.arm_auto:main',	
        ],
    },
)
