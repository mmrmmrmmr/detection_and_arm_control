from setuptools import setup

package_name = 'deep_with_arm'

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
		'camera_3d_publisher = deep_with_arm.camera_3d_publisher:main',
		'pos_camera_to_arm = deep_with_arm.pos_camera_to_arm:main',
		'trans_node = deep_with_arm.trans_node:main',
        ],
    },
)
