from setuptools import find_packages, setup

package_name = 'warmup_project'

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
    maintainer='joloujo',
    maintainer_email='joloujo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_test = warmup_project.drive_test:main',
            'drive_square = warmup_project.drive_square:main',
            'teleop = warmup_project.teleop:main',
            'follow_wall = warmup_project.follow_wall:main',
            'follow_person = warmup_project.follow_person:main',
        ],
    },
)
