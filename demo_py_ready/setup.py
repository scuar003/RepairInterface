from setuptools import find_packages, setup

package_name = 'demo_py_ready'

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
    maintainer='robotics',
    maintainer_email='scuar003@fiu.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'surface_detection=demo_py_ready.surface_detection:main',
            'repair_executer = demo_py_ready.repair_executer:main',
            'repair_grind = demo_py_ready.repair_grind:main',
            'repair_pipe = demo_py_ready.repair_pipe:main',
        ],
    },
)
