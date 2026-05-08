from setuptools import find_packages, setup

package_name = 'jiho1'

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
    maintainer='parkjiho',
    maintainer_email='parkjiho@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'jiho1_node = jiho1.jiho1_node:main',
            'publisher = jiho1.publisher:main',
            'subscriber = jiho1.subscriber:main',
        ],
    },
)
