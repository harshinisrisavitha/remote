from setuptools import find_packages, setup

package_name = 'pkg1'

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
    maintainer='harshini',
    maintainer_email='harshini@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub1=pkg1.pub1:main',
            'sub1=pkg1.sub1:main',
            # command to type:pkgName.pythonFileName:FunctionName
        ],
    },
)
