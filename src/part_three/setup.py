from setuptools import setup

package_name = 'part_three'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sai',
    maintainer_email='gmanikandan@wpi.edu',
    description='part three',
    license='apache',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	 'service = part_three.velocityConverter:main',
        	 'listener = part_three.velocityController:main',
        	 'talker = part_three.velocityControllerTwo:main',
        ],
    },
)
