from setuptools import setup

package_name = 'to_pallete_and_beyond'

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
    maintainer='rahul',
    maintainer_email='mishrarahul1997@yahoo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pallete_segment = to_pallete_and_beyond.pallete_segment:main',
            'pallete_detect = to_pallete_and_beyond.pallete_detect:main',
            'simulated_camera = to_pallete_and_beyond.simulated_camera:main'
        ],
    },
)
