from setuptools import setup

package_name = 'andante_cmd_vel_smoother'

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
    maintainer='ken',
    maintainer_email='kenyoshizoe@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'andante_cmd_vel_smoother = andante_cmd_vel_smoother.andante_cmd_vel_smoother:main'
        ],
    },
)
