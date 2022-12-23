from setuptools import setup

package_name = 'andante_camera_linetrace'

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
            'andante_camera_linetrace = andante_camera_linetrace.andante_camera_linetrace:main',
            'calibration = andante_camera_linetrace.calibration:main',
            'generate_marker = andante_camera_linetrace.generate_marker:main'
        ],
    },
)
