import os
from setuptools import setup
from itertools import chain

package_name = 'andante_gazebo'


def add_data_files(data_files):
    data_dirs = ('models', 'worlds', 'launch')
    for path, dirs, files in chain.from_iterable(os.walk(data_dir) for data_dir in data_dirs):
        install_dir = os.path.join('share', package_name, path)
        list_entry = (install_dir, [os.path.join(path, f)
                      for f in files if not f.startswith('.')])
        data_files.append(list_entry)
    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=add_data_files([
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ]),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ken',
    maintainer_email='kenyoshizoe@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
