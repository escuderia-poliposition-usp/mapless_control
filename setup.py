from setuptools import setup
from glob import glob

package_name = 'ebva_mapless'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),  # Add the config directory
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='caio',
    maintainer_email='caio.freitas@usp.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_the_gap = ebva_mapless.follow_the_gap:main',
        ],
    },
)
