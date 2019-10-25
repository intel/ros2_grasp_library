from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'handeye_dashboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages('src', exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        ('share/' + package_name + '/images', glob('images/*.png')),
        ('share/' + package_name + '/data', glob('data/*.json')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.perspective')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Yu Yan',
    author_email='yu.yan@intel.com',
    maintainer='Yu Yan',
    maintainer_email='yu.yan@intel.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD 3-Clause License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'The handeye_dashboard package.'
    ),
    license='Apache License 2.0',
    tests_require=['pytest'],
    package_dir={'':'src'},
    entry_points={
        'console_scripts': [
            'handeye_dashboard = handeye_dashboard.main:main',
        ],
    },
)
