from setuptools import find_packages
from setuptools import setup

package_name = 'launch_gz'

setup(
    name=package_name,
    version='0.246.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    package_data={'launch_gz': ['py.typed']},
    install_requires=[
        'setuptools',
        'ament_index_python',
        'launch',
        'osrf_pycommon',
        'pyyaml',
    ],
    zip_safe=True,
    author='Carlos Agüero',
    author_email='caguero@openrobotics.org',
    maintainer='Carlos, Alejandro, Michael',
    maintainer_email='aditya.pande@openrobotics.org, brandon@openrobotics.org',
    url='https://github.com/gazebosim/ros_gz/launch_gz',
    download_url='https://github.com/gazebosim/ros_gz/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='gz specific extensions to `launch`.',
    long_description=(
        'This package provides gz specific extensions to the launch package.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': [
            'launch_gz = launch_gz',
        ],
    }
)