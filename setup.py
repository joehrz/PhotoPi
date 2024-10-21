# PhotoPi/setup.py

from setuptools import setup, find_packages

# Define dependencies for each component
main_system_deps = [
    'pillow==10.4.0',
    'paramiko==3.1.0',
]

remote_server_deps = [
    'opencv-python',
    'pyyaml',
    'tqdm',
]

raspberry_pi_deps = [
    'RPi.GPIO',
    'adafruit-circuitpython-motorkit',
    'adafruit-circuitpython-motor',
]

point_cloud_analysis_deps = [
    'numpy',
    'open3d',
    'matplotlib',
    'scipy',
]

# Combine all dependencies for the 'all' option
all_deps = (
    main_system_deps +
    remote_server_deps +
    raspberry_pi_deps +
    point_cloud_analysis_deps
)

setup(
    name='PhotoPi',
    version='1.0.0',
    packages=find_packages(),
    include_package_data=True,
    install_requires=[
        # Include project-wide dependencies here, if any
    ],
    extras_require={
        'main_system': main_system_deps,
        'remote_server': remote_server_deps,
        'raspberry_pi': raspberry_pi_deps,
        'point_cloud_analysis': point_cloud_analysis_deps,
        'all': all_deps,  # Allows installation of all components at once
    },
    entry_points={
        'console_scripts': [
            'photopack-main=photopack.main_system.gui:main',
            'photopack-analyze=photopack.point_cloud_analysis.main:main',
            'photopack-turntable=photopack.raspberry_pi.Turntable:main',
        ],
    },
)


