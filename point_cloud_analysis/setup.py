# PhotoPi/setup.py

import setuptools
import os

# Read the contents of requirements.txt
with open("requirements.txt", "r") as fh:
    requirements = fh.read().splitlines()

# Read the contents of README.md (if it exists)
if os.path.exists("README.md"):
    with open("README.md", "r", encoding="utf-8") as fh:
        long_description = fh.read()
else:
    long_description = ''

setuptools.setup(
    name="PhotoPi",
    version="0.1.0",
    author="Joe Hrzich",
    author_email="hrzich-j@webmail.uwinnipeg.ca",
    description="A point cloud analysis tool for plant measurements",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/joehrz/PhotoPi",  # Replace with your repository URL
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
    install_requires=requirements,
    entry_points={
        'console_scripts': [
            'photopi=point_cloud_analysis.main:main',
        ],
    },
    include_package_data=True,
)
