#!/usr/bin/env python
import sys

from setuptools import setup
from setuptools_rust import RustExtension

setup(
    name="torcs-gym",
    version="0.1.0",
    classifiers=[
        "Operating System :: POSIX",
        "Operating System :: MacOS :: MacOS X",
        "Programming Language :: Rust",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Intended Audience :: Science/Research",
        "Development Status :: 3 - Alpha",
    ],
    packges=["torcs-gym"],
    rust_extensions=[RustExtension("torcs-gym.torcs-gym")],
    include_package_data=True,
    zip_safe=False,
)
