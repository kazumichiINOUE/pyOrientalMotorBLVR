# setup.py
from setuptools import setup, Extension
import pybind11

ext_modules = [
    Extension(
        'MotorDriver',
        ['MotorDriver.cpp'],
        include_dirs=[pybind11.get_include()],
        language='c++',
        extra_compile_args=['-std=c++11'],
        extra_link_args=['-std=c++11'],
    ),
]

setup(
    name='MotorDriver',
    version='0.0.1',
    ext_modules=ext_modules,
)
