# setup.py
from setuptools import setup, Extension
import pybind11

opencv_include_dir = '/opt/homebrew/opt/opencv@4/include/opencv4'
opencv_lib_dir = '/opt/homebrew/opt/opencv@4/lib'

ext_modules = [
    Extension(
        'MotorDriver',
        ['src/MotorDriver.cpp'],
        include_dirs=[pybind11.get_include()],
        language='c++',
        extra_compile_args=['-std=c++11'],
        extra_link_args=['-std=c++11'],
    ),
    Extension(
        'lidar_draw',
        ['src/lidar_draw.cpp'],
        include_dirs=[pybind11.get_include(), opencv_include_dir],  # OpenCVのインクルードパス
        libraries=['opencv_core', 'opencv_imgproc'],  # 使用するOpenCVライブラリ
        library_dirs=[opencv_lib_dir],  # OpenCVのライブラリディレクトリを指定
        language='c++',
        extra_compile_args=['-std=c++17'],
        extra_link_args=['-std=c++17'],
    ),
]

setup(
    name='MotorDriver',
    version='0.0.1',
    ext_modules=ext_modules,
)
