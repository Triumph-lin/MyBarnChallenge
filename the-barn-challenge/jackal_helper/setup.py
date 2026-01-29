#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# 从 package.xml 中获取设置
setup_args = generate_distutils_setup(
    packages=['jackal_helper'],
    package_dir={'': 'scripts'},
    # 如果需要安装额外的数据文件
    # package_data={'static_map_pkg': ['config/*.yaml']},
)

setup(**setup_args)