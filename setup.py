import os
import sys
DIR = os.path.abspath(os.path.dirname(__file__))
sys.path.append(os.path.join(DIR, 'external', 'pybind11'))

from glob import glob
from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension, build_ext

del sys.path[-1]

ext_modules = [
    Pybind11Extension('_rqtreemesh',
        include_dirs = ['./src'],
        sources = sorted(glob('src/*.c*')),
        define_macros = [('_rqtreemesh', '_rqtreemesh')],
		extra_compile_args = ['-O2', '-Wall'],
        ),
]

setup(
    name='rqtreemesh',
    version='1.0.0',
    install_requires=['numpy', 'pillow'],
    python_requires='>=3.9',
    author='Philip Schall',
    author_email='philip.schall@live.com',
    description='An implementation of restricted quadtree triangulation (RQT), for converting raster heightmaps to 3D meshes.',
    long_description_content_type='text/markdown',
    long_description=open('README.md').read(),
    url='https://github.com/philipschall/rqtreemesh',
    packages=['rqtreemesh'],
    license='Unlicense',
    cmdclass={"build_ext": build_ext},
    ext_modules=ext_modules,
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'License :: Public Domain',
        'Operating System :: Microsoft :: Windows',
        'Operating System :: Unix',
        'Operating System :: MacOS',
        'Programming Language :: C++',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Programming Language :: Python :: 3 :: Only',
    ],
)