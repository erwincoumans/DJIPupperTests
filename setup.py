from setuptools import find_packages
from sys import platform as _platform
import sys
import glob
import os

from distutils.core import setup
from distutils.extension import Extension
from distutils.util import get_platform
from glob import glob

# monkey-patch for parallel compilation
import multiprocessing
import multiprocessing.pool


def parallelCCompile(self,
                     sources,
                     output_dir=None,
                     macros=None,
                     include_dirs=None,
                     debug=0,
                     extra_preargs=None,
                     extra_postargs=None,
                     depends=None):
  # those lines are copied from distutils.ccompiler.CCompiler directly
  macros, objects, extra_postargs, pp_opts, build = self._setup_compile(
      output_dir, macros, include_dirs, sources, depends, extra_postargs)
  cc_args = self._get_cc_args(pp_opts, debug, extra_preargs)
  # parallel code
  N = 2 * multiprocessing.cpu_count()  # number of parallel compilations
  try:
    # On Unix-like platforms attempt to obtain the total memory in the
    # machine and limit the number of parallel jobs to the number of Gbs
    # of RAM (to avoid killing smaller platforms like the Pi)
    mem = os.sysconf('SC_PHYS_PAGES') * os.sysconf('SC_PAGE_SIZE')  # bytes
  except (AttributeError, ValueError):
    # Couldn't query RAM; don't limit parallelism (it's probably a well
    # equipped Windows / Mac OS X box)
    pass
  else:
    mem = max(1, int(round(mem / 1024**3)))  # convert to Gb
    N = min(mem, N)

  def _single_compile(obj):
    try:
      src, ext = build[obj]
    except KeyError:
      return
    newcc_args = cc_args
    if _platform == "darwin":
      if src.endswith('.cpp'):
        newcc_args = cc_args + ["-std=c++17", "-stdlib=libc++", "-stdlib=libc++"]
    self._compile(obj, src, ext, newcc_args, extra_postargs, pp_opts)

  # convert to list, imap is evaluated on-demand
  pool = multiprocessing.pool.ThreadPool(N)
  list(pool.imap(_single_compile, objects))
  return objects


import distutils.ccompiler
distutils.ccompiler.CCompiler.compile = parallelCCompile

#see http://stackoverflow.com/a/8719066/295157
import os

platform = get_platform()
print(platform)

CXX_FLAGS = ''
CXX_FLAGS += '-DUSE_SIM '


# libraries += [current_python]

libraries = []
include_dirs = []

try:
  import numpy
  NP_DIRS = [numpy.get_include()]
except:
  print("numpy is disabled. getCameraImage maybe slower.")
else:
  print("numpy is enabled.")
  CXX_FLAGS += '-DPYBULLET_USE_NUMPY '
  for d in NP_DIRS:
    print("numpy_include_dirs = %s" % d)
  include_dirs += NP_DIRS

sources = ["src/PupperDrive.cpp",
           "src/Kinematics.cpp",  
           "src/DriveSystem.cpp",
           "src/PID.cpp",
           "src/Utils.cpp",
          ]

if _platform == "linux" or _platform == "linux2":
  print("linux!")
elif _platform == "win32":
  print("win32!")
elif _platform == "darwin":
  print("darwin!")
else:
  print("bsd!")

setup_py_dir = os.path.dirname(os.path.realpath(__file__))

need_files = []
datadir = "python"

hh = setup_py_dir + "/" + datadir

for root, dirs, files in os.walk(hh):
  for fn in files:
    ext = os.path.splitext(fn)[1][1:]
    if ext and ext in 'yaml index meta data-00000-of-00001 png gif jpg urdf sdf obj txt mtl dae off stl STL xml gin npy '.split(
    ):
      fn = root + "/" + fn
      need_files.append(fn[1 + len(hh):])

print("found resource files: %i" % len(need_files))
for n in need_files:
  print("-- %s" % n)
print("packages")
print(find_packages('python'))
print("-----")

extensions = []

pupper_drive_ext = Extension(
    "pupper_drive",
    sources=sources,
    libraries=libraries,
    extra_compile_args=CXX_FLAGS.split(),
    include_dirs=include_dirs + [
        "src", "third_party",
    ])
extensions.append(pupper_drive_ext)


setup(
    name='pupper_drive',
    version='0.1',
    description=
    'Python bindings for Puppet Teensy firmware, for simulation',
    long_description=
    'Python bindings for Puppet Teensy firmware, for simulation',
    url='https://github.com/erwincoumans/DJIPupperTests',
    author='Stanford team',
    author_email='',
    license='zlib',
    platforms='any',
    keywords=[
        'physics simulation', 'robotics',
    ],
    ext_modules=extensions,
    classifiers=[
        'Development Status :: 5 - Production/Stable',
        'License :: OSI Approved :: zlib/libpng License',
        'Operating System :: Microsoft :: Windows', 'Operating System :: POSIX :: Linux',
        'Operating System :: MacOS', 'Intended Audience :: Science/Research',
        "Programming Language :: Python", 'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3.4', 'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6', 'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8', 'Topic :: Games/Entertainment :: Simulation',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Framework :: Robot Framework'
    ],
    package_dir={'': 'python'},
    packages=[x for x in find_packages('python')],
    package_data={'pybullet_data': need_files})
