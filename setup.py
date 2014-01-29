import os
from setuptools import setup, find_packages


def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

version = "1.0.0"


setup(
    name='rosstream2boot',
    author="Andrea Censi",
    author_email="andrea@cds.caltech.edu",
    url='http://github.com/AndreaCensi/rosstream2boot',
    version=version,

    description="",

    long_description=read('README.rst'),
    keywords="",
    license="LGPL",

    classifiers=[
        'Development Status :: 5 - Production/Stable',
        'Intended Audience :: Developers',
        'Intended Audience :: Science/Research',
        'Topic :: Scientific/Engineering',
        'License :: OSI Approved :: GNU Library or '
        'Lesser General Public License (LGPL)',
    ],

    package_dir={'':'src'},
    packages=find_packages('src'),
    entry_points={
     'console_scripts': [
       'rs2b = rosstream2boot.programs:main_rs2b',
      ]
    },
    install_requires=[
        'BootOlympics', 'QuickApp', 'ROSBagUtils'
    ],

    tests_require=['nose']
)

