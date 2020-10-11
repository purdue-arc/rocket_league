#!/bin/bash

# SWIG Installation
sudo apt-get install swig

# Download source
cd src
git clone https://github.com/pybox2d/pybox2d

# Build and install
cd pybox2d
python setup.py build
python setup.py install
