#!/usr/bin/env python

import os
import xml.etree.ElementTree as ET 

OUT_FILE = '.dependencies'
IGNORE_FILE = '.dependencyignore'

dir_path = os.path.dirname(os.path.realpath(__file__)) + '/..'
ignore = []
deps = []

with open(IGNORE_FILE, 'r') as f:
    ignore = f.read().splitlines()

for root, dirs, files in os.walk(dir_path):
    for file in files:
        if file == 'package.xml':
                tree = ET.parse(os.path.join(root, file))
                root = tree.getroot()
                for tag in ['depend', 'build_depend', 'build_export_depend', \
                            'exec_depend', 'test_depend', 'buildtool_depend', 'doc_depend']:
                    for dep in root.findall(tag):
                        name = dep.text.strip()
                        if name not in deps and name not in ignore:
                            deps.append(name)

with open(OUT_FILE, 'w') as f:
    for dep in deps:
        f.write('%s\n' % dep)