#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2015-2018 German Aerospace Center (DLR) and others.
# This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# which accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v20.html
# SPDX-License-Identifier: EPL-2.0

# @file    typemap.py
# @author  Michael Behrisch
# @date    2015-07-06
# @version $Id$

"""
This script rebuilds "src/netimport/typemap.h" and "src/polyconvert/pc_typemap.h", the files
representing the default typemaps.
It does this by parsing the data from the sumo data dir.
"""

from __future__ import print_function
from __future__ import absolute_import
from os.path import dirname, exists, getmtime, join


def writeTypeMap(typemapFile, typemap):
    with open(typemapFile, 'w') as f:
        for format, mapFile in sorted(typemap.items()):
            print("const std::string %sTypemap =" % format, file=f)
            for line in open(mapFile):
                print('"%s"' %
                      line.replace('"', r'\"').replace('\n', r'\n'), file=f)
            print(";", file=f)


def generateTypeMap(relPath, formats, suffix):
    typemapDataDir = join(dirname(__file__), '..', '..', 'data', 'typemap')
    typemapFile = join(dirname(__file__), '..', '..', 'src', relPath)
    typemap = {}
    maxTime = 0
    for format in formats:
        typemap[format] = join(typemapDataDir, format + suffix)
        if exists(typemap[format]):
            maxTime = max(maxTime, getmtime(typemap[format]))
    if not exists(typemapFile) or maxTime > getmtime(typemapFile):
        writeTypeMap(typemapFile, typemap)


if __name__ == "__main__":
    generateTypeMap(join('netimport', 'typemap.h'), ("opendrive", "osm"), "Netconvert.typ.xml")
    generateTypeMap(join('polyconvert', 'pc_typemap.h'), ("navteq", "osm", "visum"), "Polyconvert.typ.xml")
