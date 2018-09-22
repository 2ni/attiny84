#!/usr/bin/env python

'''
Activates a given file, ie links modules/<file> to src/<file> and removes existing link

https://stackoverflow.com/questions/26727314/multiple-files-for-one-argument-in-argparse-python-2-7
https://stackoverflow.com/questions/23766689/python-argparse-arg-with-no-flag
'''

import os, re, argparse
from serial.tools import list_ports

parser = argparse.ArgumentParser(description='Activate given files', formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('files', metavar='file', type=argparse.FileType('r'), nargs='*', help='Files to activate')

args = parser.parse_args()

# be sure to be in dir of script
os.chdir(os.path.dirname(os.path.realpath(__file__)))

# unlink any existing file
os.system('find src/ -type l -delete')

# link each given file in src
for f in args.files:
    os.system('ln -s ../{f} src/'.format(f=f.name))
