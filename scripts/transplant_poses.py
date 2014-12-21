#!/usr/bin/python

from __future__ import print_function

import argparse
import sys
from utils import DefaultHelpParser

if __name__ == "__main__":

	parser = DefaultHelpParser(description='Transplant vertex poses from one g2o file to another.')

	parser.add_argument("input", type=argparse.FileType('r'), help = "g2o file to transplant poses to.")
	parser.add_argument("poses", type=argparse.FileType('r'), help = "g2o file to transplant poses from.")
	parser.add_argument("output", type=argparse.FileType('w'), help = "Output g2o file.")

	args = parser.parse_args()

	poses = dict()

	for l in args.poses:
		e = l.split()

		if e[0] == "VERTEX_SE2" or e[0] == "VERTEX_SE3:QUAT":
			poses[ int(e[1]) ] = l


	for l in args.input:
		e = l.split()

		if e[0] == "VERTEX_SE2" or e[0] == "VERTEX_SE3:QUAT":
			if int(e[1]) in poses:
				args.output.write( poses[ int(e[1]) ] )
			else:
				print("ERROR: could not find corresponding vertex %d in reference input!" % int(e[1]))
				exit(1)

		else:
			args.output.write(l)
