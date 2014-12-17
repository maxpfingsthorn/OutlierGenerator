#!/usr/bin/python

import argparse
import sys


if __name__ == "__main__":

	parser = argparse.ArgumentParser(description='Transplant vertex poses from one g2o file to another.')

	parser.add_argument("input", type=argparse.FileType('r'), help = "g2o file to transplant poses to.")
	parser.add_argument("poses", type=argparse.FileType('r'), help = "g2o file to transplant poses from.")
	parser.add_argument("output", type=argparse.FileType('w'), help = "Output g2o file.")

	args = parser.parse_args()

	poses = dict()

	pl = args.poses.readlines()
	for l in pl:
		e = l.split()

		if e[0] == "VERTEX_SE2" or e[0] == "VERTEX_SE3:QUAT":
			poses[ int(e[1]) ] = e


	lines = args.input.readlines()

	for l in lines:
		e = l.split()

		if e[0] == "VERTEX_SE2" or e[0] == "VERTEX_SE3:QUAT":
			if int(e[1]) in poses:
				e[2:] = poses[ int(e[1]) ][2:]

			args.output.write( " ".join(e) + "\n")

		else:
			args.output.write(l)
