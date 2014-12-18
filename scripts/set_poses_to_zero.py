#!/usr/bin/python

import argparse
import sys

class DefaultHelpParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write('error: %s\n' % message)
        self.print_help()
        sys.exit(2)
        
if __name__ == "__main__":

	parser = DefaultHelpParser(description='Set vertex poses to zeros.')

	parser.add_argument("input", type=argparse.FileType('r'), help = "Path to the original dataset file (in g2o format).")
	parser.add_argument("output", type=argparse.FileType('w'), help = "Output g2o file.")

	args = parser.parse_args()

	lines = args.input.readlines()

	for l in lines:
		e = l.split()

		if e[0] == "VERTEX_SE2" or e[0] == "VERTEX_SE3:QUAT":
			for k in range(2,len(e)):
				e[k] = "0"

			if e[0] == "VERTEX_SE3:QUAT":
				e[-1] = "1" # make sure the quaternion is valid

			args.output.write( " ".join(e) + "\n")

		else:
			args.output.write(l)
