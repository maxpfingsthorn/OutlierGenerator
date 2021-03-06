#!/usr/bin/python

from __future__ import print_function

import argparse
import sys
import graph as G
from utils import DefaultHelpParser


if __name__ == "__main__":

	parser = DefaultHelpParser(description='Read g2o file set poses to zero.')

	parser.add_argument("input", type=argparse.FileType('r'), help = "Path to the original dataset file (in g2o format).")
	parser.add_argument("output", type=argparse.FileType('w'), help = "Output graph will be written into this file.")

	args = parser.parse_args()


	g = G.readg2o(args.input)

	g.setNonfixedPosesToZero()

	g.writeg2o(args.output)