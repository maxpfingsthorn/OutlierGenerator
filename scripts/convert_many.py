#!/usr/bin/python

import argparse
import sys
import os
import glob
import traceback
from subprocess import check_call
from utils import DefaultHelpParser

if __name__ == "__main__":

	parser = DefaultHelpParser(description='Convert many outlier files with a single original dataset with any of the one-shot converters. Additional arguments are passed on to worker conversion script.')

	parser.add_argument("worker", help="Script to use for conversion")
	parser.add_argument("input", help = "Path to the original dataset file (in g2o format).")
	parser.add_argument("outliers", nargs="+", help = "Filenames or glob patterns matching outlier files.")
	parser.add_argument("--output-dir", help="Optional output directory other than the directory containing outliers")
	parser.add_argument("--output-prefix", help="Optional prefix that is prepended to the output file name")
	parser.add_argument("--output-suffix", help="Optional suffix that is prepended to the output file name")
	

	(args, extra_args) = parser.parse_known_args()

	print "Extra args to be passed on are: ", extra_args

	if not os.path.exists(args.worker) or not os.path.exists(args.input):
		print "ERROR: worker script or input file does not exist!"
		exit(1)

	args.worker = os.path.realpath(args.worker)

	if args.output_dir and os.path.exists(args.output_dir) and not os.path.isdir(args.output_dir):
		print "ERROR: output dir is not a dir"
		exit(3)

	if args.output_dir and not os.path.exists(args.output_dir):
		os.makedirs(args.output_dir)

	for pattern in args.outliers:
		for outlierfile in glob.glob(pattern):
			if not os.path.exists(outlierfile):
				print "ERROR: outlier file '",outlierfile,"does not exist!"
				continue

			command = [args.worker, args.input, outlierfile]

			

			(output_name, outlier_ext) = os.path.splitext( os.path.basename(outlierfile) )

			if args.output_prefix:
				output_name = args.output_prefix + output_name

			if args.output_suffix:
				output_name += args.output_suffix


			output_name = os.path.normpath(args.output_dir + "/" + output_name + ".g2o")

			command += [output_name]

			command += extra_args

			print "*************************"
			print " ".join(command)
			check_call(command, stdout=sys.stdout, stderr=sys.stderr)