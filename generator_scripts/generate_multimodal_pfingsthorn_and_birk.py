#!/usr/bin/python

import argparse
import sys
import traceback
from subprocess import check_call
import os


n_mogcomps = [
	[1],
	[2],
	[3],
	[4],
	[8],
	[16],
	[32],
	[0,5],
	[0,0,4],
	[6,5,1],
	[12,10,2],
#	[33,13,4,1], # 63.92
#	[46,32,11,4], # comp 128 (128.0065)
#	[77,57,27,15], # comp 256 (256.1718)
#	[66,44,39,28,17,16,14,12,10,9] # 512 (512.0076)
	]

if __name__ == "__main__":

	parser = DefaultHelpParser(description='Generate multimodal motions like in Pfingsthorn and Birk\'s 2013 IJRR paper.')

	parser.add_argument("original", help = "Path to the original, outlier-free, dataset file (in g2o format).")
	parser.add_argument("output_directory", help = "Output directory for outlier files")
	parser.add_argument("--outlier-generator", default=os.path.normpath(os.path.dirname(os.path.realpath(__file__))+"/../bin/outlier_generator"), help="Path to the outlier_generator executable")
	parser.add_argument("--outlier-format", default='%(n_mogcomp)02d_%(trial)03d.outliers', help="Format of outlier files, keys to be used: n_mogcomp, trial. Default: '%%(n_mogcomp)02d_%%(trial)03d.outliers'")
	parser.add_argument("--seed-format", default='1%(n_mogcomp)02d%(trial)03d', help="Like --outlier-format, but used to generate a seed number. Same keys as above. Default: '1%%(n_mogcomp)02d%%(trial)03d'")
	parser.add_argument("--loop-information", type=float, nargs=2, default=[42,42], help="Information values for generated outlier loops. Default: [42,42]")


	args = parser.parse_args()

	if not os.path.isfile(args.outlier_generator):
		print "ERROR: path to outlier_generator executable does not exist!"
		exit(1)

	if os.path.exists(args.output_directory) and not os.path.isdir(args.output_directory):
		print "ERROR: output dir exists, but is not a directory!"
		exit(2)
	elif not os.path.exists(args.output_directory):
		os.makedirs(args.output_directory)
	
	

	try:

		for no in range(0,len(n_outliers)):
			for np in range(0,len(policies)):
				for nt in range(1,n_trials+1):
					cur = {'policy_index': np, 'n_outliers': n_outliers[no], 'trial': nt }
					commandline = [args.outlier_generator, args.original, os.path.normpath(args.output_directory + "/" + args.outlier_format % cur ), ]
					commandline +=  policies[np]
					commandline += ['--false-loops', str(n_outliers[no]), '--seed', args.seed_format % cur ]
					commandline += ['--loop-variance-translation', str(1.0/(args.loop_information[0])), '--loop-variance-rotation', str(1.0/(args.loop_information[1])) ]
					if no > 0:
						prev = cur
						prev['n_outliers'] = n_outliers[no-1]
						commandline += ['--previous-outliers', os.path.normpath(args.output_directory + "/" + args.outlier_format % prev )]

					print "*************************"
					print " ".join(commandline)
					check_call(commandline, stdout=sys.stdout, stderr=sys.stderr)
		

	except:
		traceback.print_exc(file=sys.stderr)
		exit(100)
