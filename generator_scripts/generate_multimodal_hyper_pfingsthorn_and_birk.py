#!/usr/bin/python

import argparse
import sys
import traceback
from subprocess import check_call
import os


n_mogcomps = [
	[0],
	[1],
	[2],
	[2],
	[4],
	[8],
	[16],
	[32],
	[16,16],
	[8,8,16],
	[4,4,8,16],
	]

n_hypercomps = [
	[1],
	[1],
	[1],
	[2],
	[4],
	[8],
	[16],
	[32],
	[16,16],
	[8,8,16],
	[4,4,8,16],
	]


class DefaultHelpParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write('error: %s\n' % message)
        self.print_help()
        sys.exit(2)


if __name__ == "__main__":

	parser = DefaultHelpParser(description='Generate multimodal motions and hyperedges like in Pfingsthorn and Birk\'s IJRR currently under review paper.')

	parser.add_argument("original", help = "Path to the original, outlier-free, dataset file (in g2o format).")
	parser.add_argument("output_directory", help = "Output directory for outlier files")
	parser.add_argument("--outlier-generator", default=os.path.normpath(os.path.dirname(os.path.realpath(__file__))+"/../bin/outlier_generator"), help="Path to the outlier_generator executable")
	parser.add_argument("--trial", type=int, default=1, help="Number of trial, for seed and name computation.")
	parser.add_argument("--outlier-format", default='%(n_comp)02d_%(trial)03d.outliers', help="Format of outlier files, keys to be used: n_comp, trial. Default: '%%(n_comp)02d_%%(trial)03d.outliers'")
	parser.add_argument("--seed-format", default='1%(n_comp)02d%(trial)03d', help="Like --outlier-format, but used to generate a seed number. Same keys as above. Default: '1%%(n_comp)02d%%(trial)03d'")
	parser.add_argument("--loop-information", type=float, nargs=2, default=[24,24], help="Information values for generated outlier loops. Default: [24,24]")
	parser.add_argument("--motion-information", type=float, nargs=2, default=[42,42], help="Information values for generated outlier motions. Default: [42,42]")



	args = parser.parse_args()

	if not os.path.isfile(args.outlier_generator):
		print "ERROR: path to outlier_generator executable does not exist!"
		exit(1)

	if os.path.exists(args.output_directory) and not os.path.isdir(args.output_directory):
		print "ERROR: output dir exists, but is not a directory!"
		exit(2)
	elif not os.path.exists(args.output_directory):
		os.makedirs(args.output_directory)
	

	if len(n_hypercomps) != len(n_mogcomps):
		exit(3)	

	try:

		for nc in range(0,len(n_hypercomps)):
			cur = {'n_comp': nc, 'trial': args.trial}
			commandline = [args.outlier_generator, args.original, os.path.normpath(args.output_directory + "/" + args.outlier_format % cur ), ]
			commandline += ['--false-loops']
			commandline += [str(x) for x in n_hypercomps[nc]]
			commandline += ['--false-motions']
			commandline += [str(x) for x in n_mogcomps[nc]]
			commandline += ['--seed', args.seed_format % cur ]
			commandline += ['--motion-variance-translation', str(1.0/(args.motion_information[0])), '--motion-variance-rotation', str(1.0/(args.motion_information[1])) ]
			commandline += ['--loop-variance-translation', str(1.0/(args.loop_information[0])), '--loop-variance-rotation', str(1.0/(args.loop_information[1])) ]
			commandline += ['--inflation-factor', "100"]
			if nc > 0:
				prev = cur
				prev['n_comp'] = nc-1
				commandline += ['--previous-outliers', os.path.normpath(args.output_directory + "/" + args.outlier_format % prev )]

			print "*************************"
			print " ".join(commandline)
			check_call(commandline, stdout=sys.stdout, stderr=sys.stderr)
		

	except:
		traceback.print_exc(file=sys.stderr)
		exit(100)
