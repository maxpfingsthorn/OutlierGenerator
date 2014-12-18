#!/usr/bin/python

import argparse
import sys
import traceback
from subprocess import check_call
import os

# as in Niko Suenderhauf's thesis, generate a number of outlier loops, to up 1000.
# #outlier schedule (as per section 6.5 of his thesis) is: 50, 100, 200, 300, 400, 500, 750, 10000
# for each of the four policies ( {nongrouped, grouped}x{random, local})
# times 10 trials
# = 320 trials (not 500, as written in the thesis)

n_outliers = [50,100,200,300,400,500,1000]
n_trials = 10

group_size=10
local_neighborhood=20

policies = [ 
	[],   # random 
	["--local-loops", "--local-neighborhood",str(local_neighborhood)], #local
	["--group-loops","--group-size",str(group_size)], #grouped
	["--local-loops","--local-neighborhood",str(local_neighborhood),"--group-loops","--group-size",str(group_size)] #locally grouped
]

class DefaultHelpParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write('error: %s\n' % message)
        self.print_help()
        sys.exit(2)


if __name__ == "__main__":

	parser = DefaultHelpParser(description='Generate outlier loops like in Niko Suenderhaufs PhD thesis.')

	parser.add_argument("original", help = "Path to the original, outlier-free, dataset file (in g2o format).")
	parser.add_argument("output_directory", help = "Output directory for outlier files")
	parser.add_argument("--outlier-generator", default=os.path.normpath(os.path.dirname(os.path.realpath(__file__))+"/../bin/outlier_generator"), help="Path to the outlier_generator executable")
	parser.add_argument("--outlier-format", default='%(policy_index)02d_%(n_outliers)05d_%(trial)02d.outliers', help="Format of outlier files, keys to be used: policy_index, n_outliers, trial. Default: '%%(policy_index)02d_%%(n_outliers)05d_%%(trial)02d.outliers'")
	parser.add_argument("--seed-format", default='%(policy_index)02d%(n_outliers)05d%(trial)02d', help="Like --outlier-format, but used to generate a seed number. Same keys as above. Default: '%%(policy_index)02d%%(n_outliers)05d%%(trial)02d'")
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
