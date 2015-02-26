#!/usr/bin/python

import argparse
import sys
import traceback
from subprocess import check_call
import os
from math import floor

# as in Olson and Agarwal's IJRR paper, generate a number of outlier loops, to up 4000.
# #outlier schedule (as per table 1 of the paper) is: 10, 100, 200, 300, 400, 500, 1000, 2000, 3000, 4000
# see also comment in https://github.com/agpratik/max-mixture/tree/master/datasets/multimodal


n_outliers_ratio = [.1, .2, .3, .4, .5, .6, .7, .8, .9, 1.0]
n_trials = 10
n_hypercomp = [3,4]

class DefaultHelpParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write('error: %s\n' % message)
        self.print_help()
        sys.exit(2)

def count_loop_edges(file):
	f = open(file,'r')
	count=0
	for l in f:
		e=l.split()
		if len(e)==0 or l[0]=='#':
			continue
		if e[0].startswith("EDGE_SE") and int(e[1])+1 != int(e[2]):
			count+=1
	return count

if __name__ == "__main__":

	parser = DefaultHelpParser(description='Generate multimodal outlier loops like in Olson and Agarwal\'s IJRR paper.')

	parser.add_argument("original", help = "Path to the original, outlier-free, dataset file (in g2o format).")
	parser.add_argument("output_directory", help = "Output directory for outlier files")
	parser.add_argument("--outlier-generator", default=os.path.normpath(os.path.dirname(os.path.realpath(__file__))+"/../bin/outlier_generator"), help="Path to the outlier_generator executable")
	parser.add_argument("--outlier-format", default='%(n_outliers)02d_%(n_hypercomp)02d_%(trial)03d.outliers', help="Format of outlier files, keys to be used: n_outliers, n_hypercomp, trial. Default: '%%(n_outliers)02d_%%(n_hypercomp)02d_%%(trial)03d.outliers'")
	parser.add_argument("--seed-format", default='1%(n_outliers)02d%(n_hypercomp)02d%(trial)03d', help="Like --outlier-format, but used to generate a seed number. Same keys as above. Default: '1%%(n_outliers)02d%%(n_hypercomp)02d%%(trial)03d'")
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
	
	loops = count_loop_edges(args.original)

	try:

		for no in range(0,len(n_outliers_ratio)):
			for nm in range(0,len(n_hypercomp)):
				for nt in range(0,n_trials):
					cur = {'n_outliers': no, 'n_hypercomp': nm, 'trial': nt }
					commandline = [args.outlier_generator, args.original, os.path.normpath(args.output_directory + "/" + args.outlier_format % cur ), ]                
					commandline += ['--false-loops-on-inliers']
					commandline += ["0" for i in range(0,n_hypercomp[nm]-2)] + [str(int(floor(n_outliers_ratio[no]*loops)))]
					commandline += ['--seed', args.seed_format % cur ]
					commandline += ['--loop-variance-translation', str(1.0/(args.loop_information[0])), '--loop-variance-rotation', str(1.0/(args.loop_information[1])) ]
					if no > 0:
						prev = cur
						prev['n_outliers'] = no-1
						commandline += ['--previous-outliers', os.path.normpath(args.output_directory + "/" + args.outlier_format % prev )]

					print "*************************"
					print " ".join(commandline)
					check_call(commandline, stdout=sys.stdout, stderr=sys.stderr)
		

	except:
		traceback.print_exc(file=sys.stderr)
		exit(100)
