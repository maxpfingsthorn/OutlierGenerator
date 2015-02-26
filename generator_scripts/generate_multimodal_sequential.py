#!/usr/bin/python

import argparse
import sys
import traceback
from subprocess import check_call
import os


n_mogcomps = [
	[1],
	[2],
	[10]
	]

n_trials = 10

class DefaultHelpParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write('error: %s\n' % message)
        self.print_help()
        sys.exit(2)


if __name__ == "__main__":

    parser = DefaultHelpParser(description='Generate multimodal motions like in Pfingsthorn and Birk\'s 2015? IJRR paper.')

    parser.add_argument("original", help = "Path to the original, outlier-free, dataset file (in g2o format).")
    parser.add_argument("output_directory", help = "Output directory for outlier files")
    parser.add_argument("--outlier-generator", default=os.path.normpath(os.path.dirname(os.path.realpath(__file__))+"/../bin/outlier_generator"), help="Path to the outlier_generator executable")
    parser.add_argument("--outlier-format", default='%(n_mogcomp)01d_%(trial)02d.outliers', help="Format of outlier files, keys to be used: n_mogcomp, trial. Default: '%%(n_mogcomp)01d_%%(trial)02d.outliers'")
    parser.add_argument("--seed-format", default='1%(n_mogcomp)02d%(trial)03d', help="Like --outlier-format, but used to generate a seed number. Same keys as above. Default: '1%%(n_mogcomp)02d%%(trial)03d'")
    parser.add_argument("--motion-information", type=float, nargs=2, default=[10,200], help="Information values for generated outlier loops. Default: [10,200]")


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

        for nm in range(0,len(n_mogcomps)):
            for nt in range(1,n_trials+1):
                cur = {'n_mogcomp': nm, 'trial': nt }
                commandline = [args.outlier_generator, args.original, os.path.normpath(args.output_directory + "/" + args.outlier_format % cur ) ]
                commandline += ['--false-motions']
                commandline += [str(x) for x in n_mogcomps[nm]]
                commandline += ['--seed', args.seed_format % cur ]
                commandline += ['--motion-variance-translation', str(1.0/(args.motion_information[0])), '--motion-variance-rotation', str(1.0/(args.motion_information[1])) ]
                commandline += ['--first-motion-wins']
                commandline += ['--inflation-factor', '1.5']
                if nm > 0:
                    prev = cur
                    prev['n_mogcomp'] = nm-1
                    commandline += ['--previous-outliers', os.path.normpath(args.output_directory + "/" + args.outlier_format % prev )]

                print "*************************"
                print " ".join(commandline)
                check_call(commandline, stdout=sys.stdout, stderr=sys.stderr)
		

    except:
        traceback.print_exc(file=sys.stderr)
        exit(100)
