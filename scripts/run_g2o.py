#!/usr/bin/python


import argparse
import sys
import os
import glob
import traceback
from subprocess import check_call

class DefaultHelpParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write('error: %s\n' % message)
        self.print_help()
        sys.exit(2)

if __name__ == "__main__":

	parser = DefaultHelpParser(description='Run g2o command line executable with options for many g2o files. Additional command line options are passed on to g2o.')

	parser.add_argument("graphs", nargs="+", help = "Filenames or glob patterns matching g2o files to be optimized.")
	parser.add_argument("--g2o", default="/opt/g2o/bin/g2o", help="Path to the g2o executable")
	parser.add_argument("--solver", default="gn_var_cholmod", help="Solver to use in g2o, default: gn_var_cholmod")
	parser.add_argument("--max-iterations", type=int, default=20, help="Number of iterations to run, default: 20")
	parser.add_argument("--env", default=["LD_LIBRARY_PATH=/opt/g2o/lib"], nargs="*", help="List of environment variables to set for subprocess calling g2o. e.g. LD_LIBRARY_PATH+=bla. = assigns, += appends (with semicolon)")
	parser.add_argument("--output-dir", help="Optional output directory other than the directory containing g2o files")
	parser.add_argument("--output-prefix", help="Optional prefix that is prepended to the output file name")
	parser.add_argument("--output-suffix", help="Optional suffix that is prepended to the output file name")

	(args, extra_args) = parser.parse_known_args()


	print "Going to pass these args on to g2o:", extra_args

	if not os.path.exists(args.g2o):
		print "ERROR: g2o executable not valid!"
		exit(1)

	args.g2o = os.path.realpath(args.g2o)

	if args.output_dir and os.path.exists(args.output_dir) and not os.path.isdir(args.output_dir):
		print "ERROR: output dir is not a dir"
		exit(3)

	if args.output_dir and not os.path.exists(args.output_dir):
		os.makedirs(args.output_dir)

	if not args.output_dir and not args.output_prefix and not args.output_suffix:
		args.output_suffix = "_optimized"

	env = os.environ.copy()

	for v in args.env:
		if "+=" in v:
			(name,val) = v.split('+=')
			if name in env:
				env[name] += ';' + val
			else:
				env[name] = val
		else:
			(name,val) = v.split('=')
			env[name] = val


	for pattern in args.graphs:
		for graphfile in glob.glob(pattern):
			if not os.path.exists(graphfile):
				print "ERROR: outlier file '",graphfile,"does not exist!"
				continue

			output_dir = os.path.dirname(graphfile)
			if args.output_dir:
				output_dir = args.output_dir

			(output_name, ext) = os.path.splitext( os.path.basename(graphfile) )

			if args.output_prefix:
				output_name = args.output_prefix + output_name

			if args.output_suffix:
				output_name += args.output_suffix


			output_name = os.path.normpath(output_dir + "/" + output_name + ".g2o")

			command = [args.g2o]
			command += extra_args
			command += ['-solver', args.solver]
			command += ['-i', str(args.max_iterations)]
			command += ['-o', output_name, graphfile]

			print "*************************"
			print " ".join(command)
			check_call(command, env=env)