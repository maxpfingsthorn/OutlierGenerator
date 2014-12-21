#!/usr/bin/python

from __future__ import print_function

import argparse
import sys
import os
import glob
import pose_utils as pu
import math
import graph as G
from utils import DefaultHelpParser


class error_calc:
	def __init__(self,ref):
		self.clear()
		self.ref = ref

	def clear(self):
		self.errors_tr = []
		self.errors_rot = []

	def calcRMSE(self):
		return ( math.sqrt( sum(self.errors_tr)/float(len(self.errors_tr)-1) ), math.sqrt( sum(self.errors_rot)/float(len(self.errors_rot)-1) ) )

	def __call__(self,i,v):
		if not i in self.ref.V:
			print("ERROR: vertex",i,"not in reference file!", file=sys.stderr)
			return

		v_ref = self.ref.V[i]
		(err_tr, err_rot) = pu.errors( v_ref, v )

		self.errors_tr.append(err_tr)
		self.errors_rot.append(err_rot)

if __name__ == "__main__":

	parser = DefaultHelpParser(description='Compute RMSE errors for translation and rotation (based on angle only) for a set of g2o graph files, given a reference g2o graph file (e.g. ground truth).')

	parser.add_argument("reference", type=argparse.FileType('r'), help = "Path to the reference (in g2o format).")
	parser.add_argument("graphs", nargs="+", help = "Filenames or glob patterns matching g2o files to be processed.")
	parser.add_argument("-o","--output", type=argparse.FileType('a+'), help="Output file to append the errors to. Default: stdout")
	
	args = parser.parse_args()

	if not args.output:
		args.output = sys.stdout

	G_ref = G.readg2o(args.reference)

	errs = error_calc(G_ref)

	for pattern in args.graphs:
		for graphfile in glob.glob(pattern):
			if not os.path.exists(graphfile):
				print("ERROR: graph file '",graphfile,"does not exist!", file=sys.stderr)
				continue

			G_cand =  G.readg2o( open(graphfile, 'r') )
			
			G_cand.mapVertices(errs)

			(RMSE_tr, RMSE_rot) = errs.calcRMSE()
			
			args.output.write( str(RMSE_tr) + " " + str(RMSE_rot) + "\n" )

