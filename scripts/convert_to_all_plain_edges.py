#!/usr/bin/python

from __future__ import print_function

import argparse
import sys
import graph as G
from utils import DefaultHelpParser

class plain_output(G.base_g2o_output):
	def output_edge(self,i,e):
		for b in e.motion_batches:
			for m in b.motions:
				print( "%s %d %d %s" %( self.edge_tag, e.reference, b.target, " ".join([str(x) for x in m]) ), file=self.out )

if __name__ == "__main__":

	parser = DefaultHelpParser(description='Convert a pair of original g2o file with corresponding outliers to plain g2o graph where all outliers are normal edges.')

	parser.add_argument("input", type=argparse.FileType('r'), help = "Path to the original dataset file (in g2o format).")
	parser.add_argument("outliers", type=argparse.FileType('r'), help = "Outliers will be read from this file.")
	parser.add_argument("output", type=argparse.FileType('w'), help = "Plain graph will be written into this file.")
	parser.add_argument("--make-all-loops-hyperedges", default=False, dest="all_hyper", action='store_true', help="If given, make all non-sequential edges hyperedges, even though they do not have an assigned outlier.")
	parser.add_argument("--seq-init", default=False, dest="do_seq", action='store_true', help="If given, do a sequential initialization (aka odometry init in g2o) including outliers.")
	parser.add_argument("--bfs-init", default=False, dest="do_bfs", action='store_true', help="If given, do a breadth first initialization (aka spanning tree init in g2o) based on complete graph including outliers.")
	parser.add_argument("--bfs-with-null", default=False, dest="do_bfs_with_null", action='store_true', help="If given, also use edges with null hypothesis for bfs initialization.")

	args = parser.parse_args()

	if args.do_bfs and args.do_seq:
		print("ERROR: specify either --seq-init or --bfs-init, not both")
		exit(1)

	g = G.readg2o(args.input)

	g.readExtraOutliers(args.outliers)

	if args.all_hyper:
		g.makeAllLoopsHaveNullHypothesis()

	if args.do_bfs:
		g.setNonfixedPosesToZero()
		g.intializePosesBFS(args.do_bfs_with_null)

	if args.do_seq:
		g.setNonfixedPosesToZero()
		g.intializePosesSequential()


	g.writeg2o(args.output,plain_output(g.dim))