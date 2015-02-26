#!/usr/bin/python

from __future__ import print_function

import argparse
import sys
import graph as G
from utils import DefaultHelpParser

class old_hypermog_output(G.base_g2o_output):
	def __init__(self,graph,null_weight):
		super(old_hypermog_output, self).__init__(graph)

		self.null_weight=null_weight

		self.maxmix_edge_tag = self.edge_tag+"_MIXTURE"
		self.hyper_edge_tag = self.edge_tag+"_HYPER"


	def output_edge(self,i,e):
		if e.isSimple():
			super(old_hypermog_output, self).output_edge(i,e)
			return

		e.normalize(self.null_weight)


		if e.has_null_hypothesis:
			self.out.write(self.hyper_edge_tag)
		else:
			self.out.write(self.maxmix_edge_tag)

		total_comps = sum([len(x.motions) for x in e.motion_batches])

		self.out.write( " %s %s %d" % (e.reference, e.motion_batches[0].target,  total_comps))

		for b in e.motion_batches:
			for m in b.motions:
				w=m.weight
				self.out.write( " %s %s %d %d %s" %( self.edge_tag, str(w*b.batch_weight), e.reference, b.target, " ".join([str(x) for x in m]) ) )

		self.out.write("\n")

if __name__ == "__main__":

	parser = DefaultHelpParser(description='Convert a pair of original g2o file with corresponding outliers to multimodal hypergraph for Prefilter.')

	parser.add_argument("input", type=argparse.FileType('r'), help = "Path to the original dataset file (in g2o format).")
	parser.add_argument("outliers", type=argparse.FileType('r'), help = "Outliers will be read from this file.")
	parser.add_argument("output", type=argparse.FileType('w'), help = "Plain graph will be written into this file.")
	parser.add_argument("--make-all-loops-hyperedges", default=False, dest="all_hyper", action='store_true', help="If given, make all non-sequential edges hyperedges, even though they do not have an assigned outlier.")
	parser.add_argument("--seq-init", default=False, dest="do_seq", action='store_true', help="If given, do a sequential initialization (aka odometry init in g2o) including outliers.")
	parser.add_argument("--bfs-init", default=False, dest="do_bfs", action='store_true', help="If given, do a breadth first initialization (aka spanning tree init in g2o) based on complete graph including outliers.")
	parser.add_argument("--bfs-with-null", default=False, dest="do_bfs_with_null", action='store_true', help="If given, also use edges with null hypothesis for bfs initialization.")
	parser.add_argument("--null-weight", type=float, default=1e-3, dest="null_weight", help="Weight of null hypothesis, used during hypercomponent weight normalization. Default: 1e-3")

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
		g.initializePosesSequential()


	g.writeg2o(args.output,old_hypermog_output(g, args.null_weight))
