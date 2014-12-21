#!/usr/bin/python

from __future__ import print_function

import argparse
import sys
import graph as G
from utils import DefaultHelpParser

class hyper_maxmix_output(G.base_g2o_output):
	def __init__(self,dim,null_weight, null_inf_factor):
		super(hyper_maxmix_output, self).__init__(dim)

		self.null_weight=null_weight
		self.null_inf_factor=null_inf_factor

		self.maxmix_edge_tag = self.edge_tag+"_MIXTURE"


	def output_edge(self,i,e):
		if e.isSimple():
			super(hyper_maxmix_output, self).output_edge(i,e)
			return

		e.normalize(self.null_weight)


		count = len(e.motion_batches)
		if e.has_null_hypothesis:
			count+=1

		self.out.write(self.maxmix_edge_tag)
		self.out.write( " %s %s %d" % (e.reference, e.motion_batches[0].target, count))


		if e.has_null_hypothesis:
			self.out.write(" %s %s %d %d %s %s" % ( self.edge_tag, str(self.null_weight), e.reference, e.motion_batches[0].target, " ".join([str(x) for x in e.motion_batches[0].motions[0].mean]), " ".join([str(self.null_inf_factor*x) for x in e.motion_batches[0].motions[0].inf_up]) ))

		for b in e.motion_batches:
			for w,m in zip(b.weights,b.motions):
				self.out.write( " %s %s %d %d %s" %( self.edge_tag, str(w*b.batch_weight), e.reference, b.target, " ".join([str(x) for x in m]) ) )

		self.out.write("\n")

if __name__ == "__main__":

	parser = DefaultHelpParser(description='Convert a pair of original g2o file with corresponding outliers to hyper maxmixture graph.')

	parser.add_argument("input", type=argparse.FileType('r'), help = "Path to the original dataset file (in g2o format).")
	parser.add_argument("outliers", type=argparse.FileType('r'), help = "Outliers will be read from this file.")
	parser.add_argument("output", type=argparse.FileType('w'), help = "Plain graph will be written into this file.")
	parser.add_argument("--make-all-loops-hyperedges", default=False, dest="all_hyper", action='store_true', help="If given, make all non-sequential edges hyperedges, even though they do not have an assigned outlier.")
	parser.add_argument("--seq-init", default=False, dest="do_seq", action='store_true', help="If given, do a sequential initialization (aka odometry init in g2o) including outliers.")
	parser.add_argument("--bfs-init", default=False, dest="do_bfs", action='store_true', help="If given, do a breadth first initialization (aka spanning tree init in g2o) based on complete graph including outliers.")
	parser.add_argument("--bfs-with-null", default=False, dest="do_bfs_with_null", action='store_true', help="If given, also use edges with null hypothesis for bfs initialization.")
	parser.add_argument("--null-weight", type=float, default=1e-3, dest="null_weight", help="Weight of null hypothesis, used during hypercomponent weight normalization. Default: 1e-3")
	parser.add_argument("--null-information-scale", type=float, default=1e-12, dest="null_inf_factor", help="Factor for generating the null hypothesis information matrix, default: 1e-12")

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


	g.writeg2o(args.output,hyper_maxmix_output(g.dim, args.null_weight, args.null_inf_factor))
