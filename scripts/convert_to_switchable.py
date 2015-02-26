#!/usr/bin/python

from __future__ import print_function

import argparse
import sys
import graph as G
from utils import DefaultHelpParser

class switchable_output(G.base_g2o_output):
	def __init__(self,graph,switch_inf, switch_prior,weight_as_prior):
		super(switchable_output, self).__init__(graph)

		self.switch_inf=switch_inf
		self.switch_prior=switch_prior
		self.weight_as_prior=weight_as_prior

		self.switchable_edge_tag = self.edge_tag+"_SWITCHABLE"

		self.max_vertex_id=-1

	def output_vertex(self,i,v):
		super(switchable_output, self).output_vertex(i,v)

		if i > self.max_vertex_id:
			self.max_vertex_id=i


	def output_edge(self,i,e):
		if e.isSimple():
			super(switchable_output, self).output_edge(i,e)
			return

		e.normalize()


		for b in e.motion_batches:
			
			for m in b.motions:
				w = m.weight
				prior=self.switch_prior
				if self.weight_as_prior:
					prior = w*b.batch_weight

				self.max_vertex_id+=1

				print("VERTEX_SWITCH %d %s" %(self.max_vertex_id,str(prior)), file=self.out )
				print("EDGE_SWITCH_PRIOR %d %s %s" %(self.max_vertex_id,str(prior),str(self.switch_inf)), file=self.out )
				print("%s %d %d %d %s" %( self.switchable_edge_tag,e.reference, b.target, self.max_vertex_id, " ".join([str(x) for x in m])), file=self.out )
				

if __name__ == "__main__":

	parser = DefaultHelpParser(description='Convert a pair of original g2o file with corresponding outliers to a switchable constraints graph.')

	parser.add_argument("input", type=argparse.FileType('r'), help = "Path to the original dataset file (in g2o format).")
	parser.add_argument("outliers", type=argparse.FileType('r'), help = "Outliers will be read from this file.")
	parser.add_argument("output", type=argparse.FileType('w'), help = "Plain graph will be written into this file.")
	parser.add_argument("--make-all-loops-hyperedges", default=False, dest="all_hyper", action='store_true', help="If given, make all non-sequential edges hyperedges, even though they do not have an assigned outlier.")
	parser.add_argument("--seq-init", default=False, dest="do_seq", action='store_true', help="If given, do a sequential initialization (aka odometry init in g2o) including outliers.")
	parser.add_argument("--bfs-init", default=False, dest="do_bfs", action='store_true', help="If given, do a breadth first initialization (aka spanning tree init in g2o) based on complete graph including outliers.")
	parser.add_argument("--bfs-with-null", default=False, dest="do_bfs_with_null", action='store_true', help="If given, also use edges with null hypothesis for bfs initialization.")
	parser.add_argument("--switch-inf", type=float, default=1.0, dest="switch_inf", help="Switch value information, default: 1.0")
	parser.add_argument("--switch-prior", type=float, default=1.0, dest="switch_prior", help="Prior value for switch, default: 1.0")
	parser.add_argument("--use-weight-as-prior", default=False, dest="weight_as_prior", action='store_true', help="If given, use outlier weight as switching prior.")

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

	g.writeg2o(args.output,switchable_output(g, args.switch_inf, args.switch_prior, args.weight_as_prior))

