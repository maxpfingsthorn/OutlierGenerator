from __future__ import print_function

import argparse
import sys
from collections import deque
from copy import deepcopy
import itertools

import pose_utils as pu
from utils import DefaultHelpParser

class Motion:
	def __init__(self, weight, elems):
		self.weight = weight
		if len(elems) > 13: # 3D
			self.mean = [float(x) for x in elems[3:10]]
			self.inf_up = [float(x) for x in elems[10:]]
		else:
			self.mean = [float(x) for x in elems[3:6]]
			self.inf_up = [float(x) for x in elems[6:]]

		ok = (len(self.mean) == 7 and len(self.inf_up) == 21) or (len(self.mean) == 3 and len(self.inf_up) == 6)
		if not ok:
			raise ValueError("Did not split pose and information matrix properly! %d %d" % (len(self.mean), len(self.inf_up)))

	def __iter__(self):
		return itertools.chain(self.mean, self.inf_up)

class ConstraintMotions:
	def __init__(self, batch_weight, target, init_str=None):
		self.batch_weight = batch_weight
		self.target = target

		self.motions = [] # Motion instances

		if init_str:
			self.addMotion(1.0, init_str)

	def addMotion(self, weight, elems):
		self.motions.append(Motion(weight, elems))

	def normalize(self,null_hypothesis_weight=0.0):
		norm = sum([x.weight for x in self.motions])+null_hypothesis_weight
		for i in range(len(self.motions)):
			self.motions[i].weight /= norm

	def getMaxMotion(self):
		maxmotion=None
		for m in self.motions:
			if not maxmotion or maxmotion.weight < m.weight:
				maxmotion = m

		return maxmotion



class ConstraintBatch:
	def __init__(self, has_inlier, has_null_hypothesis, reference, inlier_target=None, inlier_str=None):
		self.has_inlier = has_inlier
		self.has_null_hypothesis = has_null_hypothesis
		self.reference = reference
		self.inlier_target = inlier_target

		self.motion_batches = []

		if inlier_str:
			self.motion_batches.append( ConstraintMotions(1.0, inlier_target, inlier_str) )

	def getSimpleEdge(self):
		if not self.isSimple():
			return None
		return self.motion_batches[0].motions[0]

	def isSimple(self):
		return self.ambiguity() == 1

	def isSimpleLoop(self):
		return self.isSimple() and self.reference+1 != self.motion_batches[0].target

	def ambiguity(self):
		s= sum([len(x.motions) for x in self.motion_batches])
		if self.has_null_hypothesis:
			s += 1
		return s

	def targets(self):
		return [x.target for x in self.motion_batches]

	def getMax(self):
		maxbatch=None
		for b in self.motion_batches:
			if not maxbatch or maxbatch.batch_weight < b.batch_weight:
				maxbatch = b

		return maxbatch

	def getOrCreateMotions(self,target,weight):
		for i in range(len(self.motion_batches)):
			if self.motion_batches[i].target == target:
				return self.motion_batches[i]

		self.addMotions( ConstraintMotions(weight, target) )
		return self.motion_batches[-1]


	def addMotions(self,motion):
		self.motion_batches.append(motion)

	def normalize(self,null_hypothesis_weight=0.0):
		norm = sum([x.batch_weight for x in self.motion_batches]) + null_hypothesis_weight

		for i in range(0,len(self.motion_batches)):
			self.motion_batches[i].batch_weight /= norm

			self.motion_batches[i].normalize()

	# ordering for output
	def __lt__(self,other):
		ms=self.isSimple()
		os=other.isSimple()

		if ms and not os:
			return True
		if os and not ms:
			return False

		if ms and os:
			ml=self.isSimpleLoop()
			ol=other.isSimpleLoop()

			if ml and not ol:
				return False
			if ol and not ml:
				return True

		if self.reference < other.reference:
			return True
		elif self.reference > other.reference:
			return False

		if len(self.motion_batches) < len(other.motion_batches):
			return True
		elif len(self.motion_batches) > len(other.motion_batches):
			return False

		for i in range(0,len(self.motion_batches)):
			if self.motion_batches[i].target < other.motion_batches[i].target:
				return True
			elif self.motion_batches[i].target > other.motion_batches[i].target:
				return False

		return False # same



class base_g2o_output(object):
	def __init__(self,graph):
		self.graph=graph
		self.out=None
		self.dim = graph.dim
		
		if graph.dim == 2:
			self.vertex_tag = "VERTEX_SE2"
			self.edge_tag = "EDGE_SE2"
		else:
			self.vertex_tag = "VERTEX_SE3:QUAT"
			self.edge_tag = "EDGE_SE3:QUAT"

	def setFile(self,out):
		self.out = out

	def output_vertex(self,i,v):
		if not self.out:
			raise ValueError("Don't have an output file!")
		print( "%s %d %s" % (self.vertex_tag, i, " ".join([str(x) for x in v]) ), file=self.out)

		if i in self.graph.fixed:
			print("FIX %d" % i, file=self.out)

	def output_edge(self,i,e):
		if not self.out:
			raise ValueError("Don't have an output file!")
			
		if not e.isSimple():
			print("ERROR: base_g2o_output can't process complex edges! id: %s" % i, file=sys.stderr)
			return

		print( "%s %d %d %s" %( self.edge_tag, e.reference, e.motion_batches[0].target, " ".join([str(x) for x in e.motion_batches[0].motions[0]]) ), file=self.out )

class Graph:
	"""A class represeting a graph, maybe with outliers"""

	def __init__(self, other=None):
		if other:
			self.V = deepcopy(other.V)
			self.E = deepcopy(other.E)

			self.fixed = deepcopy(other.fixed)

			self.vertex_tag = str(other.vertex_tag)
			self.edge_tag = str(other.edge_tag)

			self.dim = other.dim

			if other.adj:
				self.buildAdjacency()
			else:
				self.adj = None

			return

		self.V = dict()
		self.E = dict()

		self.fixed = set()

		self.vertex_tag=None
		self.edge_tag=None

		self.dim = None

		self.adj = None

	def make_edge_key(self,ref, targets):
		key = str(ref)

		if type(targets) is list:
			for t in targets:
				key += "_"+str(t)
		else:
			key += "_"+str(targets)

		return key


	def readg2o(self,f):
		self.V = dict()
		self.E = dict()
		self.fixed = set()
		self.adj = None
		self.dim = None

		for l in f:
			elems = l.split()
			if l[0] == '#' or len(elems) == 0:
				continue

			if not self.dim and ( elems[0] == "VERTEX_SE2" or elems[0] == "EDGE_SE2" ):
				self.dim = 2
				self.edge_tag = "EDGE_SE2"
				self.vertex_tag = "VERTEX_SE2"
			if not self.dim and ( elems[0] == "VERTEX_SE3:QUAT" or elems[0] == "EDGE_SE3:QUAT" ):
				self.dim = 3
				self.edge_tag = "EDGE_SE3:QUAT"
				self.vertex_tag = "VERTEX_SE3:QUAT"


			if elems[0] == self.vertex_tag:
				if int(elems[1]) in self.V:
					print("WARNING: already saw vertex %s, skipping this one" %(elems[1]), file=sys.stderr)
					continue
				self.V[ int(elems[1]) ] = [float(x) for x in elems[2:]]
			elif elems[0] == "FIX":
				self.fixed.add( int(elems[1]) )
				#print("fixing %d" % int(elems[1]))
			elif elems[0] == self.edge_tag:
				key = self.make_edge_key(elems[1], elems[2])
				if key in self.E:
					print("WARNING: already saw edge from %s to %s, skipping this one" %(elems[1],elems[2]), file=sys.stderr)
					continue
				self.E[key] = ConstraintBatch(True, False, int(elems[1]), int(elems[2]), elems)

	def mapVertices(self,functor):
		return map(lambda x: functor(*x), iter(sorted(self.V.items(), key=lambda x: x[0])) )

	def mapEdges(self,functor):
		return map(lambda x: functor(*x), iter(sorted(self.E.items(), key=lambda x: x[1])) )

	def writeg2o(self,f,g2o_output_functor=None):
		if not g2o_output_functor:
			g2o_output_functor = base_g2o_output(self)

		g2o_output_functor.setFile(f)

		self.mapVertices( lambda i,v: g2o_output_functor.output_vertex(i,v) )
		self.mapEdges(    lambda i,e: g2o_output_functor.output_edge(i,e)   )

	# adds outliers to this graph, can be called multiple times to add outliers from many files
	def readExtraOutliers(self, f):
		current_outlier_batch=None
		current_motions=None
		next_weight = 1.0

		for l in f:
			elems = l.split()
			if l[0] == '#' or len(elems) == 0:
				continue

			if (elems[0] == "EDGE_SE2" and self.dim != 2) or (elems[0] == "EDGE_SE3:QUAT" and self.dim != 3):
				raise ValueError("You tried to load outliers with a different dimension than the original g2o graph!")

			if elems[0] == 'LOOP_OUTLIER_BATCH':
				if elems[3]=='1': # has inlier
					key = self.make_edge_key(elems[1],elems[4])
					if not key in self.E:
						raise ValueError("Could not find inlier edge for outlier (from: %s, to: %s)", (elems[1],elems[4]))

					current_outlier_batch = self.E[ key ]
				else:
					current_outlier_batch=ConstraintBatch(
						elems[3]=='1', # has_inlier
						elems[2]=='1', # has_null_hypothesis
						int(elems[1]), # reference
						int(elems[4]) if int(elems[4])>=0 else None # inlier_target
						)				

			elif elems[0] == 'MOTION_OUTLIER_BATCH':
				target = int(elems[1])
				weight = float(elems[2])
				current_motions = current_outlier_batch.getOrCreateMotions( target, weight )

			elif elems[0] == 'MOTION_WEIGHT':
				next_weight = float(elems[1])

			elif elems[0] == self.edge_tag:
				current_motions.addMotion(next_weight, elems)

			elif elems[0] == 'LOOP_OUTLIER_BATCH_END':
				if not current_outlier_batch.has_inlier:
					key=self.make_edge_key(current_outlier_batch.reference,current_outlier_batch.targets())
					
					self.E[key] = current_outlier_batch


	# needed for traversal
	def buildAdjacency(self):
		self.adj = dict()

		for key,e in self.E.iteritems():
			for v in [e.reference] + e.targets():
				if not v in self.adj:
					self.adj[v]=[]
				self.adj[v].append(key)


	def setNonfixedPosesToZero(self):
		for k in self.V:
			if k not in self.fixed:
				for i in range(0,len(self.V[k])):
					self.V[k][i] = 0.0
				if self.dim == 3:
					self.V[k][6] = 1.0

	def makeAllLoopsHaveNullHypothesis(self):
		for k in self.E:
			if self.E[k].isSimpleLoop():
				self.E[k].has_null_hypothesis = True

	def initializePosesSequential(self):
		if len(self.fixed) != 1:
			print("Don't know how to sequentially initialize with more than one fixed vertex, have %d" % len(self.fixed), file=sys.stderr)
			return

		v_fix = list(self.fixed)[0]

		v_cur = v_fix
		v_next = v_cur+1
		key = self.make_edge_key(v_cur, v_next)
		while key in self.E:
			ee = self.E[key]
			m = ee.getMax()
			c = m.getMaxMotion()

			self.V[v_next] = pu.compound( self.V[v_cur], c.mean, False)
			v_cur=v_next
			v_next=v_cur+1
			key = self.make_edge_key(v_cur, v_next)

		v_cur = v_fix
		v_next = v_cur-1

		key = self.make_edge_key(v_cur, v_next)
		while key in self.E:
			ee = self.E[key]
			m = ee.getMax()
			c = ee.getMaxMotion()

			self.V[v_next] = pu.compound( self.V[v_cur], c.mean, True)
			v_cur=v_next
			v_next=v_cur-1
			key = self.make_edge_key(v_cur, v_next)



	def intializePosesBFS(self,with_null=True):
		self.buildAdjacency()

		queue = deque()

		assigned = set(self.fixed)
		used = set()

		for i in assigned:
			for e in self.adj[i]:
				queue.append( (i,e) )

		while len(queue) > 0 and len(assigned) < len(self.V):
			i,e = queue.popleft()

			#print("popped %d %s" %(i,e))

			used.add(e)

			ee = self.E[e]

			if ee.has_null_hypothesis and not with_null:
				continue

			m = ee.getMax()

			if i != ee.reference and i != m.target:
				#print("ERROR: i neither ref nor tar")
				continue;

			if i == ee.reference:
				next_i = m.target
				#print("next v is %d" % next_i)
				if next_i in assigned:
					#print("already assigned")
					continue
				#print("ref: %s, diff: %s" %(" ".join([str(x) for x in self.V[i]]), " ".join([str(x) for x in m.getMax().mean])))
				self.V[next_i] = pu.compound( self.V[i], m.getMaxMotion().mean, False )
				#print(self.V[next_i])
			else:
				next_i = ee.reference
				#print("next v is %d" % next_i)
				if next_i in assigned:
					#print("already assigned")
					continue
				#print("ref: %s, diff: %s" %(" ".join([str(x) for x in self.V[i]]), " ".join([str(x) for x in m.getMax().mean])))
				self.V[next_i] = pu.compound( self.V[i], m.getMaxMotion().mean, True)
				#print(self.V[next_i])

			#print("*** Assigned %d" % next_i)
			assigned.add(next_i)

			for a in self.adj[next_i]:
				#print("%s is adj to %d" %(a, next_i))
				if a not in used:
					#print("queued (%d,%s)" %(next_i, a))
					queue.append( (next_i, a) )

		#print("len(queue): %d, len(assigned): %d, len(self.V): %d" %(len(queue), len(assigned), len(self.V)))


def readg2o(f):
	g=Graph()
	g.readg2o(f)
	return g