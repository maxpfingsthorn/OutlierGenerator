#!/usr/bin/python

import argparse
import sys


def readg2o(f):
	v=""
	e=list()
	dim = 0


	lines=f.readlines()

	for l in lines:
		elems = l.split()

		if l[0] == '#' or len(elems) == 0:
			continue

		if dim == 0 and ( elems[0] == "VERTEX_SE2" or elems[0] == "EDGE_SE2" ):
			dim = 2
		if dim == 0 and ( elems[0] == "VERTEX_SE3:QUAT" or elems[0] == "EDGE_SE3:QUAT" ):
			dim = 3

		if elems[0] == "VERTEX_SE2" or elems[0] == "VERTEX_SE3:QUAT" or elems[0] == "FIX":
			v += l

		elif elems[0] == "EDGE_SE2" or elems[0] == "EDGE_SE3:QUAT":
			e.append(elems)

	return (v,e,dim)

def readOutliers(f):
	outliers_on_inliers=dict()
	other_outliers=list()
	dim = 0

	lines=f.readlines()

	current_outlier_batch=dict()
	next_weight = 1.0

	for l in lines:
		elems = l.split()

		if l[0] == '#' or len(elems) == 0:
			continue

		if dim == 0 and elems[0] == "EDGE_SE2":
			dim = 2
		if dim == 0 and elems[0] == "EDGE_SE3:QUAT":
			dim = 3

		if elems[0] == 'LOOP_OUTLIER_BATCH':
			current_outlier_batch=dict()
			current_outlier_batch['has_inlier'] = elems[3]=='1'
			current_outlier_batch['has_null_hypothesis'] = elems[2]=='1'
			current_outlier_batch['reference'] = elems[1]
			current_outlier_batch['inlier_target'] = elems[4]
			current_outlier_batch['hyper_constraints'] = []

		elif elems[0] == 'MOTION_OUTLIER_BATCH':
			
			current_outlier_batch['hyper_constraints'].append(dict())
			current_outlier_batch['hyper_constraints'][-1]['hyper_weight']=float(elems[2])
			current_outlier_batch['hyper_constraints'][-1]['target']=elems[1]
			current_outlier_batch['hyper_constraints'][-1]['weights']=[]
			current_outlier_batch['hyper_constraints'][-1]['constraints']=[]

		elif elems[0] == 'MOTION_WEIGHT':
			next_weight = float(elems[1])

		elif elems[0] == 'EDGE_SE2' or elems[0] == 'EDGE_SE3:QUAT':
			current_outlier_batch['hyper_constraints'][-1]['constraints'].append(elems)
			current_outlier_batch['hyper_constraints'][-1]['weights'].append(next_weight)

		elif elems[0] == 'LOOP_OUTLIER_BATCH_END':
			if current_outlier_batch['has_inlier']:
				key=current_outlier_batch['reference']+','+current_outlier_batch['inlier_target']
				outliers_on_inliers[key] = current_outlier_batch
			else:
				other_outliers.append(current_outlier_batch)


	return (outliers_on_inliers, other_outliers, dim)




if __name__ == "__main__":

	parser = argparse.ArgumentParser(description='Convert a pair of original g2o file with corresponding outliers to plain g2o graph where all outliers are normal edges.')

	parser.add_argument("input", type=argparse.FileType('r'), help = "Path to the original dataset file (in g2o format).")
	parser.add_argument("outliers", type=argparse.FileType('r'), help = "Outliers will be read from this file.")
	parser.add_argument("output", type=argparse.FileType('w'), help = "Plain graph will be written into this file.")

	args = parser.parse_args()


	(V,E,dim_g2o) = readg2o(args.input)

	#print "number of vertices in g2o file:",len(V)
	print "number of edges in g2o file:",len(E)

	(outliers_on_inliers, other_outliers, dim_outliers) = readOutliers(args.outliers)

	print "number inliers that have outliers (loops and motions):",len(outliers_on_inliers)
	print "number of outlier loop batches:",len(other_outliers)

	if( len(outliers_on_inliers)==0 and len(other_outliers)==0):
		print "ERROR: No outliers!"
		sys.exit(1)

	if( dim_g2o != dim_outliers):
		print "ERROR! Dimensions of g2o and outlier files are not the same! "
		print "G2O: is %dD, outliers are %dD" % (dim_g2o, dim_outliers)
		sys.exit(2)



	args.output.write(V)


	for e in E:
		args.output.write( (" ".join(e))+ "\n")


	for o in [v for k,v in outliers_on_inliers.iteritems() ] + other_outliers:
		for h in o['hyper_constraints']:
			for c in h['constraints']:
				args.output.write( " ".join(c) + "\n")