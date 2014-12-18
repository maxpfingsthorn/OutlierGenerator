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


def output_batch(f,batch,dim,null_weight,null_inf_factor):
	edge_tag = "EDGE_"
	
	mean_len = 0

	if dim==2:
		edge_tag += "SE2"
		mean_len=3
	elif dim==3:
		edge_tag += "SE3:QUAT"
		mean_len=7
	
	mixture_tag = edge_tag+"_MIXTURE"



	for c in batch['hyper_constraints']:

		constraint=c['constraints'][0]
		scaled_inf = [ str(null_inf_factor * float(x)) for x in constraint[(3+mean_len+1):] ]

		count = len(c['constraints'])
		if batch['has_null_hypothesis']:
			count += 1

		f.write( "%s %s %s %d" %(mixture_tag,  batch['reference'], c['target'], count ) )

		if batch['has_null_hypothesis']:
			f.write(" %s %s %s %s %s %s" % ( edge_tag, str(null_weight), batch['reference'], c['target'], " ".join(constraint[3:(3+mean_len)]), " ".join(scaled_inf) ) )

		for m in range(0,len(c['constraints'])):

			f.write( " %s %s %s %s %s" %( edge_tag, str( c['weights'][m]/sum(c['weights']+[null_weight]) ), batch['reference'], c['target'], " ".join(c['constraints'][m][3:]) ))

		f.write("\n")

class DefaultHelpParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write('error: %s\n' % message)
        self.print_help()
        sys.exit(2)
        
if __name__ == "__main__":

	parser = DefaultHelpParser(description='Convert a pair of original g2o file with corresponding outliers to a MaxMix graph, one maxmix per hypercomponent.')

	parser.add_argument("input", type=argparse.FileType('r'), help = "Path to the original dataset file (in g2o format).")
	parser.add_argument("outliers", type=argparse.FileType('r'), help = "Outliers will be read from this file.")
	parser.add_argument("output", type=argparse.FileType('w'), help = "MaxMix graph will be written into this file.")
	parser.add_argument("--null-weight", type=float, default=1e-3, dest="null_weight", help="Weight of null hypothesis, used during hypercomponent weight normalization. Default: 1e-3")
	parser.add_argument("--null-information-scale", type=float, default=1e-12, dest="null_inf_factor", help="Factor for generating the null hypothesis information matrix, default: 1e-12")
	parser.add_argument("--make-all-loops-hyperedges", default=False, dest="all_hyper", action='store_true', help="If given, make all non-sequential edges hyperedges, even though they do not have an assigned outlier.")

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
		key=e[1]+','+e[2]

		if key in outliers_on_inliers:
			#args.output.write("THIS SHALL BE A HYPER_MOG: "+(" ".join(e))+"\n")
			batch=outliers_on_inliers[key]
			if batch['hyper_constraints'][0]['target'] != e[2]:
				batch['hyper_constraints'].insert(0,dict())
				batch['hyper_constraints'][0]['hyper_weight']=1.0
				batch['hyper_constraints'][0]['target']=e[2]
				batch['hyper_constraints'][0]['constraints']=[]
				batch['hyper_constraints'][0]['weights']=[]

			batch['hyper_constraints'][0]['constraints'].insert(0,e)
			batch['hyper_constraints'][0]['weights'].insert(0,1.0)

			output_batch(args.output, batch, dim_g2o, args.null_weight, args.null_inf_factor)
		else:
			if int(e[1])+1 != int(e[2]) and args.all_hyper:
				batch=dict()
				batch['has_inlier'] = False
				batch['has_null_hypothesis'] = True
				batch['reference'] = e[1]
				batch['inlier_target'] = e[2]
				batch['hyper_constraints']=[]
				batch['hyper_constraints'].append(dict())
				batch['hyper_constraints'][0]['target'] = e[2]
				batch['hyper_constraints'][0]['hyper_weight'] = 1.0
				batch['hyper_constraints'][0]['constraints']=[]
				batch['hyper_constraints'][0]['weights']=[]

				batch['hyper_constraints'][0]['constraints'].insert(0,e)
				batch['hyper_constraints'][0]['weights'].insert(0,1.0)

				output_batch(args.output, batch, dim_g2o, args.null_weight, args.null_inf_factor)

			else:
				args.output.write( (" ".join(e))+ "\n")


	for o in other_outliers:
		output_batch(args.output, o, dim_g2o, args.null_weight, args.null_inf_factor)