#include <map>
#include <string>
#include <set>
#include <fstream>
#include <iostream>
#include <sstream>
#include <utility>

#include <boost/program_options.hpp>
#include <boost/random.hpp>
#include <boost/lexical_cast.hpp>

#include <Eigen/Eigen>

#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

#include <sophus/Random.hpp>

#include <sophus/distributions/MixtureOf.hpp>
#include <sophus/distributions/MixtureOfSample.hpp>
#include <sophus/distributions/NormalDistributionOn.hpp>
#include <sophus/distributions/NormalDistributionSample.hpp>
#include <sophus/distributions/NormalDistributionConfidenceOperations.hpp>


using namespace std;
using namespace Eigen;
using namespace Sophus;
using namespace Sophus::Distributions;
namespace po = boost::program_options;


struct options {
	string original_g2o_file, output_file, existing_outliers_file;
	bool has_existing_outliers_file;

	uint32_t seed;

	vector<int> false_loops;  // i.e. loops where null is correct
	vector<int> false_loops_on_inliers; // extra hyper components on inlier constraints
	vector<int> false_motions; // extra MoG components on existing constraints, including false loops if combine_false_loops_and_motions==true
	
	bool combine_false_loops_and_motions;

	bool local_loops; // "local" policy of Suenderhauf
	int local_neighborhood;

	bool group_loops; // "grouped" policy of Suenderhauf
	int group_size;

	double min_weight;
	double max_weight;

	double loop_tr_variance;
	double loop_rot_variance;

	double motion_tr_variance;
	double motion_rot_variance;

	double min_confidence_distance_motions;
	double cov_inflation_factor_for_motion_generation;

	bool ensure_first_false_motion_has_larger_weight_than_inlier;

	options() {
		has_existing_outliers_file = false;

		seed = 0;

		combine_false_loops_and_motions = false;

		local_loops = false;
		local_neighborhood = 20;

		group_loops = false;
		group_size = 20;

		min_weight = .5;
		max_weight = 2.;

		loop_tr_variance = 10;
		loop_rot_variance = .5;

		motion_tr_variance = 1;
		motion_rot_variance = .05;

		min_confidence_distance_motions = .95;

		cov_inflation_factor_for_motion_generation = 1.5;

		ensure_first_false_motion_has_larger_weight_than_inlier = false;
	}
};

struct VertexData {
	int id;
	VectorXd ground_truth_pose;
};
struct ConstraintData {
	int reference, target;
	VectorXd mean;
	MatrixXd cov, inf;

	ConstraintData() : reference(-1), target(-1) {}
};
struct ConstraintMixtureData {
	vector<double> weights;
	vector<ConstraintData> constraints;
};
struct EdgeData {
	int reference;
	vector<int> targets;


	bool has_inlier;
	ConstraintData inlier;

	vector< double > hyper_weights;
	vector< ConstraintMixtureData > hyper_constraints;

	bool operator<(const EdgeData& o) {
		if(reference < o.reference) {
			return true;
		} else if(reference > o.reference) {
			return false;
		}
		if(targets.size() < o.targets.size()) {
			return true;
		} else if( targets.size() > o.targets.size()) {
			return false;
		}
		for(size_t i=0; i<targets.size(); i++) {
			if(targets[i] < o.targets[i]) {
				return true;
			} else if(targets[i] > o.targets[i]) {
				return false;
			}
		}
		return false;
	}

	EdgeData() : has_inlier(false) {}

	inline size_t total_ambiguity() const {
		size_t a = 0;
		for(size_t i=0; i<hyper_constraints.size(); i++) {
			a += hyper_constraints[i].constraints.size();
		}
		if(targets.size()>1) {
			a+=1; // null hypothesis, all hyperedges have one
		}
	}
};

struct Graph {
	int dim;

	vector<int> vertex_ids;
	map< int, VertexData > vertices;

	vector< EdgeData > edges;

	Graph() : dim(-1) {}
};


bool parse_options(options& op, int argc, char** argv);
bool load(Graph&, const std::string& file);
bool load_outliers(Graph&, const std::string& file);
bool save_outliers(const Graph&, const std::string& file, const options&);

bool generate_outliers(Graph&, const options&);

template <typename RNG>
void sample_mean_and_store(int dim, double loop_variance_translation, double loop_variance_rotation, ConstraintData& c, double inside_confidence, double outside_confidence, RNG& gen);
template <typename RNG>
void sample_mean_and_store(int dim, double loop_variance_translation, double loop_variance_rotation, ConstraintData& c, const ConstraintMixtureData& existing, double inflation_factor, double inside_confidence, RNG& gen);

int main(int argc, char *argv[])
{
	options op;
	bool ok = parse_options(op, argc, argv);
	if(!ok) {
		return 1;
	}
	
	Graph G;
	ok = load(G, op.original_g2o_file );

	if(!ok) {
		cerr << "error reading original graph" << endl;
		return 2;
	}

	if(op.has_existing_outliers_file) {
		ok = load_outliers(G, op.existing_outliers_file);
		if(!ok) {
			cerr << "error reading existing outliers" << endl;
			return 3;
		}
	}

	ok = generate_outliers(G, op);

	if(!ok) {
		cerr << "error generating outliers?" << endl;
		return 4;
	}

	ok = save_outliers(G, op.output_file, op);

	if(!ok) {
		cerr << "error writing outliers" << endl;
		return 4;
	}	

	return 0;
}

template<typename T>
size_t find_element(const vector<T>& v, const T& t) {
	for(size_t i=0; i<v.size(); i++) {
		if( v[i] == t ) return i;
	}
	return v.size();
}

template < typename T >
size_t accumulate(const vector< vector<T> >& v, size_t k) {
	size_t s=0;
	for(size_t i=k; i<v.size(); i++) {
		s+=v[i].size();
	}
	return s;
}
size_t accumulate(const vector<int>& v, size_t k) {
	size_t s = 0;
	for(size_t i=k; i<v.size(); i++) {
		s+=v[i];
	}
	return s;
}


bool generate_outliers(Graph& G, const options& op) {
	cout << "Graph is of dim " << G.dim << ", has " << G.vertices.size() << " vertices, " << G.edges.size() << " edges" << endl << endl;

	boost::mt19937 gen(op.seed);


	// indices
	vector<size_t> simple_loop_inliers, simple_seq_inliers;
	set< pair<int,int> > already_connected_vertices;



	// find out how many loops and motions are there
	vector< vector<size_t> > loops_on_inliers;
	vector< vector<size_t> > loops_on_null;
	vector< vector< pair<size_t, size_t> > > motions; // first: edge index, second: loop (hyper) index

	for(size_t i = 0; i < G.edges.size(); i++) {
		const EdgeData& e = G.edges[i];

		for(size_t t=0; t<e.targets.size(); t++) {
			already_connected_vertices.insert( make_pair(e.reference, e.targets[t]) );
		}

		if(e.has_inlier && e.targets.size() == 1 && e.hyper_constraints.front().constraints.size() == 1) {
			if( e.reference +1 == e.targets[0] ) {
				simple_seq_inliers.push_back(i);
			} else {
				simple_loop_inliers.push_back(i);
			}
			continue;
		}

		size_t num_comp = e.targets.size();
		if(e.has_inlier && num_comp>=2) {
			size_t index = num_comp-2; // 2 because: 1 inlier, zero base, min will be 2
			if(loops_on_inliers.size() <= index ) {
				loops_on_inliers.resize(index+1);
			}
			loops_on_inliers[index].push_back(i);
		} else if( !e.has_inlier) {
			size_t index = num_comp-1; // 1 because: zero base, min will be 1
			if(loops_on_null.size() <= index ) {
				loops_on_null.resize(index+1);
			}
			loops_on_null[index].push_back(i);
		}

		for(size_t c=0; c<e.targets.size(); c++) {
			if( e.hyper_constraints[c].constraints.size() == 1 ) continue;

			size_t index = e.hyper_constraints[c].constraints.size() - 1;
			if(e.has_inlier && c == 0) {
				index--; // remove inlier comp
			}

			if( motions.size() <= index ) {
				motions.resize(index+1);
			}
			motions[index].push_back( make_pair(i, c) );
		}
	}

	{
		// summary
		cout << "BEFORE OUTLIER GENERATION:" << endl;
		cout << "graph has " << simple_loop_inliers.size() << " simple inlier loops, and " << simple_seq_inliers.size() << " simple sequential inliers" << endl;
		cout << "          outlier loops:"<< endl;
		if(!loops_on_null.empty()) {
			cout << "            on null   :" << endl;
			for(size_t i=0; i<loops_on_null.size(); i++) {
				cout << "              " << i+1 << ": " << loops_on_null[i].size() << endl;
			}
			//cout << endl;
		}
		if(!loops_on_inliers.empty()) {
			cout << "            on inliers:" << endl;
			for(size_t i=0; i<loops_on_inliers.size(); i++) {
				cout << "              " << i+2 << ": " << loops_on_inliers[i].size() << endl;
			}
			//cout << endl;
		}
		if(!motions.empty()) {
			cout << "          outlier motions:" << endl;
			for(size_t i=0; i<motions.size(); i++) {
				cout << "              " << i+1 << ": " << motions[i].size() << endl;
			}
		}
	}

	cout << endl;


	//!!! generate loops !!!

	//*** generate loops on inliers ***
	if(!op.false_loops_on_inliers.empty())
	{
		if( loops_on_inliers.size() < op.false_loops_on_inliers.size() ) {
			loops_on_inliers.resize( op.false_loops_on_inliers.size());
		}

		cout << "starting to generate false loops on inliers: " << endl;

		vector<size_t>* extensible = &simple_loop_inliers;
		for(size_t i=0; i<op.false_loops_on_inliers.size(); i++) {
			// generating i-hyper loops
			cout << "generating " << i+2 << "-loops: " << accumulate(op.false_loops_on_inliers,i) - accumulate(loops_on_inliers,i) << " (have " << accumulate(loops_on_inliers,i) << ", need " << accumulate(op.false_loops_on_inliers,i) << ")" << endl;
			cout << "have a set of " << (*extensible).size() << " to extend" << endl;

			while(accumulate(loops_on_inliers,i) < accumulate(op.false_loops_on_inliers,i)) {
				// pick random inlier loop
				boost::uniform_int<size_t> random_index(0,(*extensible).size()-1);
				size_t index = random_index(gen);

				if( G.edges[(*extensible)[index]].reference +1 == G.edges[(*extensible)[index]].targets.front() ) {
					cerr << "FOUND seq edge while looking for loops!" << endl;
					continue;
				}

				size_t ref_index = find_element(G.vertex_ids, G.edges[(*extensible)[index]].reference);
				if(ref_index == G.vertex_ids.size()) {
					cerr << "could not find vertex " <<  G.edges[(*extensible)[index]].reference << endl;
					continue;
				}

				size_t lower_limit = 0;
				if(op.local_loops) {
					lower_limit = ref_index-op.local_neighborhood;
					if(lower_limit > ref_index ) { // wrapped
						lower_limit = 0;
					}
				}
				size_t upper_limit = G.vertex_ids.size()-1;
				if(op.local_loops) {
					upper_limit = ref_index+op.local_neighborhood;
					if(upper_limit >= G.vertex_ids.size() ) { // cap
						upper_limit = G.vertex_ids.size()-1;
					}
				}

				boost::uniform_int<size_t> random_target(lower_limit,upper_limit);
				size_t tar_index = random_target(gen);
				while(     already_connected_vertices.find(make_pair(G.vertex_ids[ref_index], G.vertex_ids[tar_index])) != already_connected_vertices.end()
						|| already_connected_vertices.find(make_pair(G.vertex_ids[tar_index], G.vertex_ids[ref_index])) != already_connected_vertices.end()
						|| tar_index == ref_index
						|| tar_index+1 == ref_index
						|| ref_index+1 == tar_index
				 ) {
					tar_index = random_target(gen);
				}

				
				EdgeData& e = G.edges[(*extensible)[index]];
				

				int target = G.vertex_ids[tar_index];
				e.targets.push_back(target);



				ConstraintData c;
				c.reference = e.reference;
				c.target = target;
				sample_mean_and_store(G.dim, op.loop_tr_variance, op.loop_rot_variance, c, .1, .7, gen);

				// "emplace"
				e.hyper_constraints.resize(e.hyper_constraints.size()+1);
				e.hyper_weights.resize(e.hyper_weights.size()+1);

				boost::uniform_real<> random_weight(op.min_weight, op.max_weight);
				double weight = random_weight(gen);

				e.hyper_weights.back() = weight;
				e.hyper_constraints.back().weights.push_back(1.0); // first in this hypercomp
				e.hyper_constraints.back().constraints.push_back(c);

				// remember
				already_connected_vertices.insert( make_pair( c.reference, c.target ) );

				loops_on_inliers[i].push_back((*extensible)[index]);
				(*extensible).erase( (*extensible).begin()+index );
			}

			cout << "done generating " << i+2 << "-loops on inliers" << endl;

			extensible = &(loops_on_inliers[i]); // swap out
		}

		cout << "done generating loops on inliers" << endl << endl;
	}

	//*** generate loops on null ***
	if(!op.false_loops.empty())
	{
		if( loops_on_null.size() < op.false_loops.size() ) {
			loops_on_null.resize( op.false_loops.size());
		}

		cout << "starting to generate false loops: " << endl;

		{
			const size_t i = 0;
			cout << "generating 1-loops: " << accumulate(op.false_loops,i) - accumulate(loops_on_null,i) << " (have " << accumulate(loops_on_null,i) << ", need " << accumulate(op.false_loops,i) << ")" << endl;
			cout << "don't need a set to extend, inventing new edges here" << endl;

			while(accumulate(loops_on_null,i) < accumulate(op.false_loops,i)) {
				boost::uniform_int<size_t> random_reference(0,G.vertex_ids.size()-1);
				size_t ref_index = random_reference(gen);

				if(op.group_loops) {
					if(ref_index > G.vertex_ids.size()-1 -op.group_size) { // need enough space for group to complete
						continue;
					}
				}

				size_t lower_limit = 0;
				if(op.local_loops) {
					lower_limit = ref_index-op.local_neighborhood;
					if(lower_limit > ref_index ) { // wrapped
						lower_limit = 0;
					}
				}
				size_t upper_limit = G.vertex_ids.size()-1;
				if(op.local_loops) {
					upper_limit = ref_index+op.local_neighborhood;
					if(upper_limit >= G.vertex_ids.size() ) { // cap
						upper_limit = G.vertex_ids.size()-1;
					}
				}

				boost::uniform_int<size_t> random_target(lower_limit,upper_limit);
				size_t tar_index = random_target(gen);
				if(     already_connected_vertices.find(make_pair(G.vertex_ids[ref_index], G.vertex_ids[tar_index])) != already_connected_vertices.end()
						|| already_connected_vertices.find(make_pair(G.vertex_ids[tar_index], G.vertex_ids[ref_index])) != already_connected_vertices.end()
						|| tar_index == ref_index
				 ) {
					continue;
				}

				// make new edge
				EdgeData e; 
				e.has_inlier = false;
				e.reference = G.vertex_ids[ref_index];
				e.targets.push_back( G.vertex_ids[tar_index] );
				ConstraintData c;
				c.reference = e.reference;
				c.target = e.targets.front();
				sample_mean_and_store(G.dim, op.loop_tr_variance, op.loop_rot_variance, c, .1, .7, gen);

				// "emplace"
				e.hyper_weights.push_back(1.0);
				e.hyper_constraints.resize(1);

				e.hyper_constraints.front().weights.push_back(1.0);
				e.hyper_constraints.front().constraints.push_back(c);

				G.edges.push_back( e );
				already_connected_vertices.insert( make_pair( e.reference, e.targets.front() ) );

				loops_on_null[0].push_back(G.edges.size()-1);

				// use the the group strategy
				if(op.group_loops) {
					for(int i=0; i<op.group_size-1; i++) {
						// we copied before, so it's ok to just modify e and push_back again

						ref_index++; tar_index++;
						if( ref_index >= G.vertex_ids.size() || tar_index >= G.vertex_ids.size() ) {
							// out of range
							cerr << "couldn't finish group, out of vertices" << endl;
							break;
						}
						e.reference = G.vertex_ids[ref_index];
						e.targets.front() = G.vertex_ids[tar_index];
						e.hyper_constraints.front().constraints.front().reference = e.reference;
						e.hyper_constraints.front().constraints.front().target = e.targets.front();

						sample_mean_and_store(G.dim, op.loop_tr_variance, op.loop_rot_variance, e.hyper_constraints.front().constraints.front(), .1, .7, gen);

						G.edges.push_back( e );
						already_connected_vertices.insert( make_pair( e.reference, e.targets.front() ) );

						loops_on_null[0].push_back(G.edges.size()-1);
					}
				}
			}
		}

		cout << "done generating 1-loops" << endl;

		vector<size_t>* extensible = &(loops_on_null[0]);
		for(size_t i=1; i<op.false_loops.size(); i++) {
			// generating i-hyper loops
			cout << "generating " << i+1 << "-loops: " << accumulate(op.false_loops,i) - accumulate(loops_on_null,i) << " (have " << accumulate(loops_on_null,i) << ", need " << accumulate(op.false_loops,i) << ")" << endl;
			cout << "have a set of " << (*extensible).size() << " to extend" << endl;

			while(accumulate(loops_on_null,i) < accumulate(op.false_loops,i)) {
				// pick random inlier loop
				boost::uniform_int<size_t> random_index(0,(*extensible).size()-1);
				size_t index = random_index(gen);

				size_t ref_index = find_element(G.vertex_ids, G.edges[(*extensible)[index]].reference);
				if(ref_index == G.vertex_ids.size()) {
					cerr << "could not find vertex " <<  G.edges[(*extensible)[index]].reference << endl;
					continue;
				}

				size_t lower_limit = 0;
				if(op.local_loops) {
					lower_limit = ref_index-op.local_neighborhood;
					if(lower_limit > ref_index ) { // wrapped
						lower_limit = 0;
					}
				}
				size_t upper_limit = G.vertex_ids.size()-1;
				if(op.local_loops) {
					upper_limit = ref_index+op.local_neighborhood;
					if(upper_limit >= G.vertex_ids.size() ) { // cap
						upper_limit = G.vertex_ids.size()-1;
					}
				}

				boost::uniform_int<size_t> random_target(lower_limit,upper_limit);
				size_t tar_index = random_target(gen);
				while(     already_connected_vertices.find(make_pair(G.vertex_ids[ref_index], G.vertex_ids[tar_index])) != already_connected_vertices.end()
						|| already_connected_vertices.find(make_pair(G.vertex_ids[tar_index], G.vertex_ids[ref_index])) != already_connected_vertices.end()
						|| tar_index == ref_index
				 ) {
					tar_index = random_target(gen);
				}

				
				EdgeData& e = G.edges[(*extensible)[index]];

				int target = G.vertex_ids[tar_index];
				e.targets.push_back(target);
				ConstraintData c;
				c.reference = e.reference;
				c.target = target;
				sample_mean_and_store(G.dim, op.loop_tr_variance, op.loop_rot_variance, c, .1, .7, gen);

				// "emplace"
				e.hyper_weights.resize(e.hyper_constraints.size()+1);
				e.hyper_constraints.resize(e.hyper_constraints.size()+1);

				boost::uniform_real<> random_weight(op.min_weight, op.max_weight);
				double weight = random_weight(gen);

				e.hyper_weights.back() = weight;
				e.hyper_constraints.back().weights.push_back(1.0); // first component here
				e.hyper_constraints.back().constraints.push_back(c);

				loops_on_null[i].push_back((*extensible)[index]);
				(*extensible).erase( (*extensible).begin()+index );
			}

			extensible = &(loops_on_null[i]); // swap out

			cout << "done generating " << i+1 << "-loops" << endl;
		}

		cout << "done generating false loops" << endl << endl;
	}

	//*** generate motions ***
	if(!op.false_motions.empty()) {

		if( motions.size() < op.false_motions.size() ) {
			motions.resize( op.false_motions.size());
		}

		cout << "starting to generate false motions: " << endl;

		vector< pair< size_t, size_t > > base_edges; base_edges.reserve(G.edges.size());
		if(op.combine_false_loops_and_motions) {
			for(size_t i=0; i<G.edges.size(); i++) {
				EdgeData& e = G.edges[i];
				for(size_t c=0; c<e.targets.size(); c++) {
					if(e.hyper_constraints[c].constraints.size() == 1) {
						base_edges.push_back( make_pair(i,c) );
					}
				}
			}
		} else { // motions only on simple sequential and loop edges
			for(size_t i=0; i<simple_seq_inliers.size(); i++) {
				base_edges.push_back( make_pair( simple_seq_inliers[i] ,0) );
			}
			/*
			for(size_t i=0; i<simple_loop_inliers.size(); i++) {
				base_edges.push_back( make_pair( simple_loop_inliers[i] ,0) );
			}
			*/
		}

		vector< pair< size_t, size_t > >* extensible = &base_edges;

		for(size_t i=0; i<op.false_motions.size(); i++) {
			// generating i-hyper loops
			cout << "generating " << i+1 << "-motions: " << accumulate(op.false_motions,i) - accumulate(motions,i)  << " (have " << accumulate(motions,i) << ", need " << accumulate(op.false_motions,i) << ")" << endl;
			cout << "have a set of " << (*extensible).size() << " to extend" << endl;

			while(accumulate(motions,i) < accumulate(op.false_motions,i)) {
				// pick random inlier constraint
				boost::uniform_int<size_t> random_index(0,(*extensible).size()-1);
				size_t index = random_index(gen);

	
				EdgeData& e = G.edges[ (*extensible)[index].first ];
				size_t j = (*extensible)[index].second;

				int target = e.targets[j];

				ConstraintData c;
				c.reference = e.reference;
				c.target = target;

				sample_mean_and_store(G.dim, op.motion_tr_variance, op.motion_rot_variance, c, e.hyper_constraints[j], op.cov_inflation_factor_for_motion_generation, op.min_confidence_distance_motions, gen);

				// "emplace"
				double min_weight = op.min_weight;
				double max_weight = op.max_weight;

				if( e.hyper_constraints[j].weights.size() == 1 && op.ensure_first_false_motion_has_larger_weight_than_inlier && min_weight < e.hyper_constraints[j].weights.front() ) {
					min_weight = e.hyper_constraints[j].weights.front() + std::numeric_limits<double>::epsilon();
					if(max_weight < e.hyper_constraints[j].weights.front()) {
						max_weight += min_weight - op.min_weight;
					}
				}

				boost::uniform_real<> random_weight(min_weight, max_weight);
				double weight = random_weight(gen);

				e.hyper_constraints[j].weights.push_back(weight);
				e.hyper_constraints[j].constraints.push_back(c);

				motions[i].push_back( (*extensible)[index] );
				(*extensible).erase( (*extensible).begin()+index );
			}

			extensible = &(motions[i]); // swap out
			cout << "done generating " << i+1 << "-motions" << endl;
		}


		cout << "done generating false motions" << endl << endl;
	}

	// rebuild for summary
	simple_seq_inliers.clear();
	simple_loop_inliers.clear();
	loops_on_inliers.clear();
	loops_on_null.clear();
	motions.clear();

	for(size_t i = 0; i < G.edges.size(); i++) {
		const EdgeData& e = G.edges[i];

		if(e.has_inlier && e.targets.size() == 1 && e.hyper_constraints.front().constraints.size() == 1) {
			if( e.reference +1 == e.targets[0] ) {
				simple_seq_inliers.push_back(i);
			} else {
				simple_loop_inliers.push_back(i);
			}
		}

		size_t num_comp = e.targets.size();
		if(e.has_inlier && num_comp>=2) {
			size_t index = num_comp-2; // 2 because: 1 inlier, zero base, min will be 2
			if(loops_on_inliers.size() <= index ) {
				loops_on_inliers.resize(index+1);
			}
			loops_on_inliers[index].push_back(i);
		} else if( !e.has_inlier) {
			size_t index = num_comp-1; // 1 because: zero base, min will be 1
			if(loops_on_null.size() <= index ) {
				loops_on_null.resize(index+1);
			}
			loops_on_null[index].push_back(i);
		}

		for(size_t c=0; c<e.targets.size(); c++) {
			if( e.hyper_constraints[c].constraints.size() == 1 ) continue;

			size_t index = e.hyper_constraints[c].constraints.size() - 1;
			if(e.has_inlier && c == 0) {
				index--; // remove inlier comp
			}

			if( motions.size() <= index ) {
				motions.resize(index+1);
			}
			motions[index].push_back( make_pair(i, c) );
		}
	}

	{
		// summary
		cout << "AFTER OUTLIER GENERATION:" << endl;
		cout << "graph has " << simple_loop_inliers.size() << " simple inlier loops, and " << simple_seq_inliers.size() << " simple sequential inliers" << endl;
		cout << "          outlier loops:" << endl;
		if(!loops_on_null.empty()) {
			cout << "            on null   :" << endl;
			for(size_t i=0; i<loops_on_null.size(); i++) {
				cout << "              " << i+1 << ": " << loops_on_null[i].size() << endl;
			}
			//cout << endl;
		}
		if(!loops_on_inliers.empty()) {
			cout << "            on inliers:" << endl;
			for(size_t i=0; i<loops_on_inliers.size(); i++) {
				cout << "              " << i+2 << ": " << loops_on_inliers[i].size() << endl;
			}
			//cout << endl;
		}
		if(!motions.empty()) {
			cout << "          outlier motions:" << endl;
			for(size_t i=0; i<motions.size(); i++) {
				cout << "              " << i+1 << ": " << motions[i].size() << endl;
			}
		}
	}

	return true;
}

template< typename Group, typename RNG >
Group sample_mean_with(const ConstraintData& c, double inside_confidence, double outside_confidence, RNG& gen) {
	typename NormalDistributionOn<Group>::Covariance Sigma = c.cov;
	NormalDistributionOn<Group> nd(Group::Tangent::Zero(), Sigma);

	typename SampleTraits< NormalDistributionOn<Group> >::Sampler sampler(nd);

	Group s = sampler(gen);
	while( inside_confidence_region(nd,s,inside_confidence) || !inside_confidence_region(nd,s,outside_confidence) ) {
		s = sampler(gen);
	}
	return s;
}

template < typename Derived >
void fill_values(SE2d& g, const Eigen::MatrixBase<Derived>& m) {
	g = SE2d(m[2], SE2d::Point(m[0],m[1]));
}
template < typename Derived >
void fill_values(SE3d& g, const Eigen::MatrixBase<Derived>& m) {
	Eigen::Matrix< double, 4, 1> mat; mat = m.tail(4);
	g = SE3d(
				Eigen::Quaternion< double >( mat ),
				m.head(3)
			);
}

template< typename Group, typename RNG >
Group sample_mean_with_constraint(const ConstraintData& c, const ConstraintMixtureData& existing, double inflation_factor, double inside_confidence, RNG& gen) {
	MixtureOf< NormalDistributionOn<Group> > mix;

	double mean_distance = 0;

	for(size_t i=0; i<existing.constraints.size(); i++) {
		typename NormalDistributionOn<Group>::Covariance Sigma = existing.constraints[i].cov;
		Group mean;
		fill_values(mean,existing.constraints[i].mean);
		NormalDistributionOn<Group> nd(mean, Sigma);

		mean_distance += existing.constraints[i].mean.norm();

		mix.addComponent(existing.weights[i], nd);
	}
	mix.normalizeWeights();

	mean_distance /= existing.constraints.size();

	

	typename NormalDistributionOn<Group>::Covariance Sigma = c.cov;

	//NormalDistributionOn<Group> sample_n( Group::Tangent::Zero(), Sigma * inflation_factor);
	//typename SampleTraits< NormalDistributionOn<Group> >::Sampler sampler(sample_n);

	Random::uniform_group<Group> sampler( Group::Point::Constant(mean_distance * inflation_factor) );

	Group s = sampler(gen);

	// cout << "original sampled group: " << flush;
	// for(int i=0; i<Group::num_parameters; i++) {
	// 	if(i!=0) cout << ",";
	// 	cout << s.data()[i] << flush;
	// }
	// cout << endl;

	//cout << "starting sample, inside_confidence=" << inside_confidence << endl;

	
	NormalDistributionOn<Group> nd(s,Sigma);

	//size_t count = 0;
	while( inside_confidence_region(mix,s,inside_confidence)  || inside_confidence_region( nd, mix.component(max_component(mix,s)).mean(), inside_confidence) )
	{
		s = sampler(gen);

		//count++; if(count%1000 == 0) cout << "." << flush;
	}
	//cout << "Made sample: " << s.log().transpose() << endl;

	return s;
}

template <typename RNG>
void sample_mean_and_store(int dim, double variance_translation, double variance_rotation, ConstraintData& c, const ConstraintMixtureData& existing, double inflation_factor, double inside_confidence, RNG& gen) {
	if(dim == 2) {
		c.cov = MatrixXd::Zero(3,3);
		c.cov(0,0) = c.cov(1,1) = variance_translation;
		c.cov(2,2) = variance_rotation;
		c.inf = c.cov.inverse();

		SE2d s = sample_mean_with_constraint<SE2d>(c, existing, inflation_factor, inside_confidence, gen);

		c.mean.resize(3);
		c.mean[0] = s.translation()[0];
		c.mean[1] = s.translation()[1];
		c.mean[2] = s.so2().log(); // theta
	} else {
		c.cov = MatrixXd::Zero(6,6);
		c.cov(0,0) = c.cov(1,1) = c.cov(2,2) = variance_translation;
		c.cov(3,3) = c.cov(4,4) = c.cov(5,5) = variance_rotation;
		c.inf = c.cov.inverse();

		SE3d s = sample_mean_with_constraint<SE3d>(c, existing, inflation_factor, inside_confidence, gen);

		c.mean.resize(7);
		c.mean[0] = s.translation()[0];
		c.mean[1] = s.translation()[1];
		c.mean[2] = s.translation()[2];

		c.mean[3] = s.so3().unit_quaternion().x();
		c.mean[4] = s.so3().unit_quaternion().y();
		c.mean[5] = s.so3().unit_quaternion().z();
		c.mean[6] = s.so3().unit_quaternion().w();
	}
}

template <typename RNG>
void sample_mean_and_store(int dim, double loop_variance_translation, double loop_variance_rotation, ConstraintData& c, double inside_confidence, double outside_confidence, RNG& gen) {
	if(dim == 2) {
		c.cov = MatrixXd::Zero(3,3);
		c.cov(0,0) = c.cov(1,1) = loop_variance_translation;
		c.cov(2,2) = loop_variance_rotation;
		c.inf = c.cov.inverse();

		SE2d s = sample_mean_with<SE2d>(c, inside_confidence, outside_confidence, gen);

		c.mean.resize(3);
		c.mean[0] = s.translation()[0];
		c.mean[1] = s.translation()[1];
		c.mean[2] = s.so2().log();
	} else {
		c.cov = MatrixXd::Zero(6,6);
		c.cov(0,0) = c.cov(1,1) = c.cov(2,2) = loop_variance_translation;
		c.cov(3,3) = c.cov(4,4) = c.cov(5,5) = loop_variance_rotation;
		c.inf = c.cov.inverse();

		SE3d s = sample_mean_with<SE3d>(c, inside_confidence, outside_confidence, gen);
		c.mean.resize(7);
		c.mean[0] = s.translation()[0];
		c.mean[1] = s.translation()[1];
		c.mean[2] = s.translation()[2];

		c.mean[3] = s.so3().unit_quaternion().x();
		c.mean[4] = s.so3().unit_quaternion().y();
		c.mean[5] = s.so3().unit_quaternion().z();
		c.mean[6] = s.so3().unit_quaternion().w();
	}
}

bool load(Graph& G, const std::string& file){
	ifstream in(file.c_str());

	if(!in.good()) {
		return false;
	}

	while(in.good()) {
		string line;
		getline(in, line);

		if( line.size() < 3 || line[0] == '#' ) {
			continue;
		}

		istringstream lin(line);

		string tag; lin>>tag;

		if( tag.substr(0,7) == "VERTEX_" ) {
			VertexData v;

			lin >> v.id;

			if(G.dim == -1) {
				if(tag.substr(7) == "SE2") {
					G.dim = 2;
				} else if(tag.substr(7) == "SE3:QUAT") {
					G.dim = 3;
				} else {
					cerr << "UNKNOWN vertex type: " << tag << endl;
					return false;
				}
				cout << "reading graph with dim " << G.dim << endl;
			}

			if( G.dim == 2 ) {
				v.ground_truth_pose.resize(3);
			} else {
				v.ground_truth_pose.resize(7);
			}

			for(size_t i=0; i<v.ground_truth_pose.innerSize(); i++) {
				lin >> v.ground_truth_pose[i];
			}

			G.vertices[v.id] = v;
			G.vertex_ids.push_back(v.id);
		} else if( tag.substr(0,5) == "EDGE_" ) {
			// sanity
			if( G.dim == 2 && tag.substr(5) != "SE2" ) {
				cerr << "found unknown edge! '" << tag << "'" << endl;
				return false;
			}
			if( G.dim == 3 && tag.substr(5) != "SE3:QUAT" ) {
				cerr << "found unknown edge! '" << tag << "'" << endl;
				return false;
			}

			if( tag != "EDGE_SE2" && tag != "EDGE_SE3:QUAT" ) {
				cerr << "unknown edge type! '" << tag << "'" << endl;
				return false; 
			}

			EdgeData e;
			lin >> e.reference;

			e.has_inlier = true;
			e.inlier.reference = e.reference;
			lin >> e.inlier.target;


			if( G.vertices.find(e.inlier.reference) == G.vertices.end() || G.vertices.find(e.inlier.target) == G.vertices.end() ) {
				cerr << "did not find both vertices for simple edge: " << e.inlier.reference << " to " << e.inlier.target << endl;
				continue;
			}

			if( G.dim == 2) {
				e.inlier.mean.resize(3);
				e.inlier.inf.resize(3,3);
				e.inlier.cov.resize(3,3);
			} else {
				e.inlier.mean.resize(7);
				e.inlier.inf.resize(6,6);
				e.inlier.cov.resize(6,6);
			}

			for(size_t i=0; i<e.inlier.mean.innerSize(); i++) {
				lin >> e.inlier.mean[i];
			}
			for(size_t i=0; i<e.inlier.inf.innerSize(); i++) {
				for(size_t j=i; j<e.inlier.inf.outerSize(); j++) {
					lin >> e.inlier.inf(i,j);
					if(i != j)
						e.inlier.inf(j,i) = e.inlier.inf(i,j);
				}
			}

			e.inlier.cov = e.inlier.inf.inverse();

			e.targets.push_back(e.inlier.target);

			e.hyper_constraints.resize(1);
			e.hyper_weights.push_back(1.0);
			e.hyper_constraints[0].weights.push_back(1.0);
			e.hyper_constraints[0].constraints.push_back(e.inlier);


			G.edges.push_back(e);
		}
	}
	return true;
}
bool load_outliers(Graph& G, const std::string& file) {
	ifstream in(file.c_str());

	if(!in.good()) {
		return false;
	}

	EdgeData* e=NULL;
	size_t current_hyper_comp=-1;
	double next_weight = -1;

	while(in.good()) {
		string line;
		getline(in,line);

		if(line.size()<3 || line[0]=='#') {
			continue;
		}

		istringstream lin(line);

		string tag; lin >> tag;
		if( tag == "LOOP_OUTLIER_BATCH" ) { // start new batch
			if(e != NULL) {
				cerr << "starting loop outlier batch with active edge, should not have one." << endl;
				return false;
			}

			int ref;
			bool has_inlier, has_null;
			int inlier_target;

			lin >> ref >> has_null >> has_inlier >> inlier_target;

			if( has_inlier ) {
				for(size_t i=0; i<G.edges.size(); i++) {
					if( G.edges[i].reference == ref && G.edges[i].has_inlier && G.edges[i].inlier.target == inlier_target) {
						e = &(G.edges[i]);
						break;
					}
				}

				if(e==NULL) {
					cerr << "could not find corresponding edge to outlier! ref:" << ref << ", tar:" << inlier_target << endl;
					return false;
				}
			} else {
				EdgeData ne;
				ne.reference = ref;
				ne.has_inlier = false;

				G.edges.push_back(ne);
				e = &(G.edges.back());
			}

		} else if( tag == "LOOP_OUTLIER_BATCH_END" ) {
			e = NULL;
		} else if( tag == "MOTION_OUTLIER_BATCH" ) {
			if(e == NULL ) {
				cerr << "starting motion outlier batch without active edge!" << endl;
				return false;
			}

			int target; double weight;
			lin >> target >> weight;

			current_hyper_comp = -1;

			// search for correct hypercomponent
			for(size_t h=0; h<e->targets.size(); h++) {
				if( e->targets[h] == target ) {
					current_hyper_comp = h;
					break;
				}
			}

			if(current_hyper_comp == -1) {
				// make new
				e->targets.push_back(target);
				e->hyper_weights.push_back(weight);
				e->hyper_constraints.resize( e->hyper_constraints.size()+1 );

				current_hyper_comp = e->hyper_constraints.size()-1;
			}

		} else if( tag == "MOTION_OUTLIER_BATCH_END" ) {
			current_hyper_comp = -1;
		} else if( tag == "MOTION_WEIGHT") {
			lin >> next_weight;
		} else if( tag == "EDGE_SE2" || tag == "EDGE_SE3:QUAT") {
			if(e == NULL ) {
				cerr << "reading edge data without active edge!" << endl;
				return false;
			}
			if(current_hyper_comp == -1) {
				cerr << "reading edge data without active component!" << endl;
				return false;
			}

			if( (tag == "EDGE_SE2" && G.dim != 2) || (tag == "EDGE_SE3:QUAT" && G.dim != 3) ) {
				cerr << "edge type '" << tag << "' does not match graph dimension (" << G.dim << ")!" << endl;
				return false;
			}

			ConstraintData c;
			lin >> c.reference >> c.target;
			if( G.dim == 2) {
				c.mean.resize(3);
				c.inf.resize(3,3);
				c.cov.resize(3,3);
			} else {
				c.mean.resize(7);
				c.inf.resize(6,6);
				c.cov.resize(6,6);
			}

			for(size_t i=0; i<c.mean.innerSize(); i++) {
				lin >> c.mean[i];
			}
			for(size_t i=0; i<c.inf.innerSize(); i++) {
				for(size_t j=i; j<c.inf.outerSize(); j++) {
					lin >> c.inf(i,j);
					if(i != j)
						c.inf(j,i) = c.inf(i,j);
				}
			}
			c.cov = c.inf.inverse();

			e->hyper_constraints[current_hyper_comp].weights.push_back(next_weight);
			e->hyper_constraints[current_hyper_comp].constraints.push_back(c);

		}
	}

	return true;
}
bool save_outliers(const Graph& G, const std::string& file, const options& op) {

	ofstream out(file.c_str());
	if(!out.good()) {
		return false;
	}


	// maybe output options, later

	// write out only OUTLIERS
	for(size_t i = 0; i < G.edges.size(); i++) {
		const EdgeData& e = G.edges[i];

		if(e.has_inlier && e.targets.size() == 1 && e.hyper_constraints.front().constraints.size() == 1) {
			// inlier only
			continue;
		}

		bool has_null_hypothesis = true;
		if( e.targets.size() == 1 && e.has_inlier ) {
			has_null_hypothesis = false;
		}

		out << "LOOP_OUTLIER_BATCH " << e.reference << " " << has_null_hypothesis << " " << e.has_inlier << " " << e.inlier.target << endl; // 1 means it does have a null hypothesis
		

		for(size_t h=0; h<e.hyper_constraints.size(); h++) {
			if(e.has_inlier && h==0 && e.hyper_constraints[h].constraints.size()==1) {
				// this is the inlier hyper component
				continue;
			}

			out << "MOTION_OUTLIER_BATCH " << e.targets[h] << " " << boost::lexical_cast<string>(e.hyper_weights[h]) << endl;

			for(size_t c=0; c<e.hyper_constraints[h].constraints.size(); c++) {
				if( e.has_inlier && c==0 && h==0 ) { // this is the inlier
					continue;
				}
				out << "MOTION_WEIGHT " << boost::lexical_cast<string>(e.hyper_constraints[h].weights[c]) << endl;
				if(G.dim == 2 ) {
					out << "EDGE_SE2 ";
				} else {
					out << "EDGE_SE3:QUAT ";
				}
				out << e.reference << " " << e.targets[h];
				for(int j=0; j< e.hyper_constraints[h].constraints[c].mean.innerSize(); j++) {
					out << " " << boost::lexical_cast<string>(e.hyper_constraints[h].constraints[c].mean[j]);
				}

				for(int j=0; j<e.hyper_constraints[h].constraints[c].inf.innerSize(); j++)
				for(int k=j; k<e.hyper_constraints[h].constraints[c].inf.outerSize(); k++) {
					out << " " << boost::lexical_cast<string>(e.hyper_constraints[h].constraints[c].inf(j,k));
				}
				out << endl;
			}

			out << "MOTION_OUTLIER_BATCH_END" << endl;
		}

		out << "LOOP_OUTLIER_BATCH_END" << endl << endl;
	}

	return true;
}


bool parse_options(options& op, int argc, char** argv) {
	po::options_description desc("options");
	desc.add_options()
	    ("help,h", "produce help message")
	    ("input", po::value<string>(&op.original_g2o_file), "input g2o file")
	    ("output", po::value<string>(&op.output_file), "output file of outliers")
	    ("previous-outliers", po::value<string>(&op.existing_outliers_file), "already generated outliers to extend")
	    ("seed", po::value<uint32_t>(&op.seed), "random number generator seed, for repeatability")
	    ("false-loops", po::value< vector<int> >(&op.false_loops)->multitoken(), "Add this many false loop constraints (i.e. global ambiguity, connecting previously unconnected vertices). The NULL hypothesis is correct. If it is a list (e.g. '100 50') the loops stack, so there 100 vetices where a single false loop starts, and 50 where two false loops starts, and so on.")
	    ("false-loops-on-inliers", po::value< vector<int> >(&op.false_loops_on_inliers)->multitoken(), "Like --false-loops, but adds these false loops next to existing inlier loops. Here, the NULL hypothesis is false, the original inlier is correct. Again, they stack. '100 50' would generate 100 constraints where two loops start, one inlier, one outlier, and 50 where three loops start, one inlier, two outliers, etc.")
	    ("false-motions", po::value< vector<int> >(&op.false_motions)->multitoken(), "Add this many false motion constraints (i.e. local ambiguity, on already existing edges). Default = 0. Again, if it is a list as above, these stack as well, so the script generates multiple false constraints for a single pair of vertices.")
	    ("combine", po::value< bool >(&op.combine_false_loops_and_motions)->default_value(op.combine_false_loops_and_motions)->zero_tokens(), "Combine false loops with false motions, i.e. false loops can also have additionally false motions (combined local and global ambiguity).")
	    ("local-loops", po::value< bool >(&op.local_loops)->default_value(op.local_loops)->zero_tokens(), "Use 'local' loop generation policy, where the second vertex is randomly chosen from a neighborhood of the first vertex. Can be combined with --group-loops. (see Niko Suenderhauf's PhD thesis).")
	    ("local-neighborhood", po::value<int>(&op.local_neighborhood)->default_value(op.local_neighborhood), "Size of local neighborhood for local policy.")
	    ("group-loops", po::value< bool >(&op.group_loops)->default_value(op.group_loops)->zero_tokens(), "Use 'grouped' loop generation policy, where the a number of loops are added for consecutive vertices. Can be combined with --local-loops. (see Niko Suenderhauf's PhD thesis).")
	    ("group-size", po::value<int>(&op.group_size)->default_value(op.group_size), "Size of group for grouped policy.")
	    ("loop-variance-translation", po::value<double>(&op.loop_tr_variance)->default_value(op.loop_tr_variance), "Translation variance for false loops.")
	    ("loop-variance-rotation", po::value<double>(&op.loop_rot_variance)->default_value(op.loop_rot_variance), "Rotation variance for false loops.")
	    ("motion-variance-translation", po::value<double>(&op.motion_tr_variance)->default_value(op.motion_tr_variance), "Translation variance for false motions.")
	    ("motion-variance-rotation", po::value<double>(&op.motion_rot_variance)->default_value(op.motion_rot_variance), "Rotation variance for false motions.")
	    ("min-motion-distance", po::value<double>(&op.min_confidence_distance_motions)->default_value(op.min_confidence_distance_motions), "Seperate false motions for the same vertex pair should lie outside each others confidence ellipse of this probability. Default: .95")
	    ("inflation-factor", po::value<double>(&op.cov_inflation_factor_for_motion_generation)->default_value(op.cov_inflation_factor_for_motion_generation), "Inflate motion randomness by this factor. If motion generation takes a long time, increase this. Also makes for more dispersed motions. Default: 1.5")
	    ("min-weight", po::value<double>(&op.min_weight)->default_value(op.min_weight), "Minimum random weight for mixtures, first weight is always 1.0. Default: 0.5")
	    ("max-weight", po::value<double>(&op.max_weight)->default_value(op.max_weight), "Maximum random weight for mixtures, first weight is always 1.0. Default: 2.0")
	    ("first-motion-wins", po::value< bool >(&op.ensure_first_false_motion_has_larger_weight_than_inlier)->default_value(op.ensure_first_false_motion_has_larger_weight_than_inlier)->zero_tokens(), "Make weight of first false motion greater than original inlier weight (i.e. more than 1) no matter the value of min-weight and max-weight.")
	;

	po::positional_options_description pd;
	pd.add("input", 1);
	pd.add("output", 2);

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).
          options(desc).positional(pd).run(), vm);
	po::notify(vm);    

	if (vm.count("help")) {
	    cout << desc << "\n";
	    return false;
	}
	
	// more validation

	if(vm.count("input") == 0 || vm.count("output") == 0) {
		cerr << "Need to specify both input and output file names!" << endl;
		cerr << desc << endl;
		return false;
	}

	op.has_existing_outliers_file = vm.count("previous-outliers") != 0;

	if( op.false_motions.empty() && op.false_loops.empty() && op.false_loops_on_inliers.empty() ) {
		cerr << "You did not ask for any outliers to be generated!" << endl;
		cerr << desc << endl;
		return false;
	}

	return true;
}
