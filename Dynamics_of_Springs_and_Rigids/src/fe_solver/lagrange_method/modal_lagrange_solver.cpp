#include "modal_lagrange_solver.h"

modal_lagrange_solver::modal_lagrange_solver()
{
	// Empty constructor
}

modal_lagrange_solver::~modal_lagrange_solver()
{
	// Empty destructor
}

void modal_lagrange_solver::clear_results()
{

}

void modal_lagrange_solver::modal_analysis_lagrangemethod_start(const nodes_list_store& model_nodes, 
	const elementline_list_store& model_lineelements, 
	const nodeconstraint_list_store& node_constraints, 
	const nodepointmass_list_store& node_ptmass, 
	const std::unordered_map<int, material_data>& material_list, 
	modal_nodes_list_store& modal_result_nodes, 
	modal_elementline_list_store& modal_result_lineelements, 
	bool& is_modal_analysis_complete)
{

	// Main solver call
	is_modal_analysis_complete = false;

	// Check the model
	// Number of nodes
	if (model_nodes.node_count == 0)
	{
		return;
	}

	// Number of elements
	if (model_lineelements.elementline_count == 0)
	{
		return;
	}

	// Number of point mass
	if (node_ptmass.ptmass_count == 0)
	{
		return;
	}

	//____________________________________________
	Eigen::initParallel();  // Initialize Eigen's thread pool

	stopwatch.start();

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << std::fixed << std::setprecision(6);

	std::cout << "Modal analysis - Elimination method started" << std::endl;

	// Create a node ID map (to create a nodes as ordered and numbered from 0,1,2...n)
	int i = 0;
	for (auto& nd : model_nodes.nodeMap)
	{
		nodeid_map[nd.first] = i;
		i++;
	}

	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Node maping completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;








}
