#include "modal_penalty_solver.h"

modal_penalty_solver::modal_penalty_solver()
{
	// Empty constructor
}


modal_penalty_solver::~modal_penalty_solver()
{
	// Empty destructor
}


void modal_penalty_solver::clear_results()
{

}


void modal_penalty_solver::modal_analysis_penaltymethod_start(const nodes_list_store& model_nodes, 
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






}
