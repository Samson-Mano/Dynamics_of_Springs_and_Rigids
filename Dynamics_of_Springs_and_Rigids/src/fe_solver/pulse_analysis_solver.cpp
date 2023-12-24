#include "pulse_analysis_solver.h"

pulse_analysis_solver::pulse_analysis_solver()
{
	// Empty constructor
}

pulse_analysis_solver::~pulse_analysis_solver()
{
	// Empty destructor
}

void pulse_analysis_solver::clear_results()
{
	// Clear the analysis results
	is_pulse_analysis_complete = false;
	time_step_count = 0;
	time_interval = 0.0;
	total_simulation_time = 0.0;

}

void pulse_analysis_solver::pulse_analysis_start(const nodes_list_store& model_nodes, 
	const elementline_list_store& model_lineelements, 
	const nodeconstraint_list_store& node_constraints, 
	const nodeload_list_store& node_loads, 
	const nodepointmass_list_store& node_ptmass, 
	const nodeinlcond_list_store& node_inlcond, 
	const std::unordered_map<int, material_data>& material_list, 
	const modal_analysis_solver& modal_solver, 
	const double total_simulation_time, 
	const double time_interval, 
	const double damping_ratio, 
	const int mode_range_startid,
	const int mode_range_endid,
	pulse_node_list_store& pulse_result_nodes, 
	pulse_elementline_list_store& pulse_result_lineelements)
{
	// Main solver call
	this->is_pulse_analysis_complete = false;




}
