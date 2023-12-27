#pragma once
#include "pulse_analysis_solver.h"

class forcedresp_analysis_solver
{
public:
	const double m_pi = 3.14159265358979323846;
	const double epsilon = 0.000001;
	bool print_matrix = false;

	// Analysis settings and results
	bool is_forcedresp_analysis_complete = false;


	forcedresp_analysis_solver();
	~forcedresp_analysis_solver();
	void clear_results();
	void forcedresp_analysis_start(const nodes_list_store& model_nodes,
		const elementline_list_store& model_lineelements,
		const nodeconstraint_list_store& node_constraints,
		const nodeload_list_store& node_loads,
		const nodepointmass_list_store& node_ptmass,
		const nodeinlcond_list_store& node_inlcond,
		const std::unordered_map<int, material_data>& material_list,
		const modal_analysis_solver& modal_solver,
		const std::vector<int> selected_nodes,
		const double start_frequency,
		const double end_frequency,
		const double frequency_interval,
		const double damping_ratio,
		const int mode_range_startid,
		const int mode_range_endid);


private:

};
