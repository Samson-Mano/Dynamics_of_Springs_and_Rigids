#pragma once
#include "modal_analysis_solver.h"
#include "../geometry_store/fe_objects/nodeload_list_store.h"
#include "../geometry_store/fe_objects/nodeinlcond_list_store.h"
#include "../geometry_store/analysis_result_objects/pulse_node_list_store.h"
#include "../geometry_store/analysis_result_objects/pulse_elementline_list_store.h"

struct pulse_load_data
{
	int load_id = 0;
	double load_start_time = 0.0;
	double load_end_time = 0.0;
	Eigen::MatrixXd modal_reducedLoadamplMatrix;
	Eigen::MatrixXd reducedLoadamplMatrix;
	Eigen::MatrixXd globalLoadamplMatrix;
};


class pulse_analysis_solver
{
public:
	const double m_pi = 3.14159265358979323846;
	const double epsilon = 0.000001;
	bool print_matrix = false;

	// Analysis settings and results
	bool is_pulse_analysis_complete = false;
	int time_step_count = 0;
	double time_interval = 0.0;
	double total_simulation_time = 0.0;

	pulse_analysis_solver();
	~pulse_analysis_solver();
	void clear_results();
	void pulse_analysis_start(const nodes_list_store& model_nodes,
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
		pulse_elementline_list_store& pulse_result_lineelements);

private:

};
