#pragma once
#include "modal_analysis_solver.h"
#include "../geometry_store/fe_objects/nodeload_list_store.h"
#include "../geometry_store/fe_objects/nodeinlcond_list_store.h"

struct pulse_load_data
{
	int load_id = 0;
	double load_start_time = 0.0;
	double load_end_time = 0.0;
	Eigen::VectorXd modal_reducedLoadamplMatrix;
	Eigen::VectorXd reducedLoadamplMatrix;
	Eigen::VectorXd globalLoadamplMatrix;
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
		const nodeinlcond_list_store& node_inldispl,
		const nodeinlcond_list_store& node_inlvelo,
		const material_data& mat_data,
		const modal_analysis_solver& modal_solver,
		const double total_simulation_time,
		const double time_interval,
		const double damping_ratio,
		const int mode_range_startid,
		const int mode_range_endid,
		const int selected_pulse_option);

private:
	Stopwatch_events stopwatch;
	std::stringstream stopwatch_elapsed_str;

	std::unordered_map<int, int> nodeid_map;



};
