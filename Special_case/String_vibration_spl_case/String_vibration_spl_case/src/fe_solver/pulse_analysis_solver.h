#pragma once
#include "modal_analysis_solver.h"
#include "../geometry_store/fe_objects/nodeload_list_store.h"
#include "../geometry_store/fe_objects/nodeinlcond_list_store.h"
#include "../geometry_store/result_objects/pulse_node_list_store.h"
#include "../geometry_store/result_objects/pulse_elementline_list_store.h"


struct pulse_load_data
{
	int load_id = 0;
	double load_start_time = 0.0;
	double load_end_time = 0.0;
	Eigen::VectorXd modal_LoadamplMatrix;
};


class pulse_analysis_solver
{
public:
	const double m_pi = 3.14159265358979323846;
	const double epsilon = 0.000001;
	bool print_matrix = false;
	int model_type = 0;
	int numDOF = 0;
	int reducedDOF = 0; // reduced dof is the dof after support condition is applied.

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
		const int selected_pulse_option,
		pulse_node_list_store& pulse_result_nodes,
		pulse_elementline_list_store& pulse_result_lineelements);

private:
	Stopwatch_events stopwatch;
	std::stringstream stopwatch_elapsed_str;

	// Pulse load data store
	std::vector<pulse_load_data> pulse_loads;

	Eigen::VectorXd eigen_values_vector; // omega n squared
	Eigen::MatrixXd eigen_vectors_matrix;
	Eigen::MatrixXd eigen_vectors_matrix_inverse;

	void create_initial_condition_matrices(Eigen::VectorXd& modal_reducedInitialDisplacementMatrix,
		Eigen::VectorXd& modal_reducedInitialVelocityMatrix,
		const nodes_list_store& model_nodes,
		const nodeinlcond_list_store& node_inldispl,
		const nodeinlcond_list_store& node_inlvelo,
		const Eigen::MatrixXd& eigen_vectors_matrix_inverse);

	void create_pulse_load_matrices(pulse_load_data& pulse_ld,
		const load_data& ld,
		const nodes_list_store& model_nodes,
		const Eigen::MatrixXd& eigen_vectors_matrix_inverse);

	double get_steady_state_initial_condition_soln(const double& time_t,
		const double& modal_mass,
		const double& modal_stiff,
		const double& modal_initial_displacement,
		const double& modal_initial_velocity);


	double get_steady_state_half_sine_pulse_soln(const double& time_t,
		const double& modal_mass,
		const double& modal_stiff,
		const double& modal_force_ampl,
		const double& modal_force_starttime,
		const double& modal_force_endtime);


	double get_steady_state_rectangular_pulse_soln(const double& time_t,
		const double& modal_mass,
		const double& modal_stiff,
		const double& modal_force_ampl,
		const double& modal_force_starttime,
		const double& modal_force_endtime);


	double get_steady_state_triangular_pulse_soln(const double& time_t,
		const double& modal_mass,
		const double& modal_stiff,
		const double& modal_force_ampl,
		const double& modal_force_starttime,
		const double& modal_force_endtime);


	double get_steady_state_stepforce_finiterise_soln(const double& time_t,
		const double& modal_mass,
		const double& modal_stiff,
		const double& modal_force_ampl,
		const double& modal_force_starttime,
		const double& modal_force_endtime);


	double get_total_harmonic_soln(const double& time_t,
		const double& modal_mass,
		const double& modal_stiff,
		const double& modal_force_ampl,
		const double& modal_force_starttime,
		const double& modal_force_endtime);


	void map_pulse_analysis_results(pulse_node_list_store& pulse_result_nodes,
		pulse_elementline_list_store& pulse_result_lineelements,
		const int& number_of_time_steps,
		const nodes_list_store& model_nodes,
		const elementline_list_store& model_lineelements,
		const std::unordered_map<int, pulse_node_result>& node_results);

};
