#pragma once
#include "modal_penalty_solver.h"
#include "../../geometry_store/fe_objects/nodeload_list_store.h"
#include "../../geometry_store/fe_objects/nodeinlcond_list_store.h"
#include "../../geometry_store/analysis_result_objects/pulse_node_list_store.h"
#include "../../geometry_store/analysis_result_objects/pulse_elementline_list_store.h"

struct pulse_load_data
{
	int load_id = 0;
	double load_start_time = 0.0;
	double load_end_time = 0.0;
	Eigen::VectorXd modal_reducedLoadamplMatrix;
	Eigen::VectorXd reducedLoadamplMatrix;
	Eigen::VectorXd globalLoadamplMatrix;
};


class pulse_penalty_solver
{
public:
	const double m_pi = 3.14159265358979323846;
	const double epsilon = 0.000001;
	bool print_matrix = false;

	// Analysis settings and results
	int time_step_count = 0;
	double time_interval = 0.0;
	double total_simulation_time = 0.0;

	pulse_penalty_solver();
	~pulse_penalty_solver();
	void clear_results();

	void pulse_analysis_penaltymethod_start(const nodes_list_store& model_nodes,
		const elementline_list_store& model_lineelements,
		const nodeconstraint_list_store& node_constraints,
		const nodeload_list_store& node_loads,
		const nodepointmass_list_store& node_ptmass,
		const nodeinlcond_list_store& node_inlcond,
		const std::unordered_map<int, material_data>& material_list,
		const modal_penalty_solver& modal_pulse_s,
		const double total_simulation_time,
		const double time_interval,
		const double damping_ratio,
		const int mode_range_startid,
		const int mode_range_endid,
		const int selected_pulse_option,
		pulse_node_list_store& pulse_result_nodes,
		pulse_elementline_list_store& pulse_result_lineelements,
		bool& is_pulse_analysis_complete);

private:
	Stopwatch_events stopwatch;
	std::stringstream stopwatch_elapsed_str;

	std::unordered_map<int, int> nodeid_map;


	void get_reduced_global_matrix(Eigen::MatrixXd& reducedglobalMatrix,
		const Eigen::MatrixXd& globalMatrix,
		const Eigen::VectorXi& globalDOFMatrix,
		const int& numDOF,
		const int& reducedDOF);


	void create_initial_condition_matrices(Eigen::VectorXd& modal_reducedInitialDisplacementMatrix,
		Eigen::VectorXd& modal_reducedInitialVelocityMatrix,
		const nodeinlcond_list_store& node_inlcond,
		const Eigen::VectorXi& globalDOFMatrix,
		const Eigen::MatrixXd& globalSupportInclinationMatrix,
		const Eigen::MatrixXd& reduced_eigenVectorsMatrix,
		const int& numDOF,
		const int& reducedDOF,
		std::ofstream& output_file);


	void get_reduced_global_vector(Eigen::VectorXd& reducedglobalVector,
		const Eigen::VectorXd& globalVector,
		const Eigen::VectorXi& globalDOFMatrix,
		const int& numDOF,
		const int& reducedDOF);


	void get_global_resp_vector(Eigen::VectorXd& globalVector,
		const Eigen::VectorXd& reducedglobalVector,
		const Eigen::VectorXi& globalDOFMatrix,
		const int& numDOF,
		const int& reducedDOF);


	void create_pulse_load_matrices(pulse_load_data& pulse_loads,
		const load_data& ld,
		const nodes_list_store& model_nodes,
		const Eigen::VectorXi& globalDOFMatrix,
		const Eigen::MatrixXd& globalSupportInclinationMatrix,
		const Eigen::MatrixXd& reduced_eigenVectorsMatrix_transpose,
		const int& numDOF,
		const int& reducedDOF,
		std::ofstream& output_file);


	void get_steady_state_initial_condition_soln(double& steady_state_displ_resp,
		const double& time_t,
		const double& modal_mass,
		const double& modal_stiff,
		const double& modal_initial_displacement,
		const double& modal_initial_velocity);


	void get_steady_state_half_sine_pulse_soln(double& steady_state_displ_resp,
		const double& time_t,
		const double& modal_mass,
		const double& modal_stiff,
		const double& modal_force_ampl,
		const double& modal_force_starttime,
		const double& modal_force_endtime);


	void get_steady_state_rectangular_pulse_soln(double& steady_state_displ_resp,
		const double& time_t,
		const double& modal_mass,
		const double& modal_stiff,
		const double& modal_force_ampl,
		const double& modal_force_starttime,
		const double& modal_force_endtime);


	void get_steady_state_triangular_pulse_soln(double& steady_state_displ_resp,
		const double& time_t,
		const double& modal_mass,
		const double& modal_stiff,
		const double& modal_force_ampl,
		const double& modal_force_starttime,
		const double& modal_force_endtime);


	void get_steady_state_stepforce_finiterise_soln(double& steady_state_displ_resp,
		const double& time_t,
		const double& modal_mass,
		const double& modal_stiff,
		const double& modal_force_ampl,
		const double& modal_force_starttime,
		const double& modal_force_endtime);


	void get_total_harmonic_soln(double& steady_state_displ_resp,
		const double& time_t,
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
