#pragma once
#include "pulse_analysis_solver.h"


struct forceresp_load_data
{
	int load_id = 0;
	double load_phase = 0.0;
	Eigen::VectorXd modal_reducedLoadamplMatrix;
	Eigen::VectorXd reducedLoadamplMatrix;
	Eigen::VectorXd globalLoadamplMatrix;
};


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
	void forcedresp_analysis_start(std::vector<frequency_reponse_data>& frf_data,
		chart_setting_data& frf_chart_setting,
		const nodes_list_store& model_nodes,
		const elementline_list_store& model_lineelements,
		const nodeconstraint_list_store& node_constraints,
		const nodeload_list_store& node_loads,
		const nodepointmass_list_store& node_ptmass,
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
	std::unordered_map<int, int> nodeid_map;

	void create_forcedresp_load_matrices(forceresp_load_data& forcedresp_loads,
		const load_data& ld,
		const nodes_list_store& model_nodes,
		const Eigen::VectorXi& globalDOFMatrix,
		const Eigen::MatrixXd& globalSupportInclinationMatrix,
		const Eigen::MatrixXd& reduced_eigenVectorsMatrix_transpose,
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


	void get_steady_state_harmonic_periodic_soln(double& steady_state_displ_resp,
		double& phase_resp,
		const double& modal_mass,
		const double& modal_stiff,
		const double& damping_ratio,
		const double& modal_force_ampl,
		const double& forcing_freq);

};
