#pragma once
#include <iostream>
#include <fstream>

// FE Objects
#include "../geometry_store/fe_objects/nodes_list_store.h"
#include "../geometry_store/fe_objects/elementline_list_store.h"
#include "../geometry_store/fe_objects/nodeconstraint_list_store.h"
#include "../geometry_store/fe_objects/nodepointmass_list_store.h"

// FE Results Modal Analysis
#include "../geometry_store/analysis_result_objects/result_node_list_store.h"
#include "../geometry_store/analysis_result_objects/result_elementline_list_store.h"

// Stop watch
#include "../events_handler/Stopwatch_events.h"

#pragma warning(push)
#pragma warning (disable : 26451)
#pragma warning (disable : 26495)
#pragma warning (disable : 6255)
#pragma warning (disable : 6294)
#pragma warning (disable : 26813)
#pragma warning (disable : 26454)

// Optimization for Eigen Library
// 1) OpenMP (Yes (/openmp)
//	 Solution Explorer->Configuration Properties -> C/C++ -> Language -> Open MP Support
// 2) For -march=native, choose "AVX2" or the latest supported instruction set.
//   Solution Explorer->Configuration Properties -> C/C++ -> Code Generation -> Enable Enhanced Instruction Set 

#include <Eigen/Dense>
#include <Eigen/Sparse>
// Define the sparse matrix type for the reduced global stiffness matrix
typedef Eigen::SparseMatrix<double> SparseMatrix;
#pragma warning(pop)


class modal_analysis_solver
{
public:
	// Result store
	int number_of_modes = 0;
	std::unordered_map<int, int> nodeid_map;
	std::unordered_map<int, double> eigen_values;
	std::unordered_map<int, std::vector<double>> eigen_vectors;
	std::unordered_map<int, std::vector<double>> eigen_vectors_reduced;
	std::vector<std::string> mode_result_str;
	bool is_modal_analysis_complete = false;

	modal_analysis_solver();
	~modal_analysis_solver();
	void clear_results();
	void modal_analysis_start(const nodes_list_store& model_nodes,
		const elementline_list_store& model_lineelements,
		const nodeconstraint_list_store& model_constarints,
		const nodepointmass_list_store& model_ptmass,
		const std::unordered_map<int, material_data>& material_list,
		result_node_list_store& modal_result_nodes,
		result_elementline_list_store& modal_result_lineelements,
		bool& is_modal_analysis_complete);
private:
	const double m_pi = 3.14159265358979323846;
	bool print_matrix = false;
	Stopwatch_events stopwatch;

	int numDOF = 0;
	int reducedDOF = 0;
	double w_penalty = 0.0; // penalty stiffness
	std::unordered_map<int, int> nodeid_map;
	Eigen::MatrixXd globalStiffnessMatrix; // global stiffness matrix
	Eigen::MatrixXd globalMassMatrix; // global mass matrix
	Eigen::MatrixXd globalDOFMatrix; // global DOF matrix

	void get_global_stiffness_matrix(Eigen::MatrixXd& globalStiffnessMatrix,
		const elementline_list_store& model_lineelements,
		const std::unordered_map<int, material_data>& material_list,
		std::ofstream& output_file);


	void get_element_stiffness_matrix(Eigen::MatrixXd& elementStiffnessMatrix,
		const elementline_store& ln,
		const material_data& elementline_material,
		std::ofstream& output_file);

	void get_global_pointmass_matrix(Eigen::MatrixXd& globalPointMassMatrix,
		const nodes_list_store& model_nodes,
		const nodepointmass_list_store& model_ptmass,
		std::ofstream& output_file);

	void get_global_dof_matrix(Eigen::MatrixXd& globalDOFMatrix,
		const nodes_list_store& model_nodes,
		const nodeconstraint_list_store& model_constarints,
		int& reducedDOF,
		std::ofstream& output_file);



};
