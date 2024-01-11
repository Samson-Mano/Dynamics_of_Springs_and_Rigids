#pragma once
#include <iostream>
#include <fstream>

// FE Objects
#include "../geometry_store/fe_objects/nodes_list_store.h"
#include "../geometry_store/fe_objects/elementline_list_store.h"
#include "../geometry_store/fe_objects/nodeconstraint_list_store.h"

// FE Results Modal Analysis
#include "../geometry_store/result_objects/modal_nodes_list_store.h"
#include "../geometry_store/result_objects/modal_elementline_list_store.h"

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
#include <Eigen/SparseLU>
#include <Eigen/Eigenvalues>
// Define the sparse matrix type for the reduced global stiffness matrix
typedef Eigen::SparseMatrix<double> SparseMatrix;
#pragma warning(pop)


class modal_analysis_solver
{
public:
	// Result store
	int number_of_modes = 0;
	int node_count = 0;
	int matrix_size = 0;
	int model_type = 0;

	// Eigen values matrices
	Eigen::VectorXd angular_freq_vector;
	Eigen::VectorXd eigen_values_vector;

	// Eigen vector matrices
	Eigen::MatrixXd displ_vectors_matrix;
	Eigen::MatrixXd eigen_vectors_matrix;
	Eigen::MatrixXd eigen_vectors_matrix_inverse;

	std::vector<std::string> mode_result_str;
	bool is_modal_analysis_complete = false;

	modal_analysis_solver();
	~modal_analysis_solver();
	void clear_results();

	void modal_analysis_start(const nodes_list_store& model_nodes,
		const elementline_list_store& model_lineelements,
		const nodeconstraint_list_store& node_constraints,
		const material_data& mat_data,
		modal_nodes_list_store& modal_result_nodes,
		modal_elementline_list_store& modal_result_lineelements);
private:
	const double m_pi = 3.14159265358979323846;
	bool print_matrix = false;
	Stopwatch_events stopwatch;
	std::stringstream stopwatch_elapsed_str;

	void modal_analysis_model_linear1(const nodes_list_store& model_nodes,
		const elementline_list_store& model_lineelements, 
		const material_data& mat_data);


	void modal_analysis_model_linear2(const nodes_list_store& model_nodes,
		const elementline_list_store& model_lineelements,
		const material_data& mat_data);


	void modal_analysis_model_linear3(const nodes_list_store& model_nodes,
		const elementline_list_store& model_lineelements,
		const material_data& mat_data);


	void modal_analysis_model_circular1(const nodes_list_store& model_nodes,
		const elementline_list_store& model_lineelements,
		const material_data& mat_data);


	void map_modal_analysis_linear_results(const nodes_list_store& model_nodes,
		const elementline_list_store& model_lineelements,
		modal_nodes_list_store& modal_result_nodes,
		modal_elementline_list_store& modal_result_lineelements);


	void map_modal_analysis_circular_results(const nodes_list_store& model_nodes,
		const elementline_list_store& model_lineelements,
		modal_nodes_list_store& modal_result_nodes,
		modal_elementline_list_store& modal_result_lineelements);
};
