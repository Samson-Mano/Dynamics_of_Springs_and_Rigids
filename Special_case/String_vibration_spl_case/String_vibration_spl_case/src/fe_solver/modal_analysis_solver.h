#pragma once
#include <iostream>
#include <fstream>

// FE Objects
#include "../geometry_store/fe_objects/nodes_list_store.h"
#include "../geometry_store/fe_objects/elementline_list_store.h"
#include "../geometry_store/fe_objects/nodeconstraint_list_store.h"

// FE Results Modal Analysis


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
	std::unordered_map<int, int> nodeid_map; // Node ID map
	std::unordered_map<int, double> m_eigenvalues;
	std::unordered_map<int, std::vector<double>> m_eigenvectors;
	std::vector<std::string> mode_result_str;
	bool is_modal_analysis_complete = false;
	int numDOF = 0;
	int reducedDOF = 0;

	// Matrix stored
	Eigen::VectorXd reduced_modalMass;
	Eigen::VectorXd reduced_modalStiff;
	Eigen::VectorXi globalDOFMatrix;
	Eigen::MatrixXd globalSupportInclinationMatrix;
	Eigen::MatrixXd reduced_eigenvectors_transformed;
	Eigen::MatrixXd global_eigenvectors_transformed;

	modal_analysis_solver();
	~modal_analysis_solver();
	void clear_results();

	void modal_analysis_start(const nodes_list_store& model_nodes,
		const elementline_list_store& model_lineelements,
		const nodeconstraint_list_store& node_constraints,
		const material_data& mat_data);
private:
	const double m_pi = 3.14159265358979323846;
	bool print_matrix = false;
	Stopwatch_events stopwatch;
	std::stringstream stopwatch_elapsed_str;


};
