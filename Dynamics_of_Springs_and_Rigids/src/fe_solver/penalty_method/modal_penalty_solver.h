#pragma once
#include <iostream>
#include <fstream>

// FE Objects
#include "../../geometry_store/fe_objects/nodes_list_store.h"
#include "../../geometry_store/fe_objects/elementline_list_store.h"
#include "../../geometry_store/fe_objects/nodeconstraint_list_store.h"
#include "../../geometry_store/fe_objects/nodepointmass_list_store.h"

// FE Results Modal Analysis
#include "../../geometry_store/analysis_result_objects/modal_nodes_list_store.h"
#include "../../geometry_store/analysis_result_objects/modal_elementline_list_store.h"

// Stop watch
#include "../../events_handler/Stopwatch_events.h"

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



class modal_penalty_solver
{
public:
	// Result store
	int number_of_modes = 0;
	std::unordered_map<int, int> nodeid_map; // Node ID map
	std::unordered_map<int, double> m_eigenvalues;
	std::unordered_map<int, std::vector<double>> m_eigenvectors;
	std::vector<std::string> mode_result_str;


	int numDOF = 0;

	// Matrix stored
	Eigen::VectorXd modalMass;
	Eigen::VectorXd modalStiff;
	//Eigen::VectorXi globalDOFMatrix;
	//Eigen::MatrixXd globalSupportInclinationMatrix;
	//Eigen::MatrixXd reduced_eigenvectors;
	//Eigen::MatrixXd global_eigenvectors;
	//Eigen::MatrixXd global_eigenvectors_transformed;

	modal_penalty_solver();
	~modal_penalty_solver();
	void clear_results();

	void modal_analysis_penaltymethod_start(const nodes_list_store& model_nodes,
		const elementline_list_store& model_lineelements,
		const nodeconstraint_list_store& node_constraints,
		const nodepointmass_list_store& node_ptmass,
		const std::unordered_map<int, material_data>& material_list,
		modal_nodes_list_store& modal_result_nodes,
		modal_elementline_list_store& modal_result_lineelements,
		bool& is_modal_analysis_complete);

private:
	const double m_pi = 3.14159265358979323846;
	bool print_matrix = true;
	Stopwatch_events stopwatch;
	std::stringstream stopwatch_elapsed_str;


	// Penalty stiffness
	double max_stiffness = 0.0;
	const double penalty_scale_factor = 1E+6;

	double zero_ptmass = 0.0; // pt mass for nodes with no pt mass assignment


	void get_global_stiffness_matrix(Eigen::MatrixXd& globalStiffnessMatrix,
		const elementline_list_store& model_lineelements,
		const nodeconstraint_list_store& node_constraints,
		const std::unordered_map<int, material_data>& material_list,
		std::ofstream& output_file);


	void get_element_stiffness_matrix(Eigen::MatrixXd& elementStiffnessMatrix,
		const elementline_store& ln,
		const nodeconstraint_list_store& node_constraints,
		const material_data& elementline_material,
		std::ofstream& output_file);


	void get_global_pointmass_matrix(Eigen::MatrixXd& globalPointMassMatrix,
		const nodes_list_store& model_nodes,
		const nodepointmass_list_store& node_ptmass,
		std::ofstream& output_file);


	void get_boundary_condition_penalty_matrix(Eigen::MatrixXd& globalPenalty_SPC_StiffnessMatrix,
		Eigen::MatrixXd& globalPenalty_MPC_StiffnessMatrix,
		int& constraint_count,
		const nodes_list_store& model_nodes,
		const elementline_list_store& model_lineelements,
		const nodeconstraint_list_store& node_constraints,
		const std::unordered_map<int, material_data>& material_list,
		std::ofstream& output_file);


	void get_invsqrt_PointMassMatrix(Eigen::MatrixXd& invsqrt_globalPointMassMatrix,
		const Eigen::MatrixXd& globalPointMassMatrix,
		std::ofstream& output_file);


	void sort_eigen_values_vectors(Eigen::VectorXd& eigenvalues,
		Eigen::MatrixXd& eigenvectors,
		const int& m_size);


	void normalize_eigen_vectors(Eigen::MatrixXd& eigenvectors,
		const int& m_size);


	void get_modal_participation_factor(Eigen::VectorXd& participation_factor,
		const Eigen::MatrixXd& globalPointMassMatrix,
		const Eigen::MatrixXd& global_eigenvectors,
		std::ofstream& output_file);


	void map_modal_analysis_results(const nodes_list_store& model_nodes,
		const elementline_list_store& model_lineelements,
		modal_nodes_list_store& modal_result_nodes,
		modal_elementline_list_store& modal_result_lineelements,
		std::ofstream& output_file);



	void get_modal_matrices(Eigen::VectorXd& modalMass,
		Eigen::VectorXd& modalStiff,
		const Eigen::MatrixXd& eigenvectors,
		const Eigen::MatrixXd& globalPointMassMatrix,
		const Eigen::MatrixXd& globalStiffnessMatrix,
		std::ofstream& output_file);






};




