#include "modal_analysis_solver.h"

modal_analysis_solver::modal_analysis_solver()
{
	// Empty constructor
}

modal_analysis_solver::~modal_analysis_solver()
{
	// Empty destructor
}


void modal_analysis_solver::clear_results()
{
	// Clear the eigen values and eigen vectors
	number_of_modes = 0;
	nodeid_map.clear();
	eigen_values.clear();
	eigen_vectors.clear();
	eigen_vectors_reduced.clear();
	mode_result_str.clear();
	is_modal_analysis_complete = false;
}