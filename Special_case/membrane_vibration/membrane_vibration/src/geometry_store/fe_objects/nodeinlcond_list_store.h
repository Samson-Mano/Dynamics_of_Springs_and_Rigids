#pragma once
#include "nodes_list_store.h"

struct nodeinl_cond
{
	int node_id = 0;
	double y_val = 0.0;
};

class nodeinlcond_list_store
{
public:
	const double epsilon = 0.000001;
	std::unordered_map<int, nodeinl_cond> inlcondMap;
	double model_total_length = 0.0;
	int number_of_nodes = 0;
	int inl_cond_type = 0; //0 - Displacement, 1 - Velocity
	int model_type = 0; // 0,1 - Line, 2,3 - Circle

	nodeinlcond_list_store();
	~nodeinlcond_list_store();
	void init(geom_parameters* geom_param_ptr);
	void set_zero_condition(nodes_list_store& model_nodes, double& model_total_length, int inl_cond_type, const int& model_type);
	void add_inlcondition(double& inl_cond_val,int& node_start,  int& node_end, int& interpolation_type, nodes_list_store& model_nodes);
	void delete_all_inlcondition(nodes_list_store& model_nodes);
	void set_buffer();
	void paint_inlcond();
	void paint_inlcond_label();
	void update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, bool set_zoomtranslation, bool set_transparency, bool set_deflscale);

private:
	geom_parameters* geom_param_ptr = nullptr;
	point_list_store inlcond_points;
	label_list_store inl_condition_labels;

	glm::vec2 linear_interpolation(glm::vec2 pt1, glm::vec2 pt2, double t_val);
	glm::vec2 cubic_bezier_interpolation(glm::vec2 pt1, glm::vec2 pt2, glm::vec2 pt3, glm::vec2 pt4, double t_val);
	glm::vec2 half_sine_interpolation(glm::vec2 pt1, glm::vec2 pt2, glm::vec2 pt3, double t_val);
	void create_inlcondition_pts(nodes_list_store& model_nodes);
	glm::vec2 get_inlcondition_offset(glm::vec2& node_pt, double y_val);

};

