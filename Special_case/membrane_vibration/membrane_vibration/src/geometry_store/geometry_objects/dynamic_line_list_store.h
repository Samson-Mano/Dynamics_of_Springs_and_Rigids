#pragma once
#include "../geometry_buffers/gBuffers.h"
#include "../geom_parameters.h"

struct dynamic_line_store
{
	// store the individual point
	int line_id = 0;
	glm::vec3 line_startpt_loc = glm::vec3(0);
	glm::vec3 line_endpt_loc = glm::vec3(0);

	std::vector<glm::vec3> line_startpt_offset; // list of start points offset
	std::vector<glm::vec3> line_endpt_offset; // list of end points offset

	std::vector<double> line_startpt_offset_val;
	std::vector<double> line_endpt_offset_val;

	int offset_pt_count = 0;
};

class dynamic_line_list_store
{
public:
	geom_parameters* geom_param_ptr = nullptr;
	unsigned int dyn_line_count = 0;
	std::vector<dynamic_line_store> dyn_lineMap;

	dynamic_line_list_store();
	~dynamic_line_list_store();
	void init(geom_parameters* geom_param_ptr);
	void add_line(int& line_id, const glm::vec3& line_startpt_loc,const glm::vec3& line_endpt_loc,
		const std::vector<glm::vec3>& line_startpt_offset,const std::vector<glm::vec3>& line_endpt_offset,
		const std::vector<double>& line_startpt_offset_mag,const std::vector<double>& line_endpt_offset_mag);
	void set_buffer();
	void paint_lines();
	void paint_lines(const int& dyn_index);
	void update_buffer(const int& dyn_index);
	void clear_lines();
	void update_opengl_uniforms(bool set_modelmatrix, bool set_pantranslation, bool set_rotatetranslation, 
		bool set_zoomtranslation, bool set_transparency, bool set_deflscale);
private:
	gBuffers dyn_line_buffer;
	Shader dyn_line_shader;

	void get_line_vertex_buffer(dynamic_line_store& ln, const int& dyn_index, 
		float* dyn_line_vertices, unsigned int& dyn_line_v_index);

	void get_line_index_buffer(unsigned int* dyn_line_vertex_indices, unsigned int& dyn_line_i_index);
};

