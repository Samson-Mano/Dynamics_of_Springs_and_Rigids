#include "VertexBuffer.h"

VertexBuffer::VertexBuffer()
{
	// Empty constructor
}

VertexBuffer::~VertexBuffer()
{
	// Destructor: deletes the vertex buffer object identified by the ID.
	glDeleteBuffers(1, &vb_id);
	vb_id = 0;
	buffer_size = 0;
}

void VertexBuffer::createVertexBuffer(const float* vertex_data, unsigned int vertex_size)
{
	// Main Constructor
	glGenBuffers(1, &vb_id);
	glBindBuffer(GL_ARRAY_BUFFER, vb_id);
	glBufferData(GL_ARRAY_BUFFER, vertex_size, vertex_data, GL_STATIC_DRAW);
}

void VertexBuffer::createDynamicVertexBuffer(unsigned int vertex_size)
{
	// Main constructor to create Dynamic vertex buffer
	buffer_size = vertex_size;
	glGenBuffers(1, &vb_id);
	glBindBuffer(GL_ARRAY_BUFFER, vb_id);
	glBufferData(GL_ARRAY_BUFFER, vertex_size, nullptr, GL_DYNAMIC_DRAW);
}

float* VertexBuffer::mapBuffer()
{
	glBindBuffer(GL_ARRAY_BUFFER, vb_id);

	// Use glMapBufferRange for better control (OpenGL 3.0+)
	float* ptr = (float*)glMapBufferRange(GL_ARRAY_BUFFER, 0, buffer_size,
		GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);

	if (ptr) 
	{
		is_mapped = true;
	}
	return ptr;
}

void VertexBuffer::unmapBuffer()
{
	if (is_mapped) 
	{
		glUnmapBuffer(GL_ARRAY_BUFFER);
		is_mapped = false;
	}
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}


void VertexBuffer::updateVertexBuffer(const float* vertex_data, unsigned int vertex_size)
{
	// Important!! Call only in Dynamic Buffer case
	// Update the vertex data
	glBindBuffer(GL_ARRAY_BUFFER, vb_id);
	glBufferSubData(GL_ARRAY_BUFFER, 0, vertex_size, vertex_data);
}


void VertexBuffer::Bind() const
{
	// Bind this vertex buffer with its id
	glBindBuffer(GL_ARRAY_BUFFER, vb_id);
}

void VertexBuffer::UnBind() const
{
	// Unbinds the currently bound vertex buffer object.
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}
