#include "IndexBuffer.h"

IndexBuffer::IndexBuffer()
{
	// Empty constructor
}

IndexBuffer::~IndexBuffer()
{
	// Release the buffer ID
	glDeleteBuffers(1, &ib_id);
}

void IndexBuffer::createIndexBuffer(const unsigned int* indices, unsigned int count)
{
	if (ib_id != 0)
	{
		// Delete old buffer
		glDeleteBuffers(1, &ib_id);
		ib_id = 0;
		m_count = 0;
		m_capacity = 0;
	}
	
	m_count = count;
	m_capacity = count;

	// Generate a new buffer ID for the index buffer
	glGenBuffers(1, &ib_id);

	// Bind the buffer to the GL_ELEMENT_ARRAY_BUFFER target
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ib_id);

	// Copy the index data to the buffer
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, count * sizeof(unsigned int), indices, GL_DYNAMIC_DRAW);
}

void IndexBuffer::updateIndexBuffer(const unsigned int* indices, unsigned int count)
{
	if (ib_id == 0) 
	{
		createIndexBuffer(indices, count);
		return;
	}

    Bind();

    if (count == 0) {
        // Clear the buffer entirely
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
        m_count = 0;
        m_capacity = 0;
    }
    else if (count > m_capacity) {
        // Need larger buffer - reallocate
        m_capacity = count;
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
            count * sizeof(unsigned int),
            indices,
            GL_DYNAMIC_DRAW);
        m_count = count;
    }
    else {
        // Buffer has enough capacity, just update the data
        glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0,
            count * sizeof(unsigned int), indices);
        m_count = count;

        // If we're shrinking significantly, optionally clear the remainder
        if (count < m_capacity / 2) {
            // Shrink to save memory (optional)
            m_capacity = count;
            glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                count * sizeof(unsigned int),
                indices,
                GL_DYNAMIC_DRAW);
        }
    }

    UnBind();

}



void IndexBuffer::Bind() const
{
	// Bind the buffer to the GL_ELEMENT_ARRAY_BUFFER target
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ib_id);
}

void IndexBuffer::UnBind() const
{
	// Unbind the buffer from the GL_ELEMENT_ARRAY_BUFFER target
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}



void IndexBuffer::clear()
{
    if (ib_id != 0) {
        Bind();
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
        m_count = 0;
        m_capacity = 0;
        UnBind();
    }
}