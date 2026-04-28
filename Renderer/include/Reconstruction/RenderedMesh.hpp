#pragma once

#include "GLShared/Point.hpp"
#include "GLShared/glcommon.hpp"

class RenderedMesh {
	GLuint vao, vbo, ebo;
	int indexCount = 0;
public:
	RenderedMesh() {
		glGenVertexArrays(1, &vao);
		glGenBuffers(1, &vbo);
		glGenBuffers(1, &ebo);

		glBindVertexArray(vao);

		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Point), (void*)offsetof(Point, position));
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Point), (void*)offsetof(Point, color));

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);

		glBindVertexArray(0);
	}
	~RenderedMesh() {
		if (vao) glDeleteVertexArrays(1, &vao);
		if (vbo) glDeleteBuffers(1, &vbo);
		if (ebo) glDeleteBuffers(1, &ebo);
	}
	void Update(const std::vector<Point>& points, const std::vector<int>& tris) {
		glBindVertexArray(vao);

		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(Point), points.data(), GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, tris.size() * sizeof(int), tris.data(), GL_DYNAMIC_DRAW);

		indexCount = tris.size();
	}
	void Draw() {
		if (indexCount == 0) return;
		glBindVertexArray(vao);
		glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
	}
};