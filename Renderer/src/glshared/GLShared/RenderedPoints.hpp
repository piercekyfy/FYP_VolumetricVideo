#pragma once

#include "glcommon.hpp"
#include "glshared/Point.hpp"
#include "opencv2/core.hpp"

class RenderedPoints {
private:
	GLuint VAO;
	GLuint VBO;
	int size{};
public:
	RenderedPoints() {
		glGenVertexArrays(1, &VAO);
		glGenBuffers(1, &VBO);

		glBindVertexArray(VAO);
		glBindBuffer(GL_ARRAY_BUFFER, VBO);

		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Point), (void*)0);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Point), (void*)offsetof(Point, texcoord));
		glEnableVertexAttribArray(1);

		glBindVertexArray(0);
	}
	~RenderedPoints() {
		if (VBO) glDeleteBuffers(1, &VBO);
		if (VAO) glDeleteVertexArrays(1, &VAO);
	}
	void Update(const std::vector<Point>& buffer) {
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, buffer.size() * sizeof(Point), buffer.data(), GL_DYNAMIC_DRAW);
		size = buffer.size();
	}
	void Draw() const {
		if (size == 0) return;

		glBindVertexArray(VAO);
		glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(size));
		glBindVertexArray(0);
	}
};