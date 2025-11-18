#include "shader.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

#include "glad/glad.h"

Shader::Shader(const char* vertexPath, const char* fragmentPath) {
	// Retrieve shader source code from path
	std::string vertSource;
	std::string fragSource;
	const char* csVertSource;
	const char* csFragSource;

	try {
		vertSource = getSource(vertexPath);
		csVertSource = vertSource.c_str();
	}
	catch (const std::ifstream::failure& ex) {
		throw std::runtime_error("ERROR:SHADER::VERTEX::FAILED_READ");
	}

	try {
		fragSource = getSource(fragmentPath);
		csFragSource = fragSource.c_str();
	}
	catch (const std::ifstream::failure& ex) {
		throw std::runtime_error("ERROR:SHADER::FRAGMENT::FAILED_READ");
	}

	unsigned int vert = glCreateShader(GL_VERTEX_SHADER), frag = glCreateShader(GL_FRAGMENT_SHADER);
	compileShader(vert, csVertSource);
	compileShader(frag, csFragSource);

	verifyShader(vert, true);
	verifyShader(frag, true);

	id = glCreateProgram();
	glAttachShader(id, vert);
	glAttachShader(id, frag);
	glLinkProgram(id);

	verifyProgram(id, true);

	glDeleteShader(vert);
	glDeleteShader(frag);
}

Shader::~Shader() {
	glDeleteProgram(id);
}

void Shader::use() {
	glUseProgram(id);
}

void Shader::setBool(const std::string& name, bool value) const
{
	glUniform1i(glGetUniformLocation(id, name.c_str()), (int)value);
}
void Shader::setInt(const std::string& name, int value) const
{
	glUniform1i(glGetUniformLocation(id, name.c_str()), value);
}
void Shader::setFloat(const std::string& name, float value) const
{
	glUniform1f(glGetUniformLocation(id, name.c_str()), value);
}

void Shader::setMatrix(const std::string& name, GLboolean transpose, const GLfloat* value) const {
	glUniformMatrix4fv(glGetUniformLocation(id, name.c_str()), 1, transpose, value);
}

std::string Shader::getSource(const char* path) {
	std::ifstream file;
	file.exceptions(std::ifstream::failbit | std::ifstream::badbit); // Throw exception if these IO errors occur.
	file.open(path);
	std::stringstream stream;
	stream << file.rdbuf(); // Read buffer into stream.
	file.close();
	return stream.str();
}

void Shader::compileShader(unsigned int& shader, const char* source) {
	glShaderSource(shader, 1, &source, 0);
	glCompileShader(shader);
}

bool Shader::verifyShader(unsigned int& shader, bool throwOnFailure) {
	int success;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
	if (!success && throwOnFailure) {
		char info[512];
		glGetShaderInfoLog(shader, 512, 0, info);
		throw std::runtime_error(std::string("ERROR::SHADER::COMPILATION_FAILED\n") + info);
	}
	return success;
}

bool Shader::verifyProgram(unsigned int& program, bool throwOnFailure) {
	int success;
	glGetProgramiv(program, GL_LINK_STATUS, &success);
	if (!success && throwOnFailure) {
		char info[512];
		glGetProgramInfoLog(program, 512, 0, info);
		throw std::runtime_error(std::string("ERROR::PROGRAM::LINKING_FAILED\n") + info);
	}
	return success;
}