#pragma once

#include <string>
#include "glad/glad.h"

class Shader
{
public:
	unsigned int id;

	Shader(const char* vertexPath, const char* fragmentPath);
	~Shader();
	void use();
	void setBool(const std::string& name, bool value) const;
	void setInt(const std::string& name, int value) const;
	void setFloat(const std::string& name, float value) const;
	void setMatrix(const std::string& name, GLboolean transpose, const GLfloat* value) const;
private:
	static std::string getSource(const char* path);
	static void compileShader(unsigned int& shader, const char* source);
	static bool verifyShader(unsigned int& shader, bool throwOnFailure);
	static bool verifyProgram(unsigned int& program, bool throwOnFailure);
};