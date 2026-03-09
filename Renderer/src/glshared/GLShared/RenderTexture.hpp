#pragma once

#include "glcommon.hpp"
#include "opencv2/opencv.hpp"

class RenderTexture {
private:
	GLuint TEX;
	bool allocated = false;
public:
	RenderTexture() {
		glGenTextures(1, &TEX);
		glBindTexture(GL_TEXTURE_2D, TEX);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	}
	void Set(const cv::Mat& image) {
		glBindTexture(GL_TEXTURE_2D, TEX);

		if (!allocated) {
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols, image.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, image.data);
			allocated = true;
		}
		else {
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image.cols, image.rows, GL_RGB, GL_UNSIGNED_BYTE, image.data);
		}

		glGenerateMipmap(GL_TEXTURE_2D);
	}
	void Bind(int slot = 0) const {
		glActiveTexture(GL_TEXTURE0 + slot);
		glBindTexture(GL_TEXTURE_2D, TEX);
	}
};