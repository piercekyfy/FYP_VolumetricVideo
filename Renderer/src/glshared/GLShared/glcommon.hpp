#pragma once
// glcommon.hpp : Collection of functions to simplify scripts

#include <string>
#include <iostream>
#include <windows.h>

// Include OpenGL dependencies
#include <glad/glad.h> 
#include <GLFW/glfw3.h>

// Include other glshared headers
#include "shader.hpp"

static void resize_cb(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

static GLFWwindow* initSimpleResizableViewport(int width, int height) 
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(width, height, "Window", nullptr, nullptr);
    if (window == nullptr)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return nullptr;
    }
    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;

        return nullptr;
    }

    glViewport(0, 0, width, height);

    glfwSetFramebufferSizeCallback(window, resize_cb);

    return window;
}

static std::string getPathWindows()
{
    char path[512];
    DWORD len = GetModuleFileNameA(nullptr, path, 512);
    if (len == 0) throw std::runtime_error{"Failed to find path."};
    std::string str = std::string(path, len);
    size_t cut = str.find_last_of('\\');
    if (cut == std::string::npos) throw std::runtime_error{"Module path resolved to directory."};
    str.erase(cut);

    return str;
}