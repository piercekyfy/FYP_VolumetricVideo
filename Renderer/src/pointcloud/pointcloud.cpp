#include <iostream>
#include <vector>

#include "glcommon.hpp"

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"


int main() {
    GLFWwindow* window = initSimpleResizableViewport(600, 600);

    if (window == nullptr) {
        glfwTerminate();
        return -1;
    }

    // Point-cloud Model-View-Projection Matrix

    // Arg1: FOV angle, Arg2: Width/Height, Arg3: zNear, Arg4: zFar
    glm::mat4 projection = glm::perspective(glm::radians(45.0f), 800.0f / 600.0f, 0.1f, 100.0f); 

    glm::mat4 view = glm::lookAt(glm::vec3(3.0f, 0.0f, 0.0f), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));//glm::mat4(1.0f);
    //// Move view back 3.0f
    //view = glm::translate(view, glm::vec3(0.0f, 0.0f, -3.0f));
    //// Rotate it
    //view = glm::rotate(view, glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 0.0f));

    glm::mat4 model = glm::mat4(1.0f);

    // Setup Shader

    std::string relPath = getPathWindows();
    Shader shader{ (relPath + "\\shaders\\pointcloud.vert").c_str(), (relPath + "\\shaders\\pointcloud.frag").c_str() };

    std::vector<glm::vec3> points{
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.5f},
        {0.0f, 0.0f, 1.0f},
        {0.0f, 0.0f, 1.5f},
        {0.0f, 0.0f, 2.0f},
        {0.0f, 0.0f, 2.5f},
        {0.0f, 0.0f, 3.0f},
    };


    GLuint VAO, VBO; 
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(glm::vec3), points.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);

    glPointSize(10.0f);

    glEnable(GL_DEPTH_TEST);

    while (!glfwWindowShouldClose(window)) {
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        float angle = (float)glfwGetTime();
        glm::vec3 camPos = glm::vec3(3.0f * cos(angle), 0.0f, 3.0f * sin(angle));
        view = glm::lookAt(camPos, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));

        shader.use();
        glm::mat4 mvp = projection * view * model;
        shader.setMatrix("mvp", GL_FALSE, glm::value_ptr(mvp));
        glBindVertexArray(VAO);
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(points.size()));

        glfwSwapBuffers(window);
        glfwPollEvents();
    };

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);

    glfwTerminate();

    return 0;
}