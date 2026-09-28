#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "part.h"

#include <cglm/cglm.h>   /* for inline */



void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

const char *vertexShaderSource = "#version 330 core\n"
    "layout (location = 0) in vec3 aPos;\n"
    "uniform mat4 transform;\n"
    "void main()\n"
    "{\n"
    "   gl_Position = transform*vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
    "}\0";
const char *fragmentShaderSource = "#version 330 core\n"
    "out vec4 FragColor;\n"
    "void main()\n"
    "{\n"
    "   FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);\n"
    "}\n\0";


int main(void) 
{
// glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Icosahedron in C", NULL, NULL);
    if (window == NULL)
    {
        printf("Failed to create GLFW window.\n");
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        printf("Failed to initialize GLAD.\n");
        return -1;
    }


    // build and compile our shader program
    // ------------------------------------
    // vertex shader
    unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);
    // check for shader compile errors
    int success;
    char infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        
        printf("ERROR::SHADER::VERTEX::COMPILATION_FAILED:\n %s\n",infoLog);
    }
    // fragment shader
    unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);
    // check for shader compile errors
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        printf("ERROR::SHADER::FRAGMENT::COMPILATION_FAILED:\n %s\n",infoLog);
    }
    // link shaders
    unsigned int shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    // check for linking errors
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        printf("ERROR::SHADER::PROGRAM::LINKING_FAILED:\n %s.\n",infoLog);
    }
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    // set up vertex data (and buffer(s)) and configure vertex attributes
    // ------------------------------------------------------------------
    float *nodes2coords = (float*)malloc(100*sizeof(float));
    FILE *my_file_nodes;
    my_file_nodes = fopen("../icosahedron/nodes2coords.txt", "r");
    unsigned int i = 0 ;
    
    while (!feof(my_file_nodes))
    { 
        fscanf(my_file_nodes, "  %f", &nodes2coords[i]);
        //printf("%f\n",nodes2coords[i] );
        i++;
    }
    unsigned int num_nodes = (i-1)/3;
    fclose(my_file_nodes);

    
    unsigned int *facets2nodes = (unsigned int*)malloc(100*sizeof(unsigned int));
    FILE *my_file_facets;
    my_file_facets = fopen("../icosahedron/facets2nodes.txt", "r");
    i = 0 ;
    
    while (!feof(my_file_facets))
    { 
        fscanf(my_file_facets, "  %u", &facets2nodes[i]);
        //printf("%u\n",facets2nodes[i] );
        i++;
    }

    unsigned int num_facets = (i-1)/3;
    fclose(my_file_facets);

    part icosahedron;

    part_initialise(&icosahedron, num_facets, num_nodes,facets2nodes, nodes2coords );

    free(nodes2coords);
    free(facets2nodes);
   
    while (!glfwWindowShouldClose(window))
    {

        processInput(window);


        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);




        float time = glfwGetTime();
        mat4 transform_matrix = {cos(time), 0, -sin(time), 0,
                                0, 1, 0 , 0,
                                 sin(time),0,  cos(time),0, 
                                 0, 0, 0, 1};



        // draw our first triangle
        glUseProgram(shaderProgram);
        glBindVertexArray(icosahedron.vertex_array_object); // seeing as we only have a single VAO there's no need to bind it every time, but we'll do so to keep things a bit more organized
        //glDrawArrays(GL_TRIANGLES, 0, 6);

        unsigned int transformLoc = glGetUniformLocation(shaderProgram, "transform");
        glUniformMatrix4fv(transformLoc, 1, GL_FALSE, (float *)transform_matrix);
        
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDrawElements(GL_TRIANGLES, 3*icosahedron.num_facets, GL_UNSIGNED_INT, 0);
        // glBindVertexArray(0); // no need to unbind it every time 
 
        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // optional: de-allocate all resources once they've outlived their purpose:
    // ------------------------------------------------------------------------
    glDeleteVertexArrays(1, &icosahedron.vertex_array_object);
    glDeleteBuffers(1, &icosahedron.vertex_buffer_object);
    glDeleteBuffers(1, &icosahedron.element_buffer_object);
    glDeleteProgram(shaderProgram);

    part_destroy(&icosahedron);

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();


    
    return 0;
}


// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}


