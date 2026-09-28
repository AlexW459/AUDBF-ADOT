//Name:        ADOT team.
//Date:        09/08/2024.
//Description: A struct typedef and related 
//             functions for the part type.

#include <stdlib.h> 
#include <stdio.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#ifndef PART_H
#define PART_H



struct part_tag; //prototyping the struct.

typedef struct part_tag {
    unsigned int num_facets; //unsigned int 0 ,1, 2, ...
    unsigned int num_nodes;      
    unsigned int *facets2nodes; // Size of this vector will be 3*num_facets.
    float *nodes2coords;       // Size of this vector will be 3*num_nodes.
    //Define variables related to openGL:
    unsigned int vertex_buffer_object;
    unsigned int vertex_array_object;
    unsigned int element_buffer_object;
    
} part;

void part_initialise(part *my_part, unsigned int input_num_facets, unsigned int input_num_nodes,
                     unsigned int *input_facets2nodes, float *input_nodes2coords ){

    my_part->num_facets = input_num_facets; //The -> is used to extract a member of
                                            //an pointer to an object proper.
    my_part->num_nodes = input_num_nodes;

    unsigned int facets_vector_size = 3*input_num_facets;
    unsigned int nodes_vector_size  = 3*input_num_nodes;

    my_part->facets2nodes = (unsigned int*)malloc(facets_vector_size*sizeof(unsigned int)); //Allocating the memory required
    my_part->nodes2coords = (float*)malloc(nodes_vector_size*sizeof(float)); //Allocating the memory required

    unsigned int offset;
    for (unsigned int row_index = 0; row_index < input_num_facets; row_index++){
        for (unsigned int col_index = 0; col_index < 3; col_index++){
            offset = row_index*3+col_index; // Row major order 
            my_part->facets2nodes[offset] = input_facets2nodes[offset];
        
        }

    }

    for (unsigned int row_index = 0; row_index < input_num_nodes; row_index++){
        for (unsigned int col_index = 0; col_index < 3; col_index++){
            offset = row_index*3+col_index;
            my_part->nodes2coords[offset] =input_nodes2coords[offset];
        
        }

    }

    glGenVertexArrays(1, &my_part->vertex_array_object);
    glGenBuffers(1, &my_part->vertex_buffer_object);
    glGenBuffers(1, &my_part->element_buffer_object);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), 
    //and then configure vertex attributes(s).
    glBindVertexArray(my_part->vertex_array_object);

    glBindBuffer(GL_ARRAY_BUFFER, my_part->vertex_buffer_object);
    glBufferData(GL_ARRAY_BUFFER, 3*input_num_nodes*sizeof(float), my_part->nodes2coords, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, my_part->element_buffer_object);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 3*input_num_facets*sizeof(unsigned int), my_part->facets2nodes, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // note that this is allowed, the call to glVertexAttribPointer registered VBO as the vertex attribute's bound vertex buffer object so afterwards we can safely unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0); 

    // remember: do NOT unbind the EBO while a VAO is active as the bound element buffer object IS stored in the VAO; keep the EBO bound.
    //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    // You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but this rarely happens. Modifying other
    // VAOs requires a call to glBindVertexArray anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
    glBindVertexArray(0); 
    

    
    return; 
}

void part_destroy(part *my_part) {
    free(my_part->facets2nodes);
    free(my_part->nodes2coords);
    return;
}

void part_print(part* my_part) {

    unsigned int node_index;
    unsigned int *nodes;
    float *coords; 
    for (unsigned int facet_index = 0; facet_index < my_part->num_facets; facet_index++){
        printf("Facet #%u:\n",facet_index);
        nodes = &(my_part->facets2nodes[3*facet_index]);
        
        for (unsigned int local_node_index = 0; local_node_index < 3; local_node_index++){
            node_index = my_part->facets2nodes[3*facet_index+local_node_index];
            coords = &(my_part->nodes2coords[3*node_index]);
            printf("Node #%u has coordinates: [ %lf, %lf, %lf]\n", nodes[local_node_index],
                                               coords[0], coords[1],coords[2]);

        }


    }


    return;
}



#endif