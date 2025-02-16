//Name:        ADOT team 
//Date:        09/08/2024
//Description: Some test code to demonstrate how to use C.
#include <stdlib.h> 
#include <stdio.h>
//#include "mex.h"
#include "part.h"



int main(void){

    unsigned int facets2nodes[3] =  {0, 1, 2};
    double nodes2coords[9] = {0.0, 0.0, 0.0, 
                              0.0, 1.0, 0.0,
                            1.0, 0.0, 0.0};
    unsigned int num_facets = 1;
    unsigned int num_nodes = 3;

    part my_part; //Declare a part.
    part_initialise(&my_part, facets2nodes, nodes2coords,num_facets, num_nodes);

    part_print(&my_part);


    unsigned int repeat_value = my_part.num_facets; //The dot . is used to extract a member of
                                                    //an object proper.

    //printf("My number's address is %x.\n",&my_number );
    //printf("my number is %u. \n", my_number);


    part_destroy(&my_part);


    return 0;
}