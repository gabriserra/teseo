/**
 * MAZEGEN
 * Generate a maze of custom dimensions and create an SDF file
 * that can be used as world for Gazebo simulator.
 * 
 * Compile: make
 * Usage: ./main <rows> <column>
 *
 * BSD 2-Clause License
 *
 * Copyright (c) 2018, Gabriele Ara, Gabriele Serra
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "lib/sdfparser.h"
#include "lib/maze.h"
#include <stdlib.h>
#include <stdio.h>

// ----------------------------
// STRING LENGTH
// ---------------------------.

#define MAX_POSE_LEN    30
#define MAX_NAME_LEN    20

// ------------------------------------
// MAIN SDF COMPONENT PATH AND SETTINGS
// ------------------------------------

#define WORLD_FILE      "sdf-element/world.sdf"
#define BOX_FILE        "sdf-element/box.sdf"
#define BOX_DIM         0.5

// ----------------------------
// ADDITIONAL SDF COMPONENT
// ---------------------------.

#define NUM_FILES       4
#define LIGHT_FILE      "sdf-element/light.sdf"     
#define GUI_FILE        "sdf-element/gui.sdf"
#define GROUND_FILE     "sdf-element/ground.sdf"
#define PHYSICS_FILE    "sdf-element/physics.sdf"
char* names[NUM_FILES] = {LIGHT_FILE, GUI_FILE, GROUND_FILE, PHYSICS_FILE};

/**
 * Print the message and return the retval
 * [IN] message: message to be terminal-printed
 * [IN] retval: value to be returned to system
 * [OUT] void
 **/
void build_world(struct sdf_document* world_d);

/**
 * Print the message and return the retval
 * [IN] message: message to be terminal-printed
 * [IN] retval: value to be returned to system
 * [OUT] void
 **/
void print_and_die(char* message, int retval);

/**
 * Search for a tag and replace its content
 * [IN] struct sdf_element*: pointer to element in which search
 * [IN] char*: tag name
 * [IN] char*: new tag content
 * [OUT] void
 **/
void search_n_replace_cont(struct sdf_element* elem, char* tag, char* content);

/**
 * Search for a tag, its attribute name and substitute its value
 * [IN] struct sdf_element*: pointer to element in which search
 * [IN] char*: tag name
 * [IN] char*: attribute name
 * [IN] char*: new value
 * [OUT] void
 **/
void search_n_replace_attr(struct sdf_element* elem, char* tag, char* name, char* value);

/**
 * Build a box and add it to the world
 * [IN] message: message to be terminal-printed
 * [IN] retval: value to be returned to system
 * [OUT] void
 **/
void add_box(struct sdf_document* world, int box_id, float x, float y, float z);

/**
 * Generate a maze and print onto screen
 * [IN] struct maze*: maze will be stored here
 * [IN] char* w_str: must contain width in string version
 * [IN] char* h_str: must contain height in string version
 * [OUT] void
 **/
void generate_maze(struct maze* m, char* w_str, char* h_str);

int main(int argc, char* argv[]) {
    struct sdf_file world_f;
    struct sdf_document world_d;
    struct maze m;    
    int i, j;

    // usage infos
    if (argc < 3) {
        printf("Usage: %s <rows> <column>\n", argv[0]);
        exit(-1);
    }
	
    // open world file and parse it
    sdf_file_open(&world_f, WORLD_FILE);    
    sdf_document_create(&world_f, &world_d);

    // build the world using basic sdf-elements
    build_world(&world_d);

    // generate the maze
    generate_maze(&m, argv[1], argv[2]);

    // for each block of the maze, add a box into the 3D world
    for (i = 0; i < m.height; i++) {
		for (j = 0; j < m.width; j++)
            if(m.graph[i * m.width + j].type == WALL) {
                if(!((i == 0 && j == 0) || (i == 1 && j == 0)))
                    add_box(&world_d, i*m.width+j, i * BOX_DIM , j * BOX_DIM , 0);
            }
    }
    
    // export the document to file
    sdf_document_print(&world_d, "maze.world");
    
    // free memory
    sdf_document_close(&world_d);
    sdf_file_close(&world_f);

    // everything ok
    return 0;
}

/**
 * Print the message and return the retval
 * [IN] message: message to be terminal-printed
 * [IN] retval: value to be returned to system
 * [OUT] void
 **/
void print_and_die(char* message, int retval) {
    fprintf(stderr, "ERROR: %s \n", message);
    exit(retval);
}

/**
 * Search for a tag and replace its content
 * [IN] struct sdf_element*: pointer to element in which search
 * [IN] char*: tag name
 * [IN] char*: new tag content
 * [OUT] void
 **/
void search_n_replace_cont(struct sdf_element* elem, char* tag, char* content) {
    struct sdf_element* e;  // will contain the tag found
	
    // search the tag
    e = sdf_element_search(elem, tag);

    // die if not found (fatal error)
    if(e == NULL)
        print_and_die("Unable to find request tag.", -1);
    
    // replace the string
    sdf_replace_string(e->content, content);
}

/**
 * Search for a tag, its attribute name and substitute its value
 * [IN] struct sdf_element*: pointer to element in which search
 * [IN] char*: tag name
 * [IN] char*: attribute name
 * [IN] char*: new value
 * [OUT] void
 **/
void search_n_replace_attr(struct sdf_element* elem, char* tag, char* name, char* value) {
    struct sdf_element*     e;  // will contain the tag found
    struct sdf_attribute*   a;  // will contain the attribute found
	
    // search the tag
    e = sdf_element_search(elem, tag);

    // die if not found (fatal error)
    if(e == NULL)
        print_and_die("Unable to find request tag.", -1);

    a = sdf_attribute_search(e->attributes, name);

    // die if not found (fatal error)
    if(a == NULL)
        print_and_die("Unable to find request attribute.", -1);
    
    // replace the string
    sdf_replace_string(a->value, value);
}

/**
 * Print the message and return the retval
 * [IN] message: message to be terminal-printed
 * [IN] retval: value to be returned to system
 * [OUT] void
 **/
void build_world(struct sdf_document* world) {
    // files list
    int                 i;
    struct sdf_file     files[NUM_FILES];
    struct sdf_document documents[NUM_FILES];

    // for each files, open file, create document and append it to world
    for(i = 0; i < NUM_FILES; i++) {
        sdf_file_open(&files[i], names[i]);
        sdf_document_create(&files[i], &documents[i]);
        sdf_element_append(&world->root->children, documents[i].root);
    }
}

/**
 * Build a box and add it to the world
 * [IN] message: message to be terminal-printed
 * [IN] retval: value to be returned to system
 * [OUT] void
 **/
void add_box(struct sdf_document* world, int box_id, float x, float y, float z) {
    struct sdf_file     box_f;
    struct sdf_document box_d;
    char                pose[MAX_POSE_LEN];
    char                name[MAX_NAME_LEN];

    // open file and parse it
    sdf_file_open(&box_f, BOX_FILE);
    sdf_document_create(&box_f, &box_d);

    // clean buffer and sprintf new position and name
    memset(name, 0, MAX_NAME_LEN * sizeof(char));
    memset(pose, 0, MAX_POSE_LEN * sizeof(char));
    sprintf(name, "'Box_Red_%d'", box_id);
    sprintf(pose, "%.3f %.3f %.3f 0 0 0", x, y, z);

    // substitute name and position in document and append it
    search_n_replace_attr(box_d.root, "model", "name", name);
    search_n_replace_cont(box_d.root->children, "pose", pose);
    sdf_element_append(&world->root->children, box_d.root);
}

/**
 * Generate a maze and print onto screen
 * [IN] struct maze*: maze will be stored here
 * [IN] char* w_str: must contain width in string version
 * [IN] char* h_str: must contain height in string version
 * [OUT] void
 **/
void generate_maze(struct maze* m, char* w_str, char* h_str) {	
	int ret;    // check for return values
    int width;  // will contains width in numeric shape
    int height; // will contains height in numeric shape
    
    // get width from str
    ret = sscanf(w_str, "%d", &width);
	if (ret < 1)
		print_and_die("Invalid size 1 provided.", -1);

    // get height from str
	ret = sscanf(h_str, "%d", &height);
	if (ret < 1)
		print_and_die("Invalid size 2 provided.", -1);

    // width and height must be odd
	if (!(width % 2) || !(height % 2))
		print_and_die("Only odd size are valid.", -1);
	
    // width and height must be positive
	if (width <= 0 || height <= 0)
		print_and_die("Dimension must be positive.", -1);

    // init the maze graph
	ret = init_maze(m, width, height);

    // exit in case of memory exhaust
	if (ret != 0)
		print_and_die("Out of memory.", -1);

    // create the maze
    create_maze(m);

    // print onto screen
    draw_maze(m);
}

