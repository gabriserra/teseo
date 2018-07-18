/**
 * MAZE
 * Implementation depth-first-search algorithm
 * on a simple graph in order to build a maze
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

#ifndef MAZE_H
#define MAZE_H

#include <stdint.h>

# define RIGHT_DIR  0b00000001
# define DOWN_DIR   0b00000010
# define LEFT_DIR   0b00000100
# define UP_DIR     0b00001000
# define ANY_DIR    0b00001111
# define NO_DIR     0b00000000  

/**
 * ENUM BLOCK
 * Possible type of a cell of graph
 */
enum block {
    NONE,               // no block (free place)
    WALL                // wall
};

/**
 * STRUCT NODE
 * Contains info for each node of the graph
 */
struct node {
    struct node*    parent; // a pointer to parent node (if one)
    uint8_t         x;      // x position of the graph-node in the maze
    uint8_t         y;      // y position of the graph-node in the maze
    uint8_t         dirs;   // directions to be explored
    enum block      type;   // type of block to be drawn
    void*           info;   // optional linkable info
};

/**
 * STRUCT MAZE
 * Contains the entire graph and maze bounds
 */
struct maze {
    struct node*    graph;  // root of the graph
    uint8_t         width;  // maze width (number of block)
    uint8_t         height; // maze height (number of block)
};

/**
 * Init the maze 
 * 
 * [IN]     maze*: pointer to the maze struct to be initialized
 * [IN]     uint8_t: maze width (number of block)
 * [IN]     uint8_t: maze height (number of block)
 * [OUT]    int: -1 in case of low mem availability, 0 in case of success
 */
int init_maze(struct maze* m, uint8_t width, uint8_t height);

/**
 * Explore the graph in order to create the maze
 * 
 * [IN]     maze*: pointer to the maze struct
 * [OUT]    void
 */
void create_maze(struct maze* m);

/**
 * Draw into the terminal screen a visual representation of the maze
 * 
 * [IN]     maze*: pointer to the maze struct
 * [OUT]    void
 */
void draw_maze(struct maze* m);


#endif