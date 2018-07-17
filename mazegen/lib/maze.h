// -----------------------------------------------------
// MAZE_H
// Implementation depth-first-search algorithm
// on a simple graph in order to build a maze
// -----------------------------------------------------

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
 * Primitive type of maze-block
 **/
enum block { 
    NONE,               // no block (free place)
    WALL                // wall
};

/**
 * STRUCT NODE
 * Contains info for each node of the graph
 **/
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
 **/
struct maze {
    struct node*    graph;  // root of the graph
    uint8_t         width;  // maze width (number of block)
    uint8_t         height; // maze height (number of block)
};

// -----------------------------------------------------
// PUBLIC METHOD
// -----------------------------------------------------

/**
 * Init the maze 
 * 
 * [IN]     maze*: pointer to the maze struct to be initialized
 * [IN]     uint8_t: maze width (number of block)
 * [IN]     uint8_t: maze height (number of block)
 * [OUT]    int: -1 in case of low mem availability, 0 in case of success
 **/
int init_maze(struct maze* m, uint8_t width, uint8_t height);

/**
 * Explore the graph in order to create the maze
 * 
 * [IN]     maze*: pointer to the maze struct
 * [OUT]    void
 **/
void create_maze(struct maze* m);

/**
 * Draw into the terminal screen a visual representation of the maze
 * 
 * [IN]     maze*: pointer to the maze struct
 * [OUT]    void
 **/
void draw_maze(struct maze* m);


#endif