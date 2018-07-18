/**
 * MAZE
 * Implementation depth-first-search algorithm
 * on a simple graph in order to build a maze
 */

#include "maze.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// -----------------------------------------------------
// PRIVATE METHOD
// -----------------------------------------------------

/**
 * Init a node of the graph 
 * 
 * [IN]     node*: pointer to the node to be initialized as maze component
 * [IN]     uint8_t: node position in the maze (x position)
 * [IN]     uint8_t: node position in the maze (y position)
 * [IN]     uint8_t: explorable directions from the node towards neighbors
 * [IN]     block: primitive type of block (wall, free, ..)
 * [OUT]    void
 */
static void init_node(struct node* n, uint8_t i, uint8_t j, uint8_t dirs, enum block type) {
    n->x = i;
    n->y = j;
    n->dirs = dirs;
    n->type = type;
}

/**
 * Remove wall between start and dest node 
 * NOTE: start and dest must be adjacent
 * 
 * [IN]     maze*: maze structure
 * [IN]     node*: pointer of first node
 * [OUT]    node*: pointer of second node
 */
static void remove_wall(struct maze* m, struct node* start, struct node* dest) {
    uint8_t         x, y;   // x, y coordinate of node between start and dest
    struct node*    mid;    // pointer of node between start and dest
    
    // compute coordinate of node between start and dest
    x = start->x + (dest->x - start->x) / 2;
    y = start->y + (dest->y - start->y) / 2;

    // get node pointer and change type
    mid = m->graph + (x * m->width) + y;
    mid->type = NONE;     
}

/**
 * Return a pointer to neighbor obtained from "start" following "dir" 
 * 
 * [IN]     maze*: maze structure
 * [IN]     node*: pointer of start
 * [OUT]    uint8_t: direction of path
 */
static struct node* get_neighbor(struct maze* m, struct node* start, uint8_t dir) {
    struct node* neighbor = NULL;

    switch (dir) {
        case RIGHT_DIR:
            if (start->x + 2 < m->height)
                neighbor = m->graph + ((start->x + 2) * m->width) + start->y;
            break;
        case DOWN_DIR:
            if (start->y + 2 < m->width)
                neighbor = m->graph + (start->x * m->width) + (start->y + 2);            
            break;        
        case LEFT_DIR:
            if (start->x - 2 >= 0)
                neighbor = m->graph + ((start->x - 2) * m->width) + start->y;            
            break;        
        case UP_DIR:
            if (start->y - 2 >= 0)
                neighbor = m->graph + (start->x * m->width) + (start->y - 2);            
            break;
        default:
            neighbor = NULL;        
    }

    return neighbor;
}

/**
 * Link the provided node to a random neighbor (if possible) and return
 * the pointer of the neighbor
 * 
 * [IN]     node*: pointer to node from which start
 * [OUT]    node*: pointer to neighbor visited
 */
static struct node* link(struct maze* m, struct node* start) {
	uint8_t         new_dir;    // randomly generated direction
	struct node*    neighbor;   // pointer to neighbor
	
	if (start == NULL) 
        return NULL;
	
	// while there are directions still unexplored
	while (start->dirs) {
		new_dir = (1 << (rand() % 4));
		
		// if it has already been explored re-try
		if (new_dir & ~start->dirs) 
            continue;
		
		// mark direction as explored and get neighbor
		start->dirs &= ~new_dir;		
		neighbor = get_neighbor(m, start, new_dir);

        // if neighbor do not exists or is an already linked node then abort
		if (neighbor == NULL || neighbor->parent != NULL) 
            continue;
		
		// make sure that neighbor node is not a wall
		if (neighbor->type == NONE) {
			
			// adopt node and remove wall between them
			neighbor->parent = start;
            remove_wall(m, start, neighbor);
			
			// return address of the child node
			return neighbor;
		}
	}
	
	// if neighbor can't be linked, turn backwards (return parent's address)
	return start->parent;
}

// -----------------------------------------------------
// PUBLIC METHOD
// -----------------------------------------------------

/**
 * Init the maze 
 * 
 * [IN]     maze*: pointer to the graph to be initialized as maze
 * [IN]     uint8_t: maze width (number of block)
 * [IN]     uint8_t: maze height (number of block)
 * [OUT]    int: -1 in case of low mem availability, 0 in case of success
 */
int init_maze(struct maze* m, uint8_t width, uint8_t height) {
	int             i, j;       // iterate over the graph
    struct node*    n;          // temp pointer to nodes
	
	// allocate memory for graph
	m->graph = calloc(width * height, sizeof(struct node));
	
	// out of memory
    if (m->graph == NULL) 
        return -1;
		
    // fill up remaining info
    m->width = width;
    m->height = height;

	// setup whole graph
	for (i = 0; i < height; i++) {
		for (j = 0; j < width; j++) {

            // iterate over graph
            n = m->graph + (i * width) + j;            

			if (i*j % 2)
                init_node(n, i, j, ANY_DIR, NONE);
			else
                init_node(n, i, j, NO_DIR, WALL);
        }
    }

	return 0;
}

/**
 * Explore the graph in order to create the maze
 * 
 * [IN]     maze*: pointer to the maze struct
 * [OUT]    void
 */
void create_maze(struct maze* m) {
    struct node* start;    // node from which begin to explore graph
    struct node* neighbor; // node reached at each step of exploration

    // start from upper right node of graph and set itself as father
	start = m->graph + m->width + 1; 
	start->parent = start;

    // with this kind of choice, the entire maze will be explored
	neighbor = start;

    // initialize random seed
    srand(time(NULL));

    do {
        neighbor = link(m, neighbor);
    } while(neighbor != start);
}

/**
 * Draw into the terminal screen a visual representation of the maze
 * 
 * [IN]     maze*: pointer to the maze struct
 * [OUT]    void
 */
void draw_maze(struct maze* m) {
	int i, j;   // iterate over the graph

	for (i = 0; i < m->height; i++) {
		for (j = 0; j < m->width; j++)
            if(m->graph[i * m->width + j].type == WALL)
			    printf("%s", "█");
            else
                printf("%s", " ");
		printf("\n");
    }
}