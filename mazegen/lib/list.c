/**
 * LIST
 * A simple linked list implementation using standard C library
 */

#include "list.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// -----------------------------------------------------
// PRIVATE METHOD
// -----------------------------------------------------

/**
 * Allocate a block of memory and fill with 0s. Exit in case of error 
 * 	
 * [IN] int: count of element to be allocated
 * [OUT] size_t: dimension of each element
 */
static void* alloc(int count, size_t dimension) {
	void* ret = calloc(count, dimension);

	// check if operation was performed
	if(ret != NULL)
		return ret;

	// Print and exit
	printf("Out of memory. Please restart your machine.");
	exit(-1);
}

// -----------------------------------------------------
// PUBLIC METHOD
// -----------------------------------------------------

/**
 * Initialize the list in order to be used
 * 
 * [IN] struct list*: pointer to list to be initialized
 * [OUT] void
 */
void list_init(struct list* l) {
    l->n = 0;
    l->root = NULL;
}

/**
 * Check if the list is empty
 * 
 * [IN] struct list*: pointer to list
 * [OUT] int: 1 if empty, 0 otherwise
 */
int list_is_empty(struct list* l) {
    if(!l->n)
        return 1;
    return 0;
}

/**
 * Add the provided info str to the list
 * 
 * [IN] struct list*: pointer to list
 * [IN] char*: str to be added to the list
 * [OUT] void
 */
void list_add_top(struct list* l, char* info) {
    struct node* n = alloc(1, sizeof(struct node));

    n->next = l->root;
    n->info = alloc(strlen(info)+1, sizeof(char));
    strcpy(n->info, info);

    l->n++;
    l->root = n;
}

/**
 * Remove the top element of the list
 * 
 * [IN] struct list*: pointer to list
 * [OUT] void
 */
void list_remove_top(struct list* l) {
    struct node* n;

    if(list_is_empty(l))
        return;
    
    n = l->root;
    l->root = l->root->next;
    l->n--;
    
    free(n->info);
    free(n);
}

/**
 * Return the pointer to info str of the top element of the list
 * 
 * [IN] struct list*: pointer to list
 * [OUT] char*: pointer to str info
 */
char* list_get_top_info(struct list* l) {
    return l->root->info;
}