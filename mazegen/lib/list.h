// LIST.H
// A simple implementation of list using std C.

struct node {
    struct node*    next;   // contains the next node in list
    char*           info;   // contains the 0-terminated info string
};

struct list {
    int             n;      // contains the number of element in the list
    struct node*    root;   // contains the root of the list
};

/**
 * Initialize the list in order to be used
 * 
 * [IN] struct list*: pointer to list to be initialized
 */
void list_init(struct list* l);

/**
 * Check if the list is empty
 * 
 * [IN] struct list*: pointer to list
 * [OUT] int: 1 if empty, 0 otherwise
 */
int list_is_empty(struct list* l);

/**
 * Add the provided info str to the list
 * 
 * [IN] struct list*: pointer to list
 * [IN] char*: str to be added to the list
 * [OUT] void
 */
void list_add_top(struct list* l, char* name);

/**
 * Remove the top element of the list
 * 
 * [IN] struct list*: pointer to list
 * [OUT] void
 */
void list_remove_top(struct list* l);

/**
 * Return the pointer to info str of the top element of the list
 * 
 * [IN] struct list*: pointer to list
 * [OUT] char*: pointer to str info
 */
char* list_get_top_info(struct list* l);