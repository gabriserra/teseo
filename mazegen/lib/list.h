/**
 * LIST
 * A simple linked list implementation using standard C library
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

#ifndef LIST_H
#define LIST_H

/**
 * STRUCT NODE
 * Represent each node of the list
 */
struct node {
    struct node*    next;   // contains the next node in list
    char*           info;   // contains the 0-terminated info string
};

/**
 * STRUCT NODE
 * Represent the list object
 */
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

#endif