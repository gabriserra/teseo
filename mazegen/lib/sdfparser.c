#include "sdfparser.h"
#include "list.h"
#include <stdlib.h>
#include <stdio.h>

// -------------------------------------
// 
// SDF PRIVATE STRUCTURES
//
// -------------------------------------

/**
 * Represent the current parser state
 */
enum parser_state {
	BEGIN,							// the parser will BEGIN to parse soon
	ANGLE_OPEN,						// the parser found < bracket
	ANGLE_CLOSE,					// the parser found > bracket
	TAG_OPEN,						// parser is analyzing a tag <....
	TAG_OPENED,						// parser conclude analysis of a tag <...>
	TAG_CLOSED						// parser conclude analysis and is closed <..></..>
};

/**
 * Represent a parser that is used to parse
 * a file and build a document
 */
struct sdf_parser {
	enum parser_state 	state;		// current state of parser
	int 				position;	// current parser position [0 - filelength]
	struct sdf_file* 	file;		// file that will be parsed
	struct sdf_element* elem;		// current element that is built
	struct list 		l;			// needed to check open/close pairing
};


// ---------------------------------------
//
// PRIVATE: UTILS
//
// ---------------------------------------

/**
 * Return the file size of a file 
 * 	
 * [IN] FILE*: file pointer
 * [OUT] long: file size
 */
long get_file_size(FILE* file) {
	long size;	// it will contain file size
	
	// seek to the end of file and get stream position
	fseek(file, 0L, SEEK_END);
	size = ftell(file);

	// rewind back to file begin and return the size
	rewind(file);
	return size;
}

/**
 * Allocate a block of memory and fill with 0s. Exit in case of error 
 * 	
 * [IN] int: count of element to be allocated
 * [OUT] size_t: dimension of each element
 */
void* alloc(int count, size_t dimension) {
	void* ret = calloc(count, dimension);

	// check if operation was performed
	if(ret != NULL)
		return ret;

	// Print and exit
	printf("Out of memory. Please restart your machine.");
	exit(-1);
}

/**
 * Print a number n of tabs in the file F 
 * 	
 * [IN] int: number of tabs
 * [IN] FILE*: file in which print
 * [OUT] void
 */
void print_tabs(int n, FILE* f) {
	int i;	// used to iterate

	for(i = 0; i < n; i++)
		fprintf(f, "%s", "\t");
}

/**
 * Allocate an sdf children element and update the passed pointer
 * 	
 * [IN] struct sdf_element**: pointer to sdf element in which create children
 * [OUT] void
 */
void use_children_elem(struct sdf_element** elem) {
	(*elem)->children = alloc(1, sizeof(struct sdf_element));
	(*elem)->children->father = (*elem);
	(*elem) = (*elem)->children;
}

/**
 * Allocate an sdf sibling element and update the passed pointer
 * 	
 * [IN] struct sdf_element*: pointer to sdf element in which create sibling
 * [OUT] void
 */
void use_sibling_elem(struct sdf_element** elem) {
	(*elem)->sibling = alloc(1, sizeof(struct sdf_element));
	(*elem)->sibling->father = (*elem)->father;
	(*elem) = (*elem)->sibling;
}

// ---------------------------------------
//
// PRIVATE: SDF PARSER BASIC FUNCTION
//
// ---------------------------------------

/**
 * Print an err string and die
 * 
 * [IN] char*: err str to be printed
 * [OUT] void
 */
void sdf_state_ex(char* err) {
	printf("Fatal: %s\n", err);
	exit(-1);
}

/**
 * Advance parser adding n position to pointer
 * 
 * [IN] struct parser*: pointer to an sdf parser
 * [IN] int: number of char to skip
 * [OUT] void
 */
void sdf_skip_char(struct sdf_parser* p, int n) {
	p->position = p->position + n;
}

/**
 * Skip all characted until a new tag is found
 * 
 * [IN] struct parser*: pointer to an sdf parser
 * [OUT] void
 */
void sdf_go_next_tag(struct sdf_parser* p) {
	while(p->file->buffer[p->position] != '<')
		if(p->position < p->file->length)
			p->position++;
		else
			break;
}

/**
 * Skip all characted until a new tag is found and
 * return the next token pointer without updating the current one
 * 
 * [IN] struct parser*: pointer to an sdf parser
 * [OUT] int: next token pointer
 */
int sdf_go_next_tag_dry(struct sdf_parser* p) {
	int temp_pointer = p->position;

	while(p->file->buffer[temp_pointer] != '<')
		temp_pointer++;
	
	return temp_pointer;
}

/**
 * Take the parser position forward until a new brackets is found
 * 
 * [IN] struct parser*: pointer to the parser
 * [OUT] void
 */
void sdf_next_token(struct sdf_parser* p) {
	while(p->file->buffer[p->position] != '>')
		p->position++;
}

/**
 * Take the parser position forward no whitespaces found
 * 
 * [IN] struct parser*: pointer to the parser
 * [OUT] void
 */
void sdf_skip_whitespaces(struct sdf_parser* p) {
	while(p->file->buffer[p->position] == ' ')
		p->position++;
}

// ---------------------------------------
//
// PRIVATE: SDF PARSER FEATURES EXTRACTION
//
// ---------------------------------------

/**
 * Extract the next feature (attribute, content, tag name) and
 * leave the result in sdf_string s. Use the each of the separator
 * passed in sep. 
 * 
 * [IN] struct parser*: pointer to the parser
 * [OUT] void
 */
void sdf_feature_extract(struct sdf_parser* p, struct sdf_string** s, char* sep) {
	int new_p_pos;	// temporary buffer pointer
	int sep_i;		// iterate over separator
	int check = 1;	// used to check logic condition over separator

	// let's start from current pointer
	new_p_pos = p->position;

	// continue until one of the sepatator is found
	while(1) {
		for(sep_i = 0; sep_i < strlen(sep); sep_i++)
			check = check && (p->file->buffer[new_p_pos] != sep[sep_i]);

		if(check)
			new_p_pos++;
		else
			break;
	}
	
	// allocate the struct and check for memory problem
	(*s) = alloc(1, sizeof(struct sdf_string));
	(*s)->length = new_p_pos - p->position;
	(*s)->buffer = alloc((*s)->length, sizeof(char));

	// fill the string, update the buffer pointer to skip =' and return
	strncpy((*s)->buffer, p->file->buffer+(p->position+1), (*s)->length-1);
	p->position = new_p_pos;
}

/**
 * Extract the entire list of attributes of a tag
 * 
 * [IN] struct parser*: pointer to the parser
 * [OUT] void
 */
void sdf_attributes_extract(struct sdf_parser* p) {
	struct sdf_attribute* attribute; 	// new attribute

	// ensure to be in a correct position
	if(p->file->buffer[p->position] != ' ')
		return;

	// extract names and values
	while(p->file->buffer[p->position] != '>' 
			&& p->file->buffer[p->position] != '/') {

		// allocate attribute struct and check for memory problem
		attribute = alloc(1, sizeof(struct sdf_attribute));

		// extract names and values
		sdf_feature_extract(p, &(attribute->name), "=");
		sdf_feature_extract(p, &(attribute->value), "> /");

		// attach attribute to head of list
		attribute->next = p->elem->attributes;
		p->elem->attributes = attribute;
	}
}

/**
 * Extract the content of a tag 
 * <tag>CONTENT</tag>
 * 
 * [IN] struct parser*: pointer to the parser
 * [OUT] void
 */
void sdf_content_extract(struct sdf_parser* p) {
	// ensure to be in a correct position
	if(p->file->buffer[p->position] != '>')
		return;

	// extract feature content
	sdf_feature_extract(p, &(p->elem->content), "<");
}

/**
 * Extract the tag name from a tag
 * 
 * [IN] struct parser*: pointer to the parser
 * [OUT] void
 */
void sdf_element_extract(struct sdf_parser* p) {
	// ensure to be in a correct position
	if(p->file->buffer[p->position] != '<')
		return;

	// extract feature name
	sdf_feature_extract(p, &(p->elem->name), " >/");
}

/**
 * Extract the tag name from a tag closing and leave the result
 * in the element passed
 * 
 * [IN] struct parser*: pointer to the parser
 * [IN] struct sdf_element*: pointer to the element in which result is left
 * [OUT] void
 */
void sdf_close_extract(struct sdf_parser* p, struct sdf_element* elem) {
	// ensure to be in a correct position
	if(p->file->buffer[p->position] != '<')
		return;
	
	// skip </ token
	sdf_skip_char(p, 1);

	// extract close tag name
	sdf_feature_extract(p, &(elem->name), ">");
}

// ---------------------------------------
//
// PRIVATE: SDF PARSER TAG HANDLER
//
// ---------------------------------------

/**
 * Handle closing tag </ found by parser
 * 
 * [IN] struct parser*: pointer to the parser
 * [OUT] void
 */
void sdf_close_tag(struct sdf_parser* p) {
	struct sdf_element elem; // it will contain close tag name for validation

	// extract close tag name
	sdf_close_extract(p, &elem);

	// a close tag must follow an open tag
	if(list_is_empty(&p->l))
		sdf_state_ex("found </ (close token) in a wrong position. Check your SDF file.");

	// check open/close constraint
	if(strcmp(list_get_top_info(&p->l), elem.name->buffer))
		sdf_state_ex("not valid SDF file. Check that close tag follow its open tag.");
	
	// remove just closed tag
	list_remove_top(&p->l);

	// return to father element
	if(p->state == TAG_CLOSED)
		p->elem = p->elem->father;
	
	// skip all whitespaces until a new tag is found
	sdf_go_next_tag(p);

	// update current state
	p->state = TAG_CLOSED;
}

/**
 * Handle self closing tag <../> found by parser
 * 
 * [IN] struct parser*: pointer to the parser
 * [OUT] void
 */
void sdf_self_close_tag(struct sdf_parser* p) {
	// file is not in a valid format
	if(p->state != TAG_OPEN)
		sdf_state_ex("found /> (self-close token) in a wrong position. Check your SDF file.");

	if(p->state == TAG_CLOSED)
		p->elem = p->elem->father;

	// skip all whitespaces until a new tag is found
	sdf_go_next_tag(p);

	// update current state
	p->state = TAG_CLOSED;
}

/**
 * Handle comment tag <!-- ... --> found by parser
 * 
 * [IN] struct parser*: pointer to the parser
 * [OUT] void
 */
void sdf_comment_tag(struct sdf_parser* p) {
	// file is not in a valid format
	if(p->state == TAG_OPEN)
		sdf_state_ex("found <! (comment token) in a wrong position. Check your SDF file.");

	// skip the entire comment until a new tag is found
	sdf_next_token(p);
	sdf_go_next_tag(p);
}

/**
 * Handle open tag <tag found by parser
 * (Decide if add the element in children or sibling, extract
 * tag name and attributes and update parser state)
 * 
 * [IN] struct parser*: pointer to the parser
 * [OUT] void
 */
void sdf_new_tag_open(struct sdf_parser* p) {
	// file is not in a valid format
	if(p->state == TAG_OPEN)
		sdf_state_ex("found < (new tag open token) in a wrong position. Check your SDF file.");
	
	// create sibling or children element
	if (p->state == TAG_CLOSED)
		use_sibling_elem(&p->elem);
	else if (p->state == TAG_OPENED)
		use_children_elem(&p->elem);
		
	// an open tag was found, so update parser state
	p->state = TAG_OPEN;

	// extract features
	sdf_element_extract(p);
	sdf_attributes_extract(p);

	// skip possible whitespaces to leave parser clean situation
	sdf_skip_whitespaces(p);
}

/**
 * Handle close brackets of open tag ...> found by parser
 * (Check for the next token, if comment of another open go forward,
 * else extract content, put tag in list and update parser state)
 * 
 * [IN] struct parser*: pointer to the parser
 * [OUT] void
 */
void sdf_new_tag_close(struct sdf_parser* p) {
	int next_tag;	// temporary pointer

	// file is not in a valid format
	if(p->state != TAG_OPEN)
		sdf_state_ex("found > (new tag close token) in a wrong position. Check your SDF file.");

	// next token search
	next_tag = sdf_go_next_tag_dry(p);

	// discriminate tag
	if(!strncmp(p->file->buffer+next_tag, "</", 2))
		sdf_content_extract(p);
	else if(!strncmp(p->file->buffer+next_tag, "<!", 2))
		sdf_go_next_tag(p);
	else if(!strncmp(p->file->buffer+next_tag, "<", 1))
		sdf_go_next_tag(p);
	else
		sdf_state_ex("not found an open tag after > (new tag close token). Check your SDF file.");

	p->state = TAG_OPENED;
	list_add_top(&p->l, p->elem->name->buffer);
}

// ---------------------------------------
//
// PRIVATE: FREE MEMORY 
//
// ---------------------------------------

/**
 * Recursive: free all allocated things of an attribute
 * 
 * [IN] struct sdf_attribute*: pointer to the attribute to be freed
 * [OUT] void
 */
void sdf_attribute_close(struct sdf_attribute* attributes) {
	if(attributes == NULL)
		return;

	sdf_attribute_close(attributes->next);
	free(attributes->name->buffer);
	free(attributes->value->buffer);
	free(attributes->name);
	free(attributes->value);
	free(attributes);
}

/**
 * Free string related to the content of a tag
 * 
 * [IN] struct sdf_string*: pointer to the string to be freed
 * [OUT] void
 */
void sdf_content_close(struct sdf_string* content) {
	free(content->buffer);
	free(content);
}

/**
 * Free string related to the name of a tag
 * 
 * [IN] struct sdf_string*: pointer to the string to be freed
 * [OUT] void
 */
void sdf_name_close(struct sdf_string* name) {
	free(name->buffer);
	free(name);
}

/**
 * Recursive: free all contents of an element, its children and siblings
 * 
 * [IN] struct sdf_element*: pointer to the element to be freed
 * [OUT] void
 */
void sdf_element_close(struct sdf_element* element) {
	if(element == NULL)
		return;

	if(element->content != NULL)
		sdf_content_close(element->content);
	if(element->name != NULL)
		sdf_name_close(element->name);
	if(element->attributes != NULL)
		sdf_attribute_close(element->attributes);

	sdf_element_close(element->children);
	sdf_element_close(element->sibling);

	free(element);
}

// ---------------------------------------
//
// PUBLIC: SYNTAX CHECKER
//
// ---------------------------------------

/**
 * Check if the content of an sdf file is correct wrt syntax <>
 * 
 * [IN] struct sdf_file*: pointer to the struct to be filled
 * [OUT] int: 1 if correct, 0 otherwise
 */
int syntax_check(struct sdf_file* file) {
	long 				i;			// iterate over file buffer
	enum parser_state	state;		// used to check current parser state

	// reset parser state
	state = BEGIN;

	// check for syntax consistency
	for(i = 0; i < (long)file->length; i++) {
		switch (file->buffer[i]) {
			case '<':
				if (state == ANGLE_OPEN)
					return 0;
				else 
					state = ANGLE_OPEN;
				break;
			case '>':
				if (state == ANGLE_CLOSE)
					return 0;
				else 
					state = ANGLE_CLOSE;				
				break;
			default:
				break;
		}
	}

	return 1;
}

// ---------------------------------------
//
// PRIVATE: STRING PRINTER
//
// ---------------------------------------

/**
 * Print into file F attribute list specified
 * 
 * [IN] struct sdf_attribute*: pointer to the struct that contains attribute
 * [OUT] FILE* f: 1 if correct, 0 otherwise
 */
void sdf_attribute_print(struct sdf_attribute* attr, FILE* f) {
	if(attr == NULL)
		return;

	fprintf(f, " %s=%s", attr->name->buffer, attr->value->buffer);
	sdf_attribute_print(attr->next, f);
}

/**
 * Print into file F the element specified
 * 
 * [IN] struct sdf_attribute*: pointer to the struct that contains attribute
 * [OUT] FILE* f: 1 if correct, 0 otherwise
 */
void sdf_element_print(struct sdf_element* e, int tab_level, FILE* f) {
	// print tabs and tag name
	print_tabs(tab_level, f);
	fprintf(f, "<%s", e->name->buffer);
	sdf_attribute_print(e->attributes, f);
	
	// print the correct open/close pair
	if(e->content != NULL) {
		fprintf(f, ">%s</%s>\n", e->content->buffer, e->name->buffer);
	} else if(e->children != NULL) {
		fprintf(f, ">\n");
		sdf_element_print(e->children, tab_level+1, f);
		print_tabs(tab_level, f);
		fprintf(f, "</%s>\n", e->name->buffer);
	} else {
		fprintf(f, "/>\n");
	}

	// print also siblings
	if(e->sibling != NULL)
		sdf_element_print(e->sibling, tab_level, f);		
}

// ---------------------------------------
//
// PUBLIC: FILE METHODS
//
// ---------------------------------------

/**
 * Open an sdf file and copy the content into file structure
 * 
 * [IN] struct sdf_file*: pointer to the struct to be filled
 * [IN] uint8_t const*: sdf filename * 	
 * [OUT] int: 0 if correct
 */
int sdf_file_open(struct sdf_file* file, char const* filename) {
	FILE* 	sdf_input;
	long 	file_size;

	// open the file in read mode
	sdf_input = fopen(filename, "r");

	// unable to open file
	if(sdf_input == NULL)
		return -1;

	// get file size
	file_size = get_file_size(sdf_input);

	// fill structure information
	file->filename = alloc(strlen(filename), sizeof(char));
	file->length = (size_t)file_size;
	file->buffer = alloc(file_size + 1, sizeof(char));
	
	// copy file content into buffer and null-termine it
	strcpy(file->filename, filename);
	fread(file->buffer, file_size, 1, sdf_input);
	file->buffer[file_size] = '\0';
	
	// close the file
	fclose(sdf_input);
	return 0;
}

/**
 * Delete the memory allocated for file structure
 * 
 * [IN] struct sdf_file*: pointer to the struct to be freed
 * [OUT] void
 */
void sdf_file_close(struct sdf_file* file) {
	free(file->buffer);
}

// ---------------------------------------
//
// PUBLIC: DOCUMENT METHODS
//
// ---------------------------------------

/**
 * Parse an SDF file and create a SDF document with
 * element, attribute and so on. 
 * 
 * [IN] struct sdf_file*: pointer to sdf file to be analyzed
 * [IN] struct sdf_document*: result will be lefe here
 * [OUT] void
 */
void sdf_document_create(struct sdf_file* file, struct sdf_document* document) {
	struct sdf_parser p; // represent the parser

	// allocate the document internal struct and check for memory problem
	document->root = alloc(1, sizeof(struct sdf_element));

	// initialize parser
	p.file = file;
	p.position = 0;
	p.state = BEGIN;
	p.elem = document->root;
	list_init(&p.l);

	// until the file is not finished
	while(p.position < p.file->length) {
		if(!strncmp(p.file->buffer+p.position, "<!", 2))
			sdf_comment_tag(&p);
		else if(!strncmp(p.file->buffer+p.position, "</", 2))
			sdf_close_tag(&p);
		else if(!strncmp(p.file->buffer+p.position, "/>", 2))
			sdf_self_close_tag(&p);
		else if(!strncmp(p.file->buffer+p.position, ">", 1))
			sdf_new_tag_close(&p);
		else if(!strncmp(p.file->buffer+p.position, "<", 1))
			sdf_new_tag_open(&p);
	}
}

/**
 * Print the entire document into a file. Pass NULL to print
 * in the terminal
 * 
 * [IN] struct sdf_document*: pointer to SDF document to be printed
 * [IN] char* filename: file in which print
 * [OUT] void
 */
int sdf_document_print(struct sdf_document* d, char* filename) {
	FILE* 	sdf_output;

	// open the file in read mode or stdout
	if(filename == NULL)
		sdf_output = stdout;
	else
		sdf_output = fopen(filename, "w");

	// unable to open file
	if(sdf_output == NULL)
		return -1;
	
	// print on file
	sdf_element_print(d->root, 0, sdf_output);
	
	// close the file
	fclose(sdf_output);

	return 0;
}

/**
 * Close an SDF document and frees the allocated memory
 * 
 * [IN] struct sdf_document*: pointer to SDF document to be cancelled
 * [OUT] void
 */
void sdf_document_close(struct sdf_document* document) {
	sdf_element_close(document->root);
}

// ---------------------------------------
//
// PUBLIC: SEARCH IN ELEMENT (BETA)
//
// ---------------------------------------

/**
 * Search a tag into an element e. Privilege children node
 * 
 * [IN] struct sdf_element*: pointer to SDF element in which operate search
 * [IN] char*: name of tag that must be searched
 * [OUT] void
 */
struct sdf_element* sdf_element_deep_search(struct sdf_element* e, char* tag_name) {
	struct sdf_element *tag;

	if(e == NULL)
		return NULL;
	
	if(!strcmp(e->name->buffer, tag_name))
		return e;
	
	tag = sdf_element_search(e->children, tag_name);
	if(tag != NULL)
		return tag;
	
	tag = sdf_element_search(e->sibling, tag_name);
	if(tag != NULL)
		return tag;

	// not found
	return NULL;
}

/**
 * Search a tag into an element e. Avoid children nodes
 * 
 * [IN] struct sdf_element*: pointer to SDF element in which operate search
 * [IN] char*: name of tag that must be searched
 * [OUT] void
 */
struct sdf_element* sdf_element_search(struct sdf_element* e, char* tag_name) {
	struct sdf_element* tag;

	if(e == NULL)
		return NULL;
	
	if(!strcmp(e->name->buffer, tag_name))
		return e;
	
	tag = sdf_element_search(e->sibling, tag_name);
	if(tag != NULL)
		return tag;

	// not found
	return NULL;
}

struct sdf_attribute* sdf_attribute_search(struct sdf_attribute* a, char* attr_name) {
	if(a == NULL)
		return NULL;

	if(!strcmp(a->name->buffer, attr_name))
		return a;
	
	return sdf_attribute_search(a->next, attr_name);
}

// ---------------------------------------
//
// PUBLIC: APPEND TO ELEMENT (BETA)
//
// ---------------------------------------

/**
 * Append the element e to sibling as sibling
 * 
 * [IN] struct sdf_element**: pointer to SDF element in which append e
 * [IN] struct sdf_element*: element taht that must be appended
 * [OUT] void
 */
void sdf_element_append_sibling(struct sdf_element** sibling, struct sdf_element* e) {
	if((*sibling)->sibling == NULL)
		(*sibling)->sibling = e;
	else
		sdf_element_append_sibling(&(*sibling)->sibling, e);
}

/**
 * Append the element e to sibling as children
 * 
 * [IN] struct sdf_element**: pointer to SDF element in which append e
 * [IN] struct sdf_element*: element taht that must be appended
 * [OUT] void
 */
void sdf_element_append(struct sdf_element** father, struct sdf_element* e) {
	if((*father)->children != NULL)
		sdf_element_append_sibling(&(*father)->children, e);
	else
		(*father)->children = e;
}

void sdf_replace_string(struct sdf_string* s, char* new_str) {
	free(s->buffer);
	s->length = strlen(new_str);
	s->buffer = alloc(s->length+1, sizeof(char));
	strncpy(s->buffer, new_str, s->length);
}