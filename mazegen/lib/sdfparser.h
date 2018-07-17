#include <string.h>
#include <stdint.h>

// -------------------------------------
// 
// SDF BASIC STRUCTURES
//
// -------------------------------------

/**
 * Basic brick of an SDF document,
 * it contains a string and its length  
 */
struct sdf_string {
	char* buffer;						// string
	size_t length;						// length of the string
};

/**
 * The SDF attribute struct is nothing but
 * a list, that contains the couple name/value
 * and a pointer to the next attribute
 * 
 * Ex. 	<tag attr1='value1' attr2='value2'>
 */
struct sdf_attribute {
	struct sdf_string* name;			// attribute name (attr2)
	struct sdf_string* value;			// attribute value ('value2')
	struct sdf_attribute* next;			// next attribute in chain (->attr1)
};

/**
 * The SDF element is the main element of an SDF document.
 * It basically contain all the information about an element,
 * its children and its siblings
 * 
 * Ex. 	<tag attr1='value'>
 * 			<son>CONTENT</son>
 *		</tag>
 *		<brother></brother>
 */
struct sdf_element {
	struct sdf_string* name;			// tag name (tag)
	struct sdf_string* content;			// tag content (NULL)
	struct sdf_attribute* attributes;	// attributes list (-> attr1)
	struct sdf_element* children;		// children node list (-> son)
	struct sdf_element* father;			// pointer to father (-> NULL)
	struct sdf_element* sibling;		// pointer to sibling (-> brother)
};

/**
 * Represent an SDF file and contains filename,
 * the entire content (buffer) and its length
 */ 
struct sdf_file {
	char* filename;						// sdf file name
	char* buffer;						// file content
	size_t length;						// file content length
};

/**
 * An SDF document, namely a struct representation
 * of an SDF file
 * the entire content (buffer) and its length
 */ 
struct sdf_document {
	struct sdf_element* root;			// first tag
};

// -------------------------------------
// 
// SYNTAX VALIDATION METHODS
//
// -------------------------------------

/**
 * Delete the memory allocated for file structure
 * 
 * [IN] struct sdf_file*: pointer to the sdf file to check
 * [OUT] int: 0 if <> are not coupled, 1 in positive case
 */
int syntax_check(struct sdf_file* file);

// -------------------------------------
// 
// FILE METHODS
//
// -------------------------------------

/**
 * Open an sdf file and copy the content into file structure
 * 
 * [IN] struct sdf_file*: pointer to the struct to be filled
 * [IN] uint8_t const*: sdf filename * 	
 * [OUT] int: 0 if correct
 */
int sdf_file_open(struct sdf_file* file, char const* filename);

/**
 * Delete the memory allocated for file structure
 * 
 * [IN] struct sdf_file*: pointer to the struct to be freed
 * [OUT] void
 */
void sdf_file_close(struct sdf_file* file);

// -------------------------------------
// 
// SDF DOCUMENT METHODS
//
// -------------------------------------

/**
 * Parse an SDF file and create a SDF document with
 * element, attribute and so on. 
 * 
 * [IN] struct sdf_file*: pointer to sdf file to be analyzed
 * [IN] struct sdf_document*: result will be lefe here
 * [OUT] void
 */
void sdf_document_create(struct sdf_file* file, struct sdf_document* document);

/**
 * Export an SDF document into a file.
 * 
 * [IN] struct sdf_document*: document to be printed
 * [IN] char* filename: name of the file in which write
 * [OUT] void
 */
int sdf_document_print(struct sdf_document* d, char* filename);

/**
 * Close an SDF document and frees the allocated memory
 * 
 * [IN] struct sdf_document*: pointer to SDF document to be cancelled
 * [OUT] void
 */
void sdf_document_close(struct sdf_document* document);

// -------------------------------------
// 
// SDF ELEMENT METHODS (BETA)
//
// -------------------------------------

/**
 * Search a tag into an element e. Privilege children node
 * 
 * [IN] struct sdf_element*: pointer to SDF element in which operate search
 * [IN] char*: name of tag that must be searched
 * [OUT] struct sdf_element*:
 */
struct sdf_element* sdf_element_deep_search(struct sdf_element* e, char* tag_name);

/**
 * Search a tag into an element e. Avoid children nodes
 * 
 * [IN] struct sdf_element*: pointer to SDF element in which operate search
 * [IN] char*: name of tag that must be searched
 * [OUT] struct sdf_element*:
 */
struct sdf_element* sdf_element_search(struct sdf_element* e, char* tag_name);

/**
 * Search an into an attribute list a
 * 
 * [IN] struct sdf_attribute*: pointer to SDF attribute list in which operate search
 * [IN] char*: name of the attribute that must be searched
 * [OUT] struct sdf_attribute*:
 */
struct sdf_attribute* sdf_attribute_search(struct sdf_attribute* a, char* attr_name);

/**
 * Append the element e to sibling as children
 * 
 * [IN] struct sdf_element**: pointer to SDF element in which append e
 * [IN] struct sdf_element*: element taht that must be appended
 * [OUT] void
 */
void sdf_element_append(struct sdf_element** father, struct sdf_element* e);

// -------------------------------------
// 
// SDF STRING METHODS
//
// -------------------------------------

void sdf_replace_string(struct sdf_string* s, char* new_str);