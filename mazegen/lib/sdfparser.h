/**
 * SDFPARSER
 * A self-contained library to parse SDF files
 * and build a DOM (tree-based) model of the document
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

#include <string.h>
#include <stdint.h>

// -------------------------------------
// 
// SDF BASIC STRUCTURES
//
// -------------------------------------

/**
 * STRUCT SDF_STRING
 * Basic brick of an SDF document,
 * it contains a string and its length  
 */
struct sdf_string {
	char* 	buffer;						// string
	size_t 	length;						// length of the string
};

/**
 * STRUCT SDF_ATTRIBUTE
 * The SDF attribute struct is nothing but
 * a list, that contains the couple name/value
 * and a pointer to the next attribute
 * 
 * Ex. 	<tag attr1='value1' attr2='value2'>
 */
struct sdf_attribute {
	struct sdf_string* 		name;		// attribute name (attr2)
	struct sdf_string* 		value;		// attribute value ('value2')
	struct sdf_attribute* 	next;		// next attribute in chain (->attr1)
};

/**
 * STRUCT SDF_ELEMENT
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
	struct sdf_string* 		name;		// tag name (tag)
	struct sdf_string* 		content;	// tag content (NULL)
	struct sdf_attribute* 	attributes;	// attributes list (-> attr1)
	struct sdf_element* 	children;	// children node list (-> son)
	struct sdf_element* 	father;		// pointer to father (-> NULL)
	struct sdf_element* 	sibling;	// pointer to sibling (-> brother)
};

/**
 * STRUCT SDF_FILE
 * Represent an SDF file and contains filename,
 * the entire content (buffer) and its length
 */ 
struct sdf_file {
	char* 	filename;					// sdf file name
	char* 	buffer;						// file content
	size_t 	length;						// file content length
};

/**
 * STRUCT SDF_DOCUMENT
 * An SDF document, namely a struct representation
 * of an SDF file
 */ 
struct sdf_document {
	struct sdf_element* root;			// document root tag
};

// -------------------------------------
// 
// SYNTAX VALIDATION METHODS
//
// -------------------------------------

/**
 * Check for syntax error, wrt angular brackets (<>)
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
 * [IN] char*: name of the file in which write
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
// SDF ELEMENT METHODS (TO BE ADJ.)
//
// -------------------------------------

/**
 * Search a tag into an element e. Privilege children node
 * 
 * [IN] struct sdf_element*: pointer to SDF element in which operate search
 * [IN] char*: name of tag that must be searched
 * [OUT] struct sdf_element*: pointer to the element struct or NULL
 */
struct sdf_element* sdf_element_deep_search(struct sdf_element* e, char* tag_name);

/**
 * Search a tag into an element e. Avoid children nodes
 * 
 * [IN] struct sdf_element*: pointer to SDF element in which operate search
 * [IN] char*: name of tag that must be searched
 * [OUT] struct sdf_element*: pointer to the element struct or NULL
 */
struct sdf_element* sdf_element_search(struct sdf_element* e, char* tag_name);

/**
 * Search an attribute into an attribute list a
 * 
 * [IN] struct sdf_attribute*: pointer to SDF attribute list in which operate search
 * [IN] char*: name of the attribute that must be searched
 * [OUT] struct sdf_attribute*: pointer to the attribute struct or NULL
 */
struct sdf_attribute* sdf_attribute_search(struct sdf_attribute* a, char* attr_name);

/**
 * Append the element e to father as children
 * 
 * [IN] struct sdf_element**: pointer to SDF element in which append e
 * [IN] struct sdf_element*: element that must be appended
 * [OUT] void
 */
void sdf_element_append(struct sdf_element** father, struct sdf_element* e);

// -------------------------------------
// 
// SDF STRING METHODS
//
// -------------------------------------

/**
 * Replace the string contained into the struct with the new one
 * 
 * [IN] struct sdf_string*: pointer to SDF string in which modify buffer
 * [IN] char*: pointer to the new string (must be 0-termined)
 * [OUT] void
 */
void sdf_replace_string(struct sdf_string* s, char* new_str);