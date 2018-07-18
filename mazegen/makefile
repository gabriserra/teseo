#--------------------------------------------------- 
# Target file to be compiled by default
#---------------------------------------------------
MAIN = main
#---------------------------------------------------
# CC will be the compiler to use
#---------------------------------------------------
CC = gcc
#---------------------------------------------------
# CFLAGS will be the options passed to the compiler
#---------------------------------------------------
CFLAGS = -Wall
#--------------------------------------------------- 
# Dependencies 
#---------------------------------------------------
$(MAIN): $(MAIN).o sdfparser.o list.o maze.o
	$(CC) $(CFLAGS) -o $(MAIN) $(MAIN).o sdfparser.o list.o maze.o
	make objclean
	
$(MAIN).o: $(MAIN).c 
	$(CC) -c $(MAIN).c

sdfparser.o: lib/sdfparser.c
	$(CC) -c lib/sdfparser.c
	
list.o: lib/list.c
	$(CC) -c lib/list.c

maze.o: lib/maze.c
	$(CC) -c lib/maze.c
#--------------------------------------------------- 
# Inline commands
#---------------------------------------------------
clean:
	rm -rf *o *world $(MAIN)

objclean:
	rm -rf *o