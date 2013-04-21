CC		= gcc
CFLAGS		= -Wall
FLAG		= -g -O0
objects		= Navigation.o motionplanning.o
executables	= Navigation motionplanning

all: motionplanning Navigation
	
util.o: util.h util.c
	$(CC) $(FLAG) $(CFLAGS) -c util.c

motionplanning.o: motionplanning.c util.h
	$(CC) $(FLAG) $(CFLAGS) -c motionplanning.c
 
motionplanning: motionplanning.o util.o
	$(CC) -o motionplanning motionplanning.o util.o
	
Navigation.o: Navigation.c util.h
	$(CC) $(FLAG) $(CFLAGS) -c Navigation.c
 
Navigation: Navigation.o util.o
	$(CC) -o Navigation Navigation.o util.o

.PHONY: clean

clean:
	rm -f $(objects)
	rm -f $(executables)
	#rm *~ *.aux *.log

