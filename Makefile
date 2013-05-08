CC		= gcc
CFLAGS		= -Wall
FLAG		= -g -O0
objects		= Navigation.o motionplanning.o util.o
executables	= motionplanning 

all: motionplanning 
	
util.o: util.h util.c
	$(CC) $(FLAG) $(CFLAGS) -c util.c

motionplanning.o: motionplanning.c util.h 
	$(CC) $(FLAG) $(CFLAGS) -c motionplanning.c
 
motionplanning: motionplanning.o util.o Navigation.o 
 
	$(CC) -o motionplanning motionplanning.o util.o Navigation.o
	
Navigation.o: Navigation.h Navigation.c util.h
	$(CC) $(FLAG) $(CFLAGS) -c Navigation.c

.PHONY: clean

clean:
	rm -f $(objects)
	rm -f $(executables)
	#rm *~ *.aux *.log

