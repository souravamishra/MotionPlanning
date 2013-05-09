CC		= gcc
CFLAGS		= -Wall
objects		= navigation.o motionplanning.o util.o
executables	= motionplanning navigation

all: motionplanning 
	
util.o: util.h util.c
	$(CC) $(CFLAGS) -c util.c
	
navigation.o: navigation.h navigation.c util.h
	$(CC) $(CFLAGS) -c navigation.c

motionplanning.o: motionplanning.c util.h 
	$(CC) $(CFLAGS) -c motionplanning.c
 
motionplanning: motionplanning.o util.o navigation.o 
	$(CC) -o motionplanning motionplanning.o util.o navigation.o

.PHONY: clean

clean:
	rm -f $(objects)
	rm -f $(executables)
	rm -f *~ *.aux *.log
