CC=gcc
CFLAGS=-Wall
objects=Navigation.o motionplanning.o
all: motionplanning Navigation

motionplanning: motionplanning.o Navigation.o
	$(CC) $(CFLAGS) -o motionplanning motionplanning.o
	$(CC) $(CFLAGS) -o Navigation Navigation.o
.PHONY: clean
clean:
	rm -f $(objects)
	rm -f Navigation
	rm -f motionplanning
	#rm *~ *.aux *.log

