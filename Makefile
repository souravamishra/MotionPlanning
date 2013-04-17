CC=gcc

all: motionplanning

motionplanning.o: motionplanning.c
	$(CC) -c motionplanning.c

clean:
	rm *.o
	rm motionplanning
	rm *~ *.aux *.log

