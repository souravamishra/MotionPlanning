#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

struct  Workspace {
	int nbrows, nbcolumns, sx, sy, dx, dy;
	int **grid, **distance;
	int *pathx, *pathy;			// arrays to store the path from source to destination
};
typedef struct Workspace workspace;
int nbrows, nbcolumns;				// just to avoid too many arrows

void	disp_grid (workspace *);
void	disp_distance(workspace *);
void	input (workspace *);
void	init (workspace *);
int	obstacles (workspace *);
void	reset_grid(workspace *);		//required if the workspace is modified in the algorithm
int	test(int, int );			// to test if the indices are not outside
int	nb_possible_ngbrs(int , int , workspace*); // to find the no. of neighbors of the cell (i,j)
void	display_path(workspace *);		// to display the path found from source to destination
void	push(int*, int );			// push a in the queue
int	pull(int* );				//pull an element from the queue
int	queueIsEmpty(int* );			//check if queue is empty
void	print_queue(int* , int*);		//print the two queues containing the indices
