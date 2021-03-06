#include "util.h"

void	computeDistance(workspace* );			// computes the distance matrix
int	path_finder(workspace*);			// finds the path using gradient descent
void	A_star(workspace*);				// A*star implementation

int	*minimum_dist_ngbr(workspace*, int, int );	//closest neighbor to i,j
int	heuristic_cost_estimate(int, int, int, int);	//manhattan distance

void	construct_path(int**, int** , int, int, int, int);// construct path in A*
int	min_fscore_openlist(int**, int*, int*);		//min in the f_score list
int	*abscissa_ngbrs(int, int, workspace*);		//find the abscissa of neighbors
int	*ordinate_ngbrs(int, int, workspace*);		//find the ordinated of neighbors
int	in_list(int , int , int *, int* );		// check if the elements are inthe lists
void	remove_min(int*, int );
