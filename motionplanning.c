#include "Navigation.h"
int main() {	

	workspace w;
	input (&w);
	init (&w);

	if(obstacles (&w)==1) {
    printf("The destination or the source is at an obstacle\n");
    return 1;

  } // destination is at the obstacle

	computeDistance(&w);
	printf("After applying the wavefront algo");
	disp_distance(&w);
  ///Reset the workspace as it was modified to know the visited cells
	reset_grid(&w);
	if(path_finder(&w)==0)
	display_path(&w);
  else printf("Path could not be found");

	printf("\nResult of A* algo\n");

  A_star(&w);	
	
	return 0;
}
