#include "navigation.h"

int main() {

	workspace w;
	input (&w);

	if(obstacles (&w) == 1) {
		printf("The destination or the source is at an obstacle\n");
		disp_grid(&w);
		return 1;
	} 		// destination is at the obstacle

	computeDistance(&w);
	printf("After applying the wavefront algorithm");
	disp_distance(&w);

	reset_grid(&w);	//Reset the workspace as it was modified to know the visited cells
	if(path_finder(&w)==0)
		display_path(&w);
  	else {
  		disp_grid(&w);
  		printf("Path could not be found");
  	}

	printf("\nResult of A* algo\n");
	A_star(&w);	
	
	return 0;
}
