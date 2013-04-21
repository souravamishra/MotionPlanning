#include "util.h"

void init (workspace *w) {

	int i, j;
	w->grid = (int *)malloc( w->x * w->y * sizeof(int));
	w->distance = (int *)malloc( w->x * w->y * sizeof(int));
	
	for (i = 0; i < w->x; i++)
		for (j = 0; j < w->y; j++)
			w->grid[i * w->y + j] = 0;
			
	for (i = 0; i < w->x; i++)
		for (j = 0; j < w->y; j++)
			w->distance[i * w->y + j] = 0;
}

void obstacles (workspace *w)  {

	int i, j, t, c;
	t = w->x * w->y / 5;
	
	for (c = 0; c < t; c++) {
		i= rand()%(w->x);
		j=rand()%(w->y);
		w->grid[i * w->y + j] = -1;
	}
	disp(w);
}

void disp (workspace *w) {

	int i, j;
	printf("\nDisplaying the workspace:");
	for (i = 0; i < w->x; i++) {
		printf("\n");
		for (j = 0; j < w->y; j++)
			printf("\t%d", w->grid[i * w->y + j]);
	}
	printf("\n-----------------------------------------");
	printf("\nDisplaying the distance matrix:");
	for (i = 0; i < w->x; i++) {
		printf("\n");
		for (j = 0; j < w->y; j++)
			printf("\t%d", w->distance[i * w->y + j]);
	}
	printf("\n");
}

void input (workspace *w) {
	/*printf ("\n Enter the size of the grid:");
	scanf ("%d%d", &w->x, &w->y);
	printf ("\n Enter source coordinate(for x, y >= 0):");
	scanf ("%d%d", &w->sx, &w->sy);
	printf ("\n Enter destination coordinate(for x, y >= 0):");
	scanf ("%d%d", &w->dx, &w->dy); */
	
	w->x  = 5; w->y  = 5;
	w->sx = 1; w->sy = 1;
	w->dx = 3; w->dy = 3;
}
