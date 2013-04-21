#include <stdio.h>
#include <stdlib.h>
#include <time.h>

struct  Workspace {
	int x, y, sx, sy, dx, dy;
	int *grid, *distance;
};
typedef struct Workspace workspace;

void disp (workspace *);
void input (workspace *);
void init (workspace *);
void obstacles (workspace *);
