#include "util.h"

void init (workspace *w) {

	int i, j;
	
	w->grid = malloc(w->nbrows * sizeof(int*));
	for(i=0;i<w->nbrows;i++)
		w->grid[i]=malloc(w->nbcolumns*sizeof(int));

	w->distance = malloc(w->nbrows * sizeof(int*));
        for(i=0;i<w->nbrows;i++)
		w->distance[i]=malloc(w->nbcolumns*sizeof(int));	
        
        w->pathx=malloc(w->nbrows*w->nbcolumns*sizeof(int));
        w->pathy=malloc(w->nbrows*w->nbcolumns*sizeof(int));
	
	for (i = 0; i < w->nbrows; i++)
		for (j = 0; j < w->nbcolumns; j++)
			w->grid[i][j] = 0;
			
	for (i = 0; i < w->nbrows ; i++)
		for (j = 0; j < w->nbcolumns; j++)
				w->distance[i][j] = 0;
	
  for(i=0;i<w->nbrows * w->nbcolumns;i++){
		w->pathx[i]=-1;
		w->pathy[i]=-1;
	}

	nbrows = w->nbrows;  //just to avoid arrows
	nbcolumns=w->nbcolumns;

}

int obstacles (workspace *w)  {

	int i, j;
  //////////////Random test/////////////////////	
 //int t,c;
  /*	t = w->nbrows * w->nbcolumns / 5;
	
	for (c = 0; c < t; c++) {
		i= rand()%(w->nbrows);
		j=rand()%(w->nbcolumns);
		w->grid[i][j] = -1;
	}
  */
	/////////////////////////////////////////////////


/////////////test 1////////////////////
/*
	w->grid[1][1]=-1;
	w->grid[2][1]=-1;
	w->grid[w->dx][w->dy]=1;  // Needed for the algo to work correctly
	for(i=0;i<w->nbrows;i++)
		for(j=0;j<w->nbcolumns;j++)
			if(w->grid[i][j]==-1)
				w->distance[i][j]=-1;  //the obstacles cannot be reached	
*/
  //////////////////////////////////////
  
  
/////test 2///////////////////////////
  w->grid[1][1]=-1;
  w->grid[1][2]=-1;
  w->grid[1][3]=-1;
  w->grid[2][1]=-1;
  w->grid[3][1]=-1;
  w->grid[3][2]=-1;
  w->grid[3][3]=-1;

  
  /*DO MAKE A CHECK while defining the obstacles(whether destination is an obstacle)
   * 
   */
  if(w->grid[w->dx][w->dy]==-1 || w->grid[w->sx][w->sy]==-1) return 1;
  w->grid[w->dx][w->dy]=1;  // Needed for the algos to work correctly
	
 for(i=0;i<w->nbrows;i++)
		for(j=0;j<w->nbcolumns;j++)
			if(w->grid[i][j]==-1)
				w->distance[i][j]=-1;  //the obstacles cannot be reached	
  
//////////////////////////////////////////////////

  /////test 3///////////////////////////
  /*  	w->grid[1][1]=-1;
    w->grid[1][2]=-1;
    w->grid[1][3]=-1;
    w->grid[2][1]=-1;
    w->grid[2][3]=-1;
    w->grid[3][1]=-1;
    w->grid[3][2]=-1;
    w->grid[3][3]=-1;
    
	w->grid[w->dx][w->dy]=1;  // Needed for the algo to work correctly
	for(i=0;i<w->nbrows;i++)
		for(j=0;j<w->nbcolumns;j++)
			if(w->grid[i][j]==-1)
				w->distance[i][j]=-1;  //the obstacles cannot be reached	
  */
//////////////////////////////////////////////////
 
	disp_grid(w);
  return 0;
  //disp_distance(w);
	

}

void disp_grid (workspace *w) {

  printf("The source is (%d,%d) \n",w->sx,w->sy);
  printf("The destination is (%d,%d) \n",w->dx,w->dy);
	int i, j;
	printf("\nDisplaying the workspace:\n");
	for (i = 0; i < w->nbrows; i++) {
		for (j = 0; j < w->nbcolumns; j++) {
			if(w->grid[i][j]!=-1)
				printf(" %d\t", w->grid[i][j]);
			else printf("%d\t",w->grid[i][j]);
		}
	printf("\n");
	}
	printf("\n");
}

void disp_distance(workspace *w){
  int i,j;
printf("\n-----------------------------------------");
	printf("\nDisplaying the distance matrix:\n");
	for (i = 0; i < w->nbrows; i++) {
		for (j = 0; j < w->nbcolumns; j++){
			if(w->distance[i][j]!=-1)
				printf(" %d\t",w->distance[i][j]);
			else printf("%d\t", w->distance[i][j]);
		}
			
		printf("\n");
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
	
  
  /////////test 1///////////
  /*
	w->nbrows  = 3; w->nbcolumns  = 3;
	w->sx = 0; w->sy = 1;
	w->dx = 2; w->dy = 2;
  */
  //////////////////////////
  
  
  /////////test 2///////////

	w->nbrows  = 5; w->nbcolumns  = 5;
	w->sx = 0; w->sy = 1;
	w->dx = 2; w->dy = 2;
 
  //////////////////////////
  

}


void reset_grid(workspace* w){
	int i,j;
	for(i=0;i<nbrows;i++)
		for(j=0;j<nbcolumns;j++)
			if(w->grid[i][j]==1) w->grid[i][j]=0;
		


}

int test(int i, int j){

  if(i>=0 && i<nbrows && j>=0 && j<nbcolumns) return 1;
  return 0;

}

int nb_possible_ngbrs(int i, int j, workspace* w){

  int tmp=0;
  if(test(i-1,j)==1 && w->grid[i-1][j]==0) tmp++;
  if(test(i,j-1)==1 && w->grid[i][j-1]==0) tmp++;
  if(test(i+1,j)==1 && w->grid[i+1][j]==0) tmp++;
  if(test(i,j+1)==1 && w->grid[i][j+1]==0) tmp++;
  return tmp;

}

void display_path(workspace* w){
   int i=0;
   printf("Displaying the path found from source to destination \n");
   printf("Starting from -> ");
   while(w->pathx[i]!=-1){
     printf("(%d, %d, [%d]) -> ",w->pathx[i],w->pathy[i],
	w->distance[w->pathx[i]][w->pathy[i]]);
     i++;
   }
   printf("Goal Reached !!\n");
 
}


void push(int* queue, int a) {
    int i;
  for(i=0;i< nbrows * nbcolumns;i++){
    if(queue[i]==-1) {
      queue[i]=a;
      return;
    }
  }


}

int pull(int* queue){

  int tmp = queue[0];
  int i;
  for(i=0;i< nbrows* nbcolumns-1;i++){
    queue[i]=queue[i+1];
  }
  queue[nbrows * nbcolumns-1]=-1;
  return tmp;


} 

int queueIsEmpty(int* queue) {
  if(queue[0]==-1) return 1;
  return 0;
  

}

void print_queue(int* queuex, int* queuey) {
  int i;
  printf("the queue is  ");
  for(i=0;i< nbrows* nbcolumns;i++)
    printf("(%d,%d)",queuex[i],queuey[i]);
  printf("\n");

}

