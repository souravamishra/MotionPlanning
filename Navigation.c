# include "stdio.h"
# include "math.h"
# include "stdlib.h"
# include <time.h>

int nbrows;
int nbcolumns;

// For printing purpose
void pretty_print(int** matrix){
  int x,y;
  printf("\n");
  for(x = 0; x < nbrows; x++) {
    for(y = 0; y < nbcolumns; y++) {
      if(matrix[x][y]!=-1)
      printf(" %d\t", matrix[x][y]);
      else printf("%d\t", matrix[x][y]);
    }
    printf("\n");
  }
}


//For segmentation error avoidance in  workspace[i,j]
int test(int i , int j){

  if(i>=0 && i<nbrows && j>=0 && j<nbcolumns) return 1;
  return 0;
}


//Number of neighbours of the cell [i,j]
int nb_possible_ngbrs(int i,int j, int** workspace){
  int tmp=0;
  if(test(i-1,j)==1 && workspace[i-1][j]==0) tmp++;
  if(test(i,j-1)==1 && workspace[i][j-1]==0) tmp++;
  if(test(i+1,j)==1 && workspace[i+1][j]==0) tmp++;
  if(test(i,j+1)==1 && workspace[i][j+1]==0) tmp++;
  return tmp;
}

//push a in the queue
void push(int* queue, int a){
  int i;
    for(i=0;i<nbrows*nbcolumns;i++){
    if(queue[i]==-1) {
      //printf("in %d\n",i);
      queue[i]=a;
      return;
    }
  }
  
}

//pull an element from the queue
int pull(int* queue){
  int tmp = tab[0];
  int i;
  for(i=0;i<nbrows*nbcolumns-1;i++){
    queue[i]=queue[i+1];
  }
  queue[nbrows*nbcolumns-1]=-1;
  return tmp;
}


//check if the queue is empty
int queueIsEmpty(int* queue){
  if(queue[0]==-1) return 1;
  return 0;
  
}

//print the queue
void print_queue(int* queuex, int* queuey){
  int i;
  printf("the queue is  ");
  for(i=0;i<nbrows*nbcolumns;i++)
    printf("(%d,%d)",queuex[i],queuey[i]);
  printf("\n");
}


void computeDistance(int** workspace, int** dist_from_goal, int goal_x, int goal_y) {
  int i,j;
  depth=1;
  //int counter=1;  
  int* tovisitx=malloc(nbrows*nbcolumns*sizeof(int)); //the queue(FIFO) for x
  int* tovisity=malloc(nbrows*nbcolumns*sizeof(int)); //the queue for y
  
  int* parentx=malloc(nbrows*sizeof(int)); //the abscissa of parent
  int* parenty=malloc(nbcolumns*sizeof(int)); //the ordinate of parent
  
  parentx[goal_x]=goal_x;
  parenty[goal_y]=goal_y;
  

  for(i=0;i<nbrows*nbcolumns;i++){
    tovisitx[i]=-1;
    tovisity[i]=-1;
  }
  //workspace[goal_x][goal_y]=dist;
  push(tovisitx,goal_x);
  push(tovisity,goal_y);
  dist_from_goal[goal_x][goal_y]=0; //root is at distance 0
  
  while(queueIsEmpty(tovisitx)==0){
    //print_queue(tovisitx,tovisity);
    //array_print(depth_from_root);
    i=pull(tovisitx);
    j=pull(tovisity); 
    //workspace[i][j]=pull_depth(depth_from_root);
    int nbrs = nb_possible_ngbrs(i,j,workspace); // nb of ngbrs of (i,j)
    int *ngbrx = malloc(nbrs*sizeof(int)); // abcisses of ngbrs
    int *ngbry = malloc(nbrs*sizeof(int)); // ordinates of ngbrs
    int index=0;    
    if(test(i-1,j)==1 && workspace[i-1][j]==0) {
      ngbrx[index]=i-1;
      ngbry[index]=j;
      index++;
      //printf(" In case1 \n");
    }
    if(test(i,j-1)==1 && workspace[i][j-1]==0){
      ngbrx[index]=i;
      ngbry[index]=j-1;
      index++;
      // printf(" In case2\n");
    } 
    if(test(i+1,j)==1 && workspace[i+1][j]==0) {
      ngbrx[index]=i+1;
      ngbry[index]=j;
      index++;
      //printf(" In case3\n");
    }
    if(test(i,j+1)==1 && workspace[i][j+1]==0) {
      ngbrx[index]=i;
      ngbry[index]=j+1;
      index++;
      //printf(" In case4\n");
    }
    int t;
    for(t=0;t<nbrs;t++){
      push(tovisitx,ngbrx[t]);
      push(tovisity,ngbry[t]);
      workspace[ngbrx[t]][ngbry[t]]=1; //this node i visited
      dist_from_goal[ngbrx[t]][ngbry[t]]=
        dist_from_goal[i][j]+1;
      parentx[ngbrx[t]]=i;
      parentx[ngbry[t]]=j;
    }
    //pretty_print(depth_from_root);
    }
  free(tovisitx);
  free(tovisity);
}   


int main(int argc, char** argv){  
  
  nbrows = 6; // nb of rows
  nbcolumns = 12;  // nb of columns

  int i;
  int goal_x = 4; // abscissa of goal 
  int goal_y = 9; // ordinate of goal
  
  int **workspace; //matrix to create the workspace
  workspace =  malloc(sizeof(int*) * nbrows);
  for(i = 0; i < nbrows; i++) 
    workspace[i] = malloc(sizeof(int) * nbcolumns);
  
  workspace[2][3]=-1; //obstacles in the workspace denoted by -1
  workspace[2][4]=-1;
  workspace[2][5]=-1;
  workspace[2][6]=-1;
  workspace[3][3]=-1;
  //workspace[3][4]=-1;
  workspace[3][5]=-1;
  workspace[3][6]=-1;
  workspace[4][3]=-1; 
  workspace[4][4]=-1;
  workspace[4][5]=-1;
  workspace[4][6]=-1;
  

  workspace[goal_x][goal_y]=1; // Goal in the workspace denoted by 1
  
  int** dist_from_goal;   // matrix to store the distance of each
                           // cell from the goal cell 
  dist_from_goal = malloc(nbrows*sizeof(int*));
  for(i = 0; i < nbrows; i++) 
    dist_from_goal[i] = malloc(sizeof(int) * nbcolumns);
  
  dist_from_goal[2][3]=-1; // distance to obstacles
  dist_from_goal[2][4]=-1;
  dist_from_goal[2][5]=-1;
  dist_from_goal[2][6]=-1;
  dist_from_goal[3][3]=-1;
  //dist_from_goal[3][4]=-1;
  dist_from_goal[3][5]=-1;
  dist_from_goal[3][6]=-1;
  dist_from_goal[4][3]=-1; 
  dist_from_goal[4][4]=-1;
  dist_from_goal[4][5]=-1;
  dist_from_goal[4][6]=-1;
 

  printf("Here is the workspace \n");
  pretty_print(workspace);
  
  computeDistance(workspace,dist_from_goal,goal_x,goal_y);
  printf("\n");
  for(i=0;i<nbcolumns-1;i++)
  printf("xxxxxxxxx");
  printf("\n\n");
  printf("here is the distance matrix\n");
  pretty_print(dist_from_goal);
  for (i = 0; i < nbrows; i++){  
    free(workspace[i]);
    free(dist_from_goal[i]);
  }
    free(workspace);
    free(dist_from_goal);
  return 0;
}
