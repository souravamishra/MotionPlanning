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
  int tmp = queue[0];
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


//Computes the distance matrix using the numeric navigation function
void computeDistance(int** workspace, int** dist_from_goal, int goal_x, int goal_y) {
  int i,j;
  //int counter=1;  
  int* tovisitx=malloc(nbrows*nbcolumns*sizeof(int)); //the queue(FIFO) for abscissa
  int* tovisity=malloc(nbrows*nbcolumns*sizeof(int)); //the queue for ordinate
  
  int* parentx=malloc(nbrows*sizeof(int)); //the abscissa of parent
  int* parenty=malloc(nbcolumns*sizeof(int)); //the ordinate of parent
  
  parentx[goal_x]=goal_x;  //Goal is the parent of itself
  parenty[goal_y]=goal_y;
  

  for(i=0;i<nbrows*nbcolumns;i++){ //None of the nodes have been visited yet
    tovisitx[i]=-1;
    tovisity[i]=-1;
  }
  push(tovisitx,goal_x);  //Push the goal in the queue
  push(tovisity,goal_y);
  dist_from_goal[goal_x][goal_y]=0; //goal is at distance 0 from itself
  
  while(queueIsEmpty(tovisitx)==0){
    i=pull(tovisitx);
    j=pull(tovisity); 
    int nbrs = nb_possible_ngbrs(i,j,workspace); // nb of ngbrs of (i,j)
    int *ngbrx = malloc(nbrs*sizeof(int)); // abscissae of ngbrs
    int *ngbry = malloc(nbrs*sizeof(int)); // ordinates of ngbrs
    int index=0;    
    if(test(i-1,j)==1 && workspace[i-1][j]==0) { //check for valid and free cell in the workspace
      ngbrx[index]=i-1;
      ngbry[index]=j;
      index++;
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
      push(tovisitx,ngbrx[t]); //Push the daughter notes in the queue
      push(tovisity,ngbry[t]);
      workspace[ngbrx[t]][ngbry[t]]=1; //this node is visited
      dist_from_goal[ngbrx[t]][ngbry[t]]=
        dist_from_goal[i][j]+1;   // distance from root to x = distance from root to parent+1
      parentx[ngbrx[t]]=i;  // parent of ngbr is (i,j)
      parentx[ngbry[t]]=j;
    }
    //pretty_print(depth_from_root);
    }
  free(tovisitx);
  free(tovisity);
}

int* minimum_dist_ngbr(int** distance_from_goal, int i, int j){
  int *minum_ngbrs_xy=malloc(2*sizeof(int));
  int min=distance_from_goal[i][j];
  minum_ngbrs_xy[0]=i;
  minum_ngbrs_xy[1]=j;
  
  if(test(i,j-1)==1 && distance_from_goal[i][j-1]>=0){
     if(distance_from_goal[i][j-1]<min){ 
       min=distance_from_goal[i][j-1];
       minum_ngbrs_xy[0]=i;
       minum_ngbrs_xy[1]=j-1;
     }
  }
  if(test(i-1,j)==1 && distance_from_goal[i-1][j]>=0){
      if(distance_from_goal[i-1][j]<min){
        min=distance_from_goal[i-1][j];
        minum_ngbrs_xy[0]=i-1;
        minum_ngbrs_xy[1]=j;
      }
    } 
    if(test(i+1,j)==1 && distance_from_goal[i+1][j]>=0) {
      if(distance_from_goal[i+1][j]<min){
        min=distance_from_goal[i+1][j];
        minum_ngbrs_xy[0]=i+1;
        minum_ngbrs_xy[1]=j;
      }
    }
    if(test(i,j+1)==1 && distance_from_goal[i][j+1]>=0) {
      if(distance_from_goal[i][j+1]<min){
        min=distance_from_goal[i][j+1];
        minum_ngbrs_xy[0]=i;
        minum_ngbrs_xy[1]=j+1;
      }
      }
    return minum_ngbrs_xy;

  
}
void path_finder(int** distance_from_goal, int startx, int starty, int goalx,int goaly, int* pathx, int* pathy ){
   
  int tmp=0;
  pathx[0]=startx;
  pathy[0]=starty;
  while(!(startx==goalx && starty==goaly)){
    int* closest_ngbr_to_goal=minimum_dist_ngbr(distance_from_goal,startx,starty);
    tmp++;
    
    //printf("Check 1 : Closest neighbour of (%d,%d) is (%d,%d)\n",startx,starty,closest_ngbr_to_goal[0],closest_ngbr_to_goal[1]);

    pathx[tmp]=closest_ngbr_to_goal[0];
    pathy[tmp]=closest_ngbr_to_goal[1];
   
    startx=closest_ngbr_to_goal[0];
    starty=closest_ngbr_to_goal[1];
  }
   

}

void display_path(int*pathx, int* pathy, int** dist_from_goal){
  int i=0;
  printf("Starting from -> ");
  while(pathx[i]!=-1){
    printf("(%d, %d, [%d]) -> ",pathx[i],pathy[i],dist_from_goal[pathx[i]][pathy[i]]);
    i++;
  }
  printf("Goal Reached !!\n");

}


int main(int argc, char** argv){  
  
  nbrows = 6; // 6 nb of rows
  nbcolumns= 12; // 12 nb of columns

  int i;
  int goal_x = 4; // 4 abscissa of goal 
  int goal_y = 9; //9 ordinate of goal
  
  int sourcex=0; //source abscissa
  int sourcey=2; //source ordinate
   
  int** workspace; //matrix to create the workspace
  workspace = malloc(sizeof(int*) * nbrows);
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
  
  int* pathx=malloc(nbrows*nbcolumns*sizeof(int));//the queue(FIFO) for abscissa
  int* pathy=malloc(nbrows*nbcolumns*sizeof(int));//the queue for ordinate
  for(i=0;i<nbrows*nbcolumns;i++){ //Path initialization
    pathx[i]=-1;
    pathy[i]=-1;
  }
  
  printf("\n");
  for(i=0;i<nbcolumns-1;i++)
  printf("xxxxxxxxx");
  printf("\n\n");
  printf("here is the distance matrix\n");
  pretty_print(dist_from_goal);

  printf("Here is a path from source to destination\n");
  
  //minimum_dist_ngbr(dist_from_goal,sourcex, sourcey);
  path_finder(dist_from_goal,sourcex, sourcey, goal_x, goal_y,pathx,pathy);
  display_path(pathx,pathy, dist_from_goal);
  for (i = 0; i < nbrows; i++){  
    free(workspace[i]);
    free(dist_from_goal[i]);
  }
    free(workspace);
    free(dist_from_goal);
    free(pathx);
    free(pathy);
  return 0;
}
