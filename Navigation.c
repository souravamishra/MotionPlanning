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
  
  //int* parentx=malloc(nbrows*sizeof(int)); //the abscissa of parent
  //int* parenty=malloc(nbcolumns*sizeof(int)); //the ordinate of parent
  
  //parentx[goal_x]=goal_x;  //Goal is the parent of itself
  //parenty[goal_y]=goal_y;
  

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
      //parentx[ngbrx[t]]=i;  // parent of ngbr is (i,j)
      //parentx[ngbry[t]]=j;
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


//Manhattan Distance
int heuristic_cost_estimate(int startx, int starty, int goalx, int goaly){
 
  return abs(goaly-starty)+abs(goalx-startx);

}

void construct_path(int** came_fromx, int** came_fromy,int goalx, int goaly, int startx, int starty){
  
  int a;
  int b;
  printf("(%d,%d) -> ",goalx,goaly);
  while(!(a==startx && b==starty)){
    a=came_fromx[goalx][goaly];
    b=came_fromy[goalx][goaly];
    if(a==startx && b==starty)
      printf("(%d,%d)",a,b);
    else printf("(%d,%d) -> ",a,b);
    goalx=a;
    goaly=b;
  }
  printf("\n");

}


int min_fscore_openlist(int** fscore, int* openlistx, int* openlisty){
  int min=80000,i=0;
  int min_index, a,b;
  while(openlistx[i]!=-1){
    a=openlistx[i];
    b=openlistx[i];
    if(fscore[a][b]<min){
      min=fscore[a][b];
      min_index=i;
    }
    i++;
  }
  return min_index;

}

void remove_min(int* list, int min_index ){
  list[min_index]=-1;
  int i=min_index;
  do{
    list[i]=list[i+1];
    i++;
  }while(list[i]!=-1);
}

int* abscissa_ngbrs(int i, int j, int**workspace){
  int nbrs = nb_possible_ngbrs(i,j,workspace); // nb of ngbrs of (i,j)
  int *ngbrx = malloc(nbrs*sizeof(int)); // abscissae of ngbrs
    
  int index=0;    
  if(test(i-1,j)==1 && workspace[i-1][j]==0) { //check for valid and free cell in the workspace
    ngbrx[index]=i-1;
    //ngbry[index]=j;
    index++;
  }
  if(test(i,j-1)==1 && workspace[i][j-1]==0){
    ngbrx[index]=i;
    //ngbry[index]=j-1;
    index++;
    // printf(" In case2\n");
  } 
  if(test(i+1,j)==1 && workspace[i+1][j]==0) {
    ngbrx[index]=i+1;
    //ngbry[index]=j;
    index++;
    //printf(" In case3\n");
  }
  if(test(i,j+1)==1 && workspace[i][j+1]==0) {
    ngbrx[index]=i;
    //ngbry[index]=j+1;
    index++;
    //printf(" In case4\n");
  }
  return ngbrx;
  
}

int* ordinate_ngbrs(int i, int j, int**workspace){
  
  int nbrs = nb_possible_ngbrs(i,j,workspace); // nb of ngbrs of (i,j)
  int *ngbry = malloc(nbrs*sizeof(int)); // ordinates of ngbrs
  int index=0;

  if(test(i-1,j)==1 && workspace[i-1][j]==0) { //check for valid and free cell in the workspace
    //ngbrx[index]=i-1;
    ngbry[index]=j;
    index++;
  }
  if(test(i,j-1)==1 && workspace[i][j-1]==0){
    //ngbrx[index]=i;
    ngbry[index]=j-1;
    index++;
    // printf(" In case2\n");
  } 
  if(test(i+1,j)==1 && workspace[i+1][j]==0) {
    //ngbrx[index]=i+1;
    ngbry[index]=j;
    index++;
    //printf(" In case3\n");
  }
  if(test(i,j+1)==1 && workspace[i][j+1]==0) {
    //ngbrx[index]=i;
    ngbry[index]=j+1;
    index++;
    //printf(" In case4\n");
  }
  return ngbry;
}

int in_list(int a, int b, int* closedlistx, int* closedlisty){
  int i=0;
  while(closedlistx[i]!=-1){
    if(closedlistx[i]==a && closedlisty[i]==b){
      return 1;
    }
    i++;
    
  }
  return 0;
  
}



void A_star(int** workspace, int** dist_from_goal, int startx, int starty, int goalx, int goaly){
  
  int* openlistx=malloc(nbrows*nbcolumns*sizeof(int)); //the queue(FIFO) for abscissa
  int* openlisty=malloc(nbrows*nbcolumns*sizeof(int)); //the queue for ordinate
  
  int* closedlistx=malloc(nbrows*nbcolumns*sizeof(int)); //the queue(FIFO) for abscissa
  int* closedlisty=malloc(nbrows*nbcolumns*sizeof(int)); //the queue for ordinate
  
  int** came_fromx;
  came_fromx=malloc(nbrows*sizeof(int*)); //the abscissa of parent
  int** came_fromy;
  came_fromy=malloc(nbrows*sizeof(int*)); //the ordinate of parent
 
  int** fscore;
  int i,j;
  fscore=malloc(sizeof(int*) * nbrows); //the fscore matrix
  for(i=0;i<nbrows;i++){
    fscore[i]=malloc(sizeof(int) * nbcolumns);
    came_fromx[i]=malloc(sizeof(int) * nbcolumns);
    came_fromy[i]=malloc(sizeof(int) * nbcolumns);
  }
 
  int** gscore;
 
  gscore=malloc(nbrows*sizeof(int*)); //the gscore matrix
  for(i=0;i<nbrows;i++){
    gscore[i]=malloc(nbcolumns*sizeof(int));
  }
 

  for(i=0;i<nbrows*nbcolumns;i++){ //the queues are empty
    openlistx[i]=-1;
    openlisty[i]=-1;
    closedlistx[i]=-1;
    closedlisty[i]=-1;
    //came_fromy[i]=-1;
    //came_fromx[i]=-1;
  }
  
  for(i=0;i<nbrows;i++){
    for(j=0;j<nbcolumns;j++){
      came_fromx[i][j]=-1;
      came_fromy[i][j]=-1;
    }
  }
  
  
  push(openlistx,startx); //initialy openlist contains the start cell
  push(openlisty,starty);
  
  gscore[startx][starty]=0;
  fscore[startx][starty]=heuristic_cost_estimate(startx, starty, goalx, goaly);
  
    
  int currentx;
  int currenty;
  //int index=0;
  
  while(queueIsEmpty(openlistx)==0){
    //printf("Open list is : \n");
    //print_queue(openlistx, openlisty);

    //printf("Closed list is :\n");
    //print_queue(closedlistx, closedlisty);
    
    /* printf("Camefromx list is : \n");
    for(i=0;i<nbrows*nbcolumns;i++)
      printf("%d",came_fromx[i]);
    printf("\n");
    for(i=0;i<nbrows*nbcolumns;i++)
      printf("%d",came_fromy[i]);
    */
    
    //printf("\nThe gscore is : \n");
    //pretty_print(gscore);
    //printf("The fscore is : \n");
    //pretty_print(fscore);
    //print_queue(closedlistx, closedlisty);
    int index_minlist=min_fscore_openlist(fscore, openlistx, openlisty);
    currentx=openlistx[index_minlist];
    currenty=openlisty[index_minlist];
    //printf("In open and lowest in fscore (%d, %d)\n",currentx,currenty);

    if(currentx==goalx && currenty==goaly){
      printf("Printing the path now\n");
      return(construct_path(came_fromx,came_fromy,goalx,goaly, startx, starty));
    }
    
    remove_min(openlistx,index_minlist);
    remove_min(openlisty,index_minlist);
    //printf("Removing from Open list : \n");
    //print_queue(openlistx, openlisty);

    push(closedlistx,currentx);
    push(closedlisty,currenty);
    //printf("Added to Closed list : \n");
    //print_queue(closedlistx, closedlisty);

    
    int ngbrs=nb_possible_ngbrs(currentx,currenty, workspace);
    //pretty_print(workspace);
    //printf("(%d,%d)",currentx, currenty);
    int* ngbrsx=abscissa_ngbrs(currentx, currenty, workspace);
    int* ngbrsy=ordinate_ngbrs(currentx, currenty, workspace);
    
    //printf("The neighbors of current are %d \n", ngbrs);
    //for(i=0;i<ngbrs;i++)
    //printf("(%d,%d)",ngbrsx[i],ngbrsy[i]);
    
    for(i=0;i<ngbrs;i++){
      int dist_from_ngbr=1;
      int tentative_gscore=gscore[currentx][currenty]+dist_from_ngbr;
      //printf("Tentataive score here is %d \n", tentative_gscore);
      if(in_list(ngbrsx[i],ngbrsy[i],closedlistx, closedlisty)==1){
          continue;
        
      }
      if(in_list(ngbrsx[i],ngbrsy[i],openlistx, openlisty)==0 
         || tentative_gscore<gscore[ngbrsx[i]][ngbrsy[i]]){
        //printf("I'm here 2\n");
        came_fromx[ngbrsx[i]][ngbrsy[i]]=currentx;
        came_fromy[ngbrsx[i]][ngbrsy[i]]=currenty;
        //index++;
        gscore[ngbrsx[i]][ngbrsy[i]]=tentative_gscore;
        fscore[ngbrsx[i]][ngbrsy[i]]=gscore[ngbrsx[i]][ngbrsy[i]]+heuristic_cost_estimate(ngbrsx[i],ngbrsy[i], goalx, goaly);
        //printf("\nThe gscore in nbgr is : \n");
         //pretty_print(gscore);
         //printf("The fscore in ngbr is : \n");
         //pretty_print(fscore);
        
        if(in_list(ngbrsx[i],ngbrsy[i],openlistx, openlisty)==0){
          push(openlistx,ngbrsx[i]);
          push(openlisty,ngbrsy[i]);
        }
        
      }
    }
    } //while loop
    
  printf("Search failed !! \n");
  for (i = 0; i < nbrows; i++){  
    free(fscore[i]);
    free(gscore[i]);
  }
  free(fscore);
  free(gscore);
  free(openlistx);
  free(openlisty);
  free(closedlistx);
  free(closedlisty);
  
  
 
}


int main(int argc, char** argv){  
  
  nbrows = 6; // 6 nb of rows
  nbcolumns= 12; // 12 nb of columns

  int i,j;
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
  
  //workspace[1][1]=-1;
  //workspace[2][1]=-1;
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
  
  //dist_from_goal[1][1]=-1;
  //dist_from_goal[2][1]=-1;
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
  printf("Output of A* algorithm \n");
  
  //Reset workspace previously modified 
  for(i=0;i<nbrows;i++){
    for(j=0;j<nbcolumns;j++)
      if(workspace[i][j]==1) workspace[i][j]=0;
  }
  
  A_star(workspace, dist_from_goal, sourcex, sourcey, goal_x, goal_y);  
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
