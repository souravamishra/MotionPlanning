#include <math.h>
#include "Navigation.h"

//Computes the distance matrix using the numeric navigation function
void computeDistance(workspace* w) {
  int i,j; 
  int nbrs;  

	
 int* tovisitx=malloc(w->nbrows * w->nbcolumns*sizeof(int)); //the queue(FIFO) for abscissa
  int* tovisity=malloc(w->nbrows * w->nbcolumns*sizeof(int)); //the queue for ordinate
   
  for(i=0;i<w->nbrows * w->nbcolumns;i++){ //None of the nodes have been visited yet
    tovisitx[i]=-1;
    tovisity[i]=-1;
  }

  push(tovisitx,w->dx);  //Push the goal in the queue
  push(tovisity,w->dy);

  w->distance[w->dx][w->dy]=0; //goal is at distance 0 from itself
  
  while(queueIsEmpty(tovisitx)==0){
    i=pull(tovisitx);
    j=pull(tovisity); 
    nbrs = nb_possible_ngbrs(i,j,w); // nb of ngbrs of (i,j)
    
    int *ngbrx = malloc(nbrs*sizeof(int)); // abscissae of ngbrs
    int *ngbry = malloc(nbrs*sizeof(int)); // ordinates of ngbrs
    
    int index=0;    
    if(test(i-1,j)==1 && w->grid[i-1][j]==0) { //check for valid and free cell in the workspace
      ngbrx[index]=i-1;   
      ngbry[index]=j;
      index++;
    }
    if(test(i,j-1)==1 && w->grid[i][j-1]==0){
      ngbrx[index]=i;
      ngbry[index]=j-1;
      index++;
      
    } 
    if(test(i+1,j)==1 && w->grid[i+1][j]==0) {
      ngbrx[index]=i+1;
      ngbry[index]=j;
      index++;
      
    }
    if(test(i,j+1)==1 && w->grid[i][j+1]==0) {
      ngbrx[index]=i;
      ngbry[index]=j+1;
      index++;
      
    }
    int t;

    for(t=0;t<nbrs;t++){
      push(tovisitx,ngbrx[t]); //Push the daughter nodes in the queue
      push(tovisity,ngbry[t]);
      w->grid[ngbrx[t]][ngbry[t]]=1; //mark the node as visited
      w->distance[ngbrx[t]][ngbry[t]]= w->distance[i][j]+1;   // distance from root to x = distance from root to parent+1
    }
    
  }
  free(tovisitx);
  free(tovisity);
}

int* minimum_dist_ngbr(workspace *w, int i, int j){
  int *minum_ngbrs_xy=malloc(2*sizeof(int));
  int min=w->distance[i][j];
  minum_ngbrs_xy[0]=i;
  minum_ngbrs_xy[1]=j;
  
  if(test(i,j-1)==1 && w->distance[i][j-1]>=0){
    if(w->distance[i][j-1]<min){ 
      min=w->distance[i][j-1];
      minum_ngbrs_xy[0]=i;
      minum_ngbrs_xy[1]=j-1;
    }
  }
  if(test(i-1,j)==1 && w->distance[i-1][j]>=0){
    if(w->distance[i-1][j]<min){
      min=w->distance[i-1][j];
      minum_ngbrs_xy[0]=i-1;
      minum_ngbrs_xy[1]=j;
    }
  } 
  if(test(i+1,j)==1 && w->distance[i+1][j]>=0) {
    if(w->distance[i+1][j]<min){
      min=w->distance[i+1][j];
      minum_ngbrs_xy[0]=i+1;
      minum_ngbrs_xy[1]=j;
    }
  }
  if(test(i,j+1)==1 && w->distance[i][j+1]>=0) {
    if(w->distance[i][j+1]<min){
      min=w->distance[i][j+1];
      minum_ngbrs_xy[0]=i;
      minum_ngbrs_xy[1]=j+1;
    }
  }
  return minum_ngbrs_xy;

  
}
int path_finder(workspace * w){
  
 
  if(nb_possible_ngbrs(w->dx,w->dy,w)==0) return 1; //path could not be found
 
  int tmp=0,startx,starty;
  startx = w->sx;
  starty = w->sy;

  w->pathx[0]=startx;
  w->pathy[0]=starty;
  
  while(!(startx==w->dx && starty==w->dy)){
    int* closest_ngbr_to_goal=minimum_dist_ngbr(w, startx,starty);
    tmp++;
    
    w->pathx[tmp]=closest_ngbr_to_goal[0];
    w->pathy[tmp]=closest_ngbr_to_goal[1];
   
    startx=closest_ngbr_to_goal[0];
    starty=closest_ngbr_to_goal[1];
  }
  return 0;

}


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

int* abscissa_ngbrs(int i, int j, workspace* w){
  int nbrs = nb_possible_ngbrs(i,j,w); // nb of ngbrs of (i,j)
  int *ngbrx = malloc(nbrs*sizeof(int)); // abscissae of ngbrs
    
  int index=0;    
  if(test(i-1,j)==1 && w->grid[i-1][j]==0) { //check for valid and free cell in the workspace
    ngbrx[index]=i-1;
    index++;
  }
  if(test(i,j-1)==1 && w->grid[i][j-1]==0){
    ngbrx[index]=i;
    index++;
  } 
  if(test(i+1,j)==1 && w->grid[i+1][j]==0) {
    ngbrx[index]=i+1;
    index++;
  }
  if(test(i,j+1)==1 && w->grid[i][j+1]==0) {
    ngbrx[index]=i;
    index++;
  }
  return ngbrx;
  
}

int* ordinate_ngbrs(int i, int j, workspace* w){
  
  int nbrs = nb_possible_ngbrs(i,j,w); // nb of ngbrs of (i,j)
  int *ngbry = malloc(nbrs*sizeof(int)); // ordinates of ngbrs
  int index=0;

  if(test(i-1,j)==1 && w->grid[i-1][j]==0) { //check for valid and free cell in the workspace
    ngbry[index]=j;
    index++;
  }
  if(test(i,j-1)==1 && w->grid[i][j-1]==0){
    ngbry[index]=j-1;
    index++;
  } 
  if(test(i+1,j)==1 && w->grid[i+1][j]==0) {
    ngbry[index]=j;
    index++;
  }
  if(test(i,j+1)==1 && w->grid[i][j+1]==0) {
    ngbry[index]=j+1;
    index++;
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



void A_star(workspace* w){
  
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
  }
  
  for(i=0;i<nbrows;i++){
    for(j=0;j<nbcolumns;j++){
      came_fromx[i][j]=-1;
      came_fromy[i][j]=-1;
    }
  }
  
  
  push(openlistx,w->sx); //initialy openlist contains the start cell
  push(openlisty,w->sy);
  
  gscore[w->sx][w->sy]=0;
  fscore[w->sx][w->sy]=heuristic_cost_estimate(w->sx, w->sy, w->dx, w->dy);
  
    
  int currentx;
  int currenty;
 
  while(queueIsEmpty(openlistx)==0){
    
    int index_minlist=min_fscore_openlist(fscore, openlistx, openlisty);
    currentx=openlistx[index_minlist];
    currenty=openlisty[index_minlist];
    

    if(currentx==w->dx && currenty==w->dy){
      printf("Printing the path now in reveser order\n");
      return(construct_path(came_fromx,came_fromy,w->dx,w->dy, w->sx, w->sy));
    }
    
    remove_min(openlistx,index_minlist);
    remove_min(openlisty,index_minlist);
    

    push(closedlistx,currentx);
    push(closedlisty,currenty);
    

    
    int ngbrs=nb_possible_ngbrs(currentx,currenty, w);
    
    int* ngbrsx=abscissa_ngbrs(currentx, currenty, w);
    int* ngbrsy=ordinate_ngbrs(currentx, currenty, w);
    
    
    for(i=0;i<ngbrs;i++){
      int dist_from_ngbr=1;
      int tentative_gscore=gscore[currentx][currenty]+dist_from_ngbr;
      if(in_list(ngbrsx[i],ngbrsy[i],closedlistx, closedlisty)==1){
          continue;
        
      }
      if(in_list(ngbrsx[i],ngbrsy[i],openlistx, openlisty)==0 
         || tentative_gscore<gscore[ngbrsx[i]][ngbrsy[i]]){
        came_fromx[ngbrsx[i]][ngbrsy[i]]=currentx;
        came_fromy[ngbrsx[i]][ngbrsy[i]]=currenty;
        gscore[ngbrsx[i]][ngbrsy[i]]=tentative_gscore;
        fscore[ngbrsx[i]][ngbrsy[i]]=gscore[ngbrsx[i]][ngbrsy[i]]+heuristic_cost_estimate(ngbrsx[i],ngbrsy[i], w->dx, w->dy);
        
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

