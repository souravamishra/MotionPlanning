// For printing purpose
void pretty_print(int** matrix)

//For segmentation error avoidance in  workspace[i,j]
int test(int i , int j)

//Number of neighbours of the cell [i,j]
int nb_possible_ngbrs(int i,int j, int** workspace)

//push a in the queue
void push(int* queue, int a)

//pull an element from the queue
int pull(int* queue)

//check if the queue is empty
int queueIsEmpty(int* queue)

//print the queue
void print_queue(int* queuex, int* queuey)

//Computes the distance matrix using the numeric navigation function
void computeDistance(int** workspace, int** dist_from_goal, int goal_x, int goal_y)

int* minimum_dist_ngbr(int** distance_from_goal, int i, int j)

void path_finder(int** distance_from_goal, int startx, int starty, int goalx,int goaly, int* pathx, int* pathy )

void display_path(int*pathx, int* pathy, int** dist_from_goal)

//Manhattan Distance
int heuristic_cost_estimate(int startx, int starty, int goalx, int goaly)

void construct_path(int** came_fromx, int** came_fromy,int goalx, int goaly, int startx, int starty)


int min_fscore_openlist(int** fscore, int* openlistx, int* openlisty)

void remove_min(int* list, int min_index )

int* abscissa_ngbrs(int i, int j, int**workspace)

int* ordinate_ngbrs(int i, int j, int**workspace)

int in_list(int a, int b, int* closedlistx, int* closedlisty)

void A_star(int** workspace, int** dist_from_goal, int startx, int starty, int goalx, int goaly)

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
