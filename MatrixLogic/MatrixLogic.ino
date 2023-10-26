
//In the below matrix, every cell with a path number is in a (2x+1,2y+1) position compared 
//to the real maze where (x,y) is its position. 
//The path number is the distance to the exit. Cells that aren't junctions [e.g straight lines]
//are marked with a -1 as they won't be recognized by the robot. When the robot gets
//the command to go home the idea is that it goes to the next junction and from there 
//examines the path distance in the adjacent cells. It then chooses the one with the 
//lowest number and calculates what turning action it must take to go there. 

//This depends on:
//*The robots current orientation.
//*The relative position of the current cell and the next cell.
// For example:
/*
 * Robot at (3,3), robo direction == 3 (West) -> Wall to north, paths to east, south and west
 * Check Path Distance Numbers in the path cells. 6, 4 and 6. 
 * Next cell is to south -> path direction == 2. [North == 0, East == 1, South == 2, West = 3]
 * Turn commands: 
 * In any junction: Turn180 -> In case the way to the back is quicker. 
 *    - In right or left turns -> just follow line otherwise
 *    - In T, or X junctions -> Calculate action
 *    
 * Calculate turn command:
 * turnAction = (4+pathDir-robotDir)%4;
 *
 */
int maze[15][15] = {
    {-1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1},
    {0, 9, 1, 10, 0, 11, 1, 10, 1, 11, 0, 12, 1, 11, 0},
    {-1, 1, -1, 1, -1, 1, -1, 1, -1, 0, -1, 0, -1, 1, -1},
    {0, -1, 0, 11, 1, 12, 0, -1, 0, 8, 1, -1, 1, 10, 0},
    {-1, 1, -1, 0, -1, 0, -1, 1, -1, 1, -1, 0, -1, 0, -1},
    {0, -1, 0, 6, 1, 7, 0, 8, 1, 7, 0, 16, 1, 17, 0},
    {-1, 1, -1, 1, -1, 1, -1, 0, -1, 1, -1, 1, -1, 0, -1},
    {0, -1, 0, 5, 0, 6, 1, 5, 1, 6, 0, 15, 1, 14, 0},
    {-1, 1, -1, 1, -1, 0, -1, 1, -1, 0, -1, 0, -1, 1, -1},
    {0, -1, 0, -1, 0, 5, 1, 4, 1, -1, 1, 6, 0, 13, 0},
    {-1, 1, -1, 1, -1, 0, -1, 1, -1, 0, -1, 1, -1, 1, -1},
    {0, 4, 1, 3, 1, 2, 1, 3, 0, 8, 1, 7, 0, 12, 0},
    {-1, 0, -1, 1, -1, 1, -1, 0, -1, 1, -1, 0, -1, 1, -1},
    {0, 5, 1, 4, 0, 1, 1, 0, 0, 9, 1, -1, 1, 11, 0},
    {-1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1},
};
int rDir = 2; // robot's direction0 = north, 1 = east etc.
int thisX = 2, thisY = 1; //The current pos of the robot
int matX = 2*thisX+1; //The corresponding position in the matrix
int matY = 2*thisY+1; //The corresponding position in the matrix
int pathDir; // direction of the path the robot should move in
int turnAction; // the resulting action from the command calculation, 
                // 0 = goStraight(), 1 = turnRight(), 2 = turn180(), 3 = turnLeft()
int pd_xN, pd_yN, pd_xE, pd_yE, pd_xS, pd_yS, pd_xW, pd_yW; // The path direction number of the cell in the      
int pd_min; //The minimum path distance value of a connected cell
int pd_xDir, pd_yDir;
 
int getNextPD(int matX, int matY, int dir){ //Gets the next path distance number (that isn't a -1) in a direction 
//The path distance in the cell to the respective direction
int pd_north, pd_east, pd_south, pd_west; //If these values are left unchanged below (because there is a wall in that direction) //we should still be able to compare them, so they need a big value (cause the smallest values is searched for). 
  
                                                            
    switch(dir){
      case 0: //Search to north (negative y-dir)
        if(maze[matY-2][matX] != -1){
          pd_north = maze[matY-2][matX];
          pd_xN = matX;
          pd_yN = matY-2;
          return pd_north;
        }else{
          return getNextPD(matX,matY-2,0);
        }
        break;
      case 1: //Search to east (positive x-dir)
        if(maze[matY][matX+2] != -1){
          pd_east = maze[matY][matX+2];
          pd_xE = matX+2;
          pd_yE = matY;
          return pd_east;
        }else{
          return getNextPD(matY,matX+2,1);
        }
          break;
      case 2: //Search to south (positive y-dir)
        if(maze[matY+2][matX] != -1){
          pd_south = maze[matY+2][matX];
          pd_xS = matX;
          pd_yS = matY+2;
          return pd_south;
        }else{
          return getNextPD(matX,matY+2,2);
        }
          break;
      case 3: //Search to west (negative x-dir)
        if(maze[matY][matX-2] != -1){
          pd_west = maze[matY][matX-2];
          pd_xW = matX-2;
          pd_yW = matY;
          return pd_west;
        }else{
          return getNextPD(matX-2,matY,3);
        }
          break;
    }
}

int comparePD(int pd_north, int pd_east, int pd_south, int pd_west){ //Compares the different path distance numbers and returns the direction of the one with the lowest one
    pd_min = 99;
    int dir;
    
    if(pd_north < pd_min){
      pd_min = pd_north;
      dir = 0;
      pd_xDir = pd_xN;
      pd_yDir = pd_yN;
    }
    if(pd_east < pd_min){
      pd_min = pd_east;
      dir = 1;
      pd_xDir = pd_xE;
      pd_yDir = pd_yE;
    }
    if(pd_south < pd_min){
      pd_min = pd_south;
      dir = 2;
      pd_xDir = pd_xS;
      pd_yDir = pd_yS;
    }
    if(pd_west < pd_min){
      pd_min = pd_west;
      dir = 3;
      pd_xDir = pd_xW;
      pd_yDir = pd_yW;
    }
    return dir;
}

int inv(int c){ //From matrix coord system to x,y
    return (c-1)/2;
}
void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
}

void loop() {
  int pd_north = 99, pd_east = 99, pd_south = 99, pd_west = 99; //The path distance in the cell to the respective direction
    if(maze[matY-1][matX]){ //Wall or path to North (negative y-dir)
        pd_north = getNextPD(matX,matY,0); //Finds the next PD number and its location
    }
    if(maze[matY][matX+1]){ //Wall or path to East (positive x-dir)
        pd_east = getNextPD(matX,matY,1); //Finds the next PD number and its location
    }
    if(maze[matY+1][matX]){ //Wall or path to South (positive y-dir)
        pd_south = getNextPD(matX,matY,2); //Finds the next PD number and its location
    }
    if(maze[matY][matX-1]){ //Wall or path to East (negative x-dir)
        pd_west = getNextPD(matX,matY,3); //Finds the next PD number and its location
    }

    
    pathDir = comparePD(pd_north,pd_east,pd_south,pd_west); // FUNCTION THAT COMPARES WHICH DIRECTION HAS SMALLEST PD NUMBER
    Serial.print(inv(matX));
    Serial.print(" ");
    Serial.print(inv(matY));
    Serial.print(" ");
    Serial.print(pathDir);
    Serial.print(" ");
    matX = pd_xDir; matY = pd_yDir; //updates the current (or soon to be current) location of the robot
    turnAction = (4+pathDir-rDir)%4;
    rDir = pathDir; //New robot direction
    Serial.print(turnAction);
    Serial.print(" ");
    Serial.print(rDir);
    Serial.println(" ");
    delay(2000); //REMOVE THIS
}
