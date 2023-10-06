/** functs.c
 * =============================================================
 * Name: Victor Chen
 * Section:  T 6/7
 * Project:  MicroMouse Flood Fill Algorithm
 * Purpose:  This program is a simulation of a micromouse robot trying to
 * solve a 16 x 16 maze by using a flood fill algorithm to reach the center.
 * Documentation Statement:  NONE
 * ============================================================= */


#include "fffuncts.h"
#include <time.h>


/**
 * @brief This function generates a 16 x 16 maze with no walls, except for the outer boundary
 * walls of the maze, which is essentially how the robot sees the entire maze as it begins.
 * 
 * 
 * @return Cell** 
 */
Cell** initiateFloodFill() {//char filename[]) {

    // Dynamically allocates a 16x16 array of Cell values for the maze
    Cell** maze = (Cell**)malloc(sizeof(Cell*) * 16);
    for (int row = 0; row < 16; row++) {

        maze[row] = (Cell*)malloc(sizeof(Cell) * 16);

        for (int col = 0; col < 16; col++) {
            maze[row][col].floodfillValue = 0;          // with each floodfillValue initialized to 0
            maze[row][col].topWall = FALSE;             // initialize all wall values to be FALSE
            maze[row][col].bottomWall = FALSE;
            maze[row][col].leftWall = FALSE;
            maze[row][col].rightWall = FALSE;
        }
    }

    // These functions create the outer boundary of the maze
    for (int col = 0; col < 16; col++) {
        maze[0][col].topWall = TRUE;
        maze[15][col].bottomWall = TRUE;
    }
    for (int row = 0; row < 16; row++) {
        maze[row][0].leftWall = TRUE;
        maze[row][15].rightWall = TRUE;
    }

    // These functions fill the initial maze values, starting with zeroes in the center and increasing
    for (int row = 0; row < 8; row++) {
        int initVal = 14 - row;
        for (int col = 0; col < 8; col++) {
            maze[row][col].floodfillValue = initVal;
            initVal--;
        }
        initVal++;
        for (int col = 8; col < 16; col++) {
            maze[row][col].floodfillValue = initVal;
            initVal++;
        }
    }
    for (int row = 8; row < 16; row++) {
        int initVal = row - 1;
        for (int col = 0; col < 8; col++) {
            maze[row][col].floodfillValue = initVal;
            initVal--;
        }
        initVal++;
        for (int col = 8; col < 16; col++) {
            maze[row][col].floodfillValue = initVal;
            initVal++;
        }
    }
    return maze;
}


/**
 * @brief This function will update the maze wall values as the micrmouse
 * "discovers" new walls in each and every cell it travels to. This function also
 * accounts for the issue of, if the Robot discovers a right wall in cell(0,0),
 * then cell(0,1) should have a left wall.
 * 
 * @param maze 
 * @param microMouse 
 * @param mazeWall 
 */
void updateMazeValues(Cell** maze, Robot microMouse, Cell** mazeWall) {  

    // This function updates the maze with mazeWall wall values
    // to simulate the robot detecting new walls within the new cell it entered
    int row = microMouse.robotRow;
    int col = microMouse.robotCol;

    maze[row][col].topWall = mazeWall[row][col].topWall;
    maze[row][col].bottomWall = mazeWall[row][col].bottomWall;          // This will become read wall function
    maze[row][col].leftWall = mazeWall[row][col].leftWall;
    maze[row][col].rightWall = mazeWall[row][col].rightWall;

    // These functions account for the four corners, then the boundary cells, then the interior
    
    // top left corner
    if (maze[0][0].rightWall == TRUE) {
        maze[0][1].leftWall = TRUE;
    }
    if (maze[0][0].bottomWall == TRUE) {
        maze[1][0].topWall = TRUE;
    }
    // top right corner
    if (maze[0][15].leftWall == TRUE) {
        maze[0][14].rightWall = TRUE;
    }
    if (maze[0][15].bottomWall == TRUE) {
        maze[1][15].topWall = TRUE;
    }
    // bottom right corner
    if (maze[15][15].leftWall == TRUE) {
        maze[0][14].rightWall = TRUE;
    }
    if (maze[15][15].topWall == TRUE) {
        maze[14][15].bottomWall = TRUE;
    }
    // bottom left corner
    if (maze[15][0].rightWall == TRUE) {
        maze[15][1].leftWall = TRUE;
    }
    if (maze[15][0].topWall == TRUE) {
        maze[14][0].bottomWall = TRUE;
    }

    // This function accounts for the top and bottom boundary cells
    for (int col = 1; col < 15; col++) {

        if (maze[0][col].leftWall == TRUE) {
            maze[0][col - 1].rightWall = TRUE;
        }
        if (maze[0][col].rightWall == TRUE) {
            maze[0][col + 1].leftWall = TRUE;
        }
        if (maze[0][col].bottomWall == TRUE) {
            maze[1][col].topWall = TRUE;
        }

        if (maze[15][col].leftWall == TRUE) {
            maze[15][col - 1].rightWall = TRUE;
        }
        if (maze[15][col].rightWall == TRUE) {
            maze[15][col + 1].leftWall = TRUE;
        }
        if (maze[15][col].topWall == TRUE) {
            maze[14][col].bottomWall = TRUE;
        }
    }

    // This function accounts for the left and right boundary cells
    for (int row = 1; row < 15; row++) {

        if (maze[row][0].topWall == TRUE) {
            maze[row - 1][0].bottomWall = TRUE;
        }
        if (maze[row][0].bottomWall == TRUE) {
            maze[row + 1][0].topWall = TRUE;
        }
        if (maze[row][0].rightWall == TRUE) {
            maze[row][1].leftWall = TRUE;
        }

        if (maze[row][15].topWall == TRUE) {
            maze[row - 1][15].bottomWall = TRUE;
        }
        if (maze[row][15].bottomWall == TRUE) {
            maze[row + 1][15].topWall = TRUE;
        }
        if (maze[row][15].leftWall == TRUE) {
            maze[row][14].rightWall = TRUE;
        }

    }

    // This function accounts for the interior cells of the maze
    for (int row = 1; row < 15; row++) {
        for (int col = 1; col < 15; col++) {

            if (maze[row][col].topWall == TRUE) {
                maze[row - 1][col].bottomWall = TRUE;
            }
            if (maze[row][col].rightWall == TRUE) {
                maze[row][col + 1].leftWall = TRUE;
            }
            if (maze[row][col].bottomWall == TRUE) {
                maze[row + 1][col].topWall = TRUE;
            }
            if (maze[row][col].leftWall == TRUE) {
                maze[row][col - 1].rightWall = TRUE;
            }
        }
    }
}

/**
 * @brief This function starts the robot off in the a select corner and orientation
 * 
 * @param initialCorner ranges from 0-3, with 0 = top-left corner, 1 = top-right corner, 2 = bottom-right corner, and 3 = bottom-left corner
 * @param initialOrientation ranges from 0 -3, with 0 = facing top, 1 = facing right, 2 = facing bottom, 3 = facing left
 * @return Robot 
 */
Robot initializeRobot(int initialCorner, int initialOrientation) {

    Robot microMouse;

    if (initialCorner == 0) {
        microMouse.robotRow = 0;
        microMouse.robotCol = 0;
    }
    if (initialCorner == 1) {
        microMouse.robotRow = 0;
        microMouse.robotCol = 15;
    }
    if (initialCorner == 2) {
        microMouse.robotRow = 15;
        microMouse.robotCol = 15;
    }
    if (initialCorner == 3) {
        microMouse.robotRow = 15;
        microMouse.robotCol = 0;
    }

    microMouse.orientation = initialOrientation;

    return microMouse;

}

/**
 * @brief This function moves the micromouse robot through the maze based on
 * the surrounding maze wall and floodfill Values, this is where the main
 * algorithm happens in determining where the micromouse should travel to based
 * on its surrounding environment.
 * 
 * @param maze 
 * @param microMouse 
 */
void moveRobot(Cell** maze, Robot* microMouse) {                      // Need to insert motor control

    int row = microMouse->robotRow;
    int col = microMouse->robotCol;

    int compareArray[4] = {255, 255, 255, 255};

    if (maze[row][col].topWall == FALSE) {      // if there is no wall, give them floodfillValue
        compareArray[0] = maze[row - 1][col].floodfillValue;
    }
    if (maze[row][col].bottomWall == FALSE) {
        compareArray[2] = maze[row + 1][col].floodfillValue;
    }
    if (maze[row][col].leftWall == FALSE) {
        compareArray[3] = maze[row][col - 1].floodfillValue;
    }
    if (maze[row][col].rightWall == FALSE) {
        compareArray[1] = maze[row][col + 1].floodfillValue;
    }

    int lowestVal = compareArray[0];
    int lowestSide = 0;

    // This is where the micromouse determines which path to take
    for (int i = 1; i < 4; i++) {
        if (compareArray[i] < lowestVal) {
            lowestVal = compareArray[i];
            lowestSide = i;
        }
    }

    if (lowestSide == 0) {
        microMouse->orientation = 0;
        microMouse->robotRow = row - 1;
    }
    if (lowestSide == 1) {
        microMouse->orientation = 1;
        microMouse->robotCol = col + 1;
    }
    if (lowestSide == 2) {
        microMouse->orientation = 2;
        microMouse->robotRow = row + 1;
    }
    if (lowestSide == 3) {
        microMouse->orientation = 3;
        microMouse->robotCol = col - 1;
    }

}

/**
 * @brief This function is the most basic function of flood fill, which
 * increments each time it floods another cell. See more in the runFloodFill()
 * function, which incorporates this function. Honestly, this function itself
 * can just be thrown into runFloodFill().
 * 
 * @param maze 
 * @param row the current row to update cell values
 * @param col the current col to updates cell values
 * @param floodValue increment by one based on the current floodValue, for example,
 * if a cell has a floodfillValue of 5, its surrounding cells should have 6.
 */
void singleCellFloodFill(Cell** maze, int row, int col, int floodValue) {

    if (maze[row][col].floodfillValue == floodValue) {

        if ((maze[row][col].topWall == FALSE) && (maze[row - 1][col].floodfillValue == 255)) {
            maze[row - 1][col].floodfillValue = floodValue + 1;
        }
        if ((maze[row][col].rightWall == FALSE) && (maze[row ][col + 1].floodfillValue == 255)) {
            maze[row][col + 1].floodfillValue = floodValue + 1;
        }   
        if ((maze[row][col].bottomWall == FALSE) && (maze[row + 1][col].floodfillValue == 255)) {
            maze[row + 1][col].floodfillValue = floodValue + 1;
        }
        if ((maze[row][col].leftWall == FALSE) && (maze[row][col - 1].floodfillValue == 255)) {
            maze[row][col - 1].floodfillValue = floodValue + 1;
        }
    }
}

/**
 * @brief This function is where the floodfill magic happens. A new maze is generated
 * and with every single cell's (except the center four, which marks the end of the maze)
 * floodfillValue set to 255. It then floods the entire maze, based on the known wall that it
 * encounters throughout the floodfill.
 * 
 * @param maze is the current version of the maze we are in
 * @return Cell** 
 */
Cell** runFloodFill(Cell** maze) { // run with newly discovered wall values

    // This is where we run the flood fill algorithm, flood filling from
    // the center and out

    // take the floodfillvalue of the current cell, add one to it, then apply to
    // surrounding cells, remember to account for walls

    // gets old maze and creates/returns a new one

    Cell** newMaze = (Cell**)malloc(sizeof(Cell*) * 16);
    for (int row = 0; row < 16; row++) {

        newMaze[row] = (Cell*)malloc(sizeof(Cell) * 16);

        for (int col = 0; col < 16; col++) {
            newMaze[row][col].floodfillValue = 255;                         // set all floodfill values to 255
            newMaze[row][col].topWall = maze[row][col].topWall;             // copy all wall values from old maze
            newMaze[row][col].bottomWall = maze[row][col].bottomWall;       // copied with updated walls
            newMaze[row][col].leftWall = maze[row][col].leftWall;
            newMaze[row][col].rightWall = maze[row][col].rightWall;
        }
    }

    newMaze[7][8].floodfillValue = 0;       // make maze center four values zero
    newMaze[7][7].floodfillValue = 0;
    newMaze[8][7].floodfillValue = 0;
    newMaze[8][8].floodfillValue = 0;


    int currfloodfillVal = 0;
    int noFillCounter = 256;
    
    while (noFillCounter > 0) {             // While there are still empty cells "255 valued cells"
                                            // noFillCounter will count how many cells have floodfillValue of 255.
        for (int row = 0; row < 16; row++) {
            for (int col = 0; col < 16; col++) {
                singleCellFloodFill(newMaze, row, col, currfloodfillVal);
            }
        }

        currfloodfillVal++;
        noFillCounter = 0;

        for (int row = 0; row < 16; row++) {            // This function checks how many unfilled cells there are
            for (int col = 0; col < 16; col++) {
                if (newMaze[row][col].floodfillValue == 255) {
                    noFillCounter++;
                }
            }
        }
    }

    free(maze);

    return newMaze;
}


/**
 * @brief This function runs the flood fill for the return
 * trip of the micromouse, back to its origin
 * 
 * @param maze 
 * @return Cell** 
 */
Cell** returnFloodFill(Cell** maze, int initialCorner) {


    Cell** newMaze = (Cell**)malloc(sizeof(Cell*) * 16);
    for (int row = 0; row < 16; row++) {

        newMaze[row] = (Cell*)malloc(sizeof(Cell) * 16);

        for (int col = 0; col < 16; col++) {
            newMaze[row][col].floodfillValue = 255;                         // set all floodfill values to 255
            newMaze[row][col].topWall = maze[row][col].topWall;             // copy all wall values from old maze
            newMaze[row][col].bottomWall = maze[row][col].bottomWall;       // copied with updated walls
            newMaze[row][col].leftWall = maze[row][col].leftWall;
            newMaze[row][col].rightWall = maze[row][col].rightWall;
        }
    }

    if (initialCorner == 0) {newMaze[0][0].floodfillValue = 0;}
    if (initialCorner == 1) {newMaze[0][15].floodfillValue = 0;}
    if (initialCorner == 2) {newMaze[15][15].floodfillValue = 0;}
    if (initialCorner == 3) {newMaze[15][0].floodfillValue = 0;}

    int currfloodfillVal = 0;
    int noFillCounter = 256;
    
    while (noFillCounter > 0) {             // While there are still empty cells "255 valued cells"
                                            // noFillCounter will count how many cells have floodfillValue of 255.
        for (int row = 0; row < 16; row++) {
            for (int col = 0; col < 16; col++) {
                singleCellFloodFill(newMaze, row, col, currfloodfillVal);
            }
        }

        currfloodfillVal++;
        noFillCounter = 0;

        for (int row = 0; row < 16; row++) {            // This function checks how many unfilled cells there are
            for (int col = 0; col < 16; col++) {
                if (newMaze[row][col].floodfillValue == 255) {
                    noFillCounter++;
                }
            }
        }
    }

    free(maze);

    return newMaze;

}

/**
 * @brief This function will return the microMouse back to its origin based on
 * the fastest discovered path.
 * 
 * @param maze 
 * @param microMouse 
 */
void returnToOrigin(Cell** maze, Robot microMouse, Cell** mazeWall, int initialCorner) {

    int totalCellsTraveled = 0;

    Cell** newMaze = returnFloodFill(maze, initialCorner);

    displayMaze(newMaze, microMouse);

    while(newMaze[microMouse.robotRow][microMouse.robotCol].floodfillValue != 0) {
        
        moveRobot(newMaze, &microMouse);

        updateMazeValues(newMaze, microMouse, mazeWall);

        newMaze = returnFloodFill(newMaze, initialCorner);

        displayMaze(newMaze, microMouse);

        totalCellsTraveled++;

        move(6, 91);
        printw("%3d", totalCellsTraveled);
    }
}