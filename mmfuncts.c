/*
    Major functions for the ECE Robot Club MicroMouse robot
*/

// Includes
#include "fffuncts.h"
#include "mmfuncts.h"

/**
 * Main function to control robot
*/
void controlRobot(void) {
    // Info to init robot (by definition)
    int startingCorner = 0;
    int startingOrientation = 0;

    // Init maze and robot
    Cell** theMaze = initiateFloodFill();
    Robot theMicroMouse = initializeRobot(startingCorner, startingOrientation);

    // Main loop
    while(theMaze[theMicroMouse.robotRow][theMicroMouse.robotCol].floodfillValue != 0) {
        
        observe();

        think(theMaze, theMicroMouse, data??);

        pathfind(theMaze, &theMicroMouse);

    }
}

/**
 * Robot reads surroundings and updates
*/
void observe(void);

/**
 * 
*/
void think(Cell** maze, Robot robot, data??) {
    updateMazeValues(maze, robot, data??);
    maze = runFloodFill(maze);
}

/**
 * 
*/
void pathfind(Cell** maze, Robot* robotptr) {
    moveRobot(maze, robotptr);
}