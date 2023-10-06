/** functs.h
 * =============================================================
 * Name: Victor Chen
 * Section:  T 6/7
 * Project:  MicroMouse Flood Fill Algorithm
 * Purpose:  This program is a simulation of a micromouse robot trying to
 * solve a 16 x 16 maze by using a flood fill algorithm to reach the center.
 * Documentation Statement:  NONE
 * ============================================================= */


#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define NORTH 0;
#define EAST  1;
#define SOUTH 2;
#define WEST  3;

#define WHITE_BLACK 1
#define RED_BLACK 2


typedef struct Robot {

    int orientation; // 0 - north, 1 - east, south - 2, west, 3
    int robotRow;    // 0-15
    int robotCol;    // 0-15

} Robot;

// This is the struct for each individual cell of the maze
// It holds TRUE/FALSE values for walls, and holds a floodfillValue
typedef struct Cell {

    bool topWall;
    bool bottomWall;
    bool leftWall;
    bool rightWall;

    int floodfillValue;     // 0 --> infinity

} Cell;


// This function creates the maze for floodfill
Cell** initiateFloodFill();


void updateMazeValues(Cell** maze, Robot microMouse, Cell** mazeWall);


Robot initializeRobot(int initialCorner, int initialOrientation);


void moveRobot(Cell** maze, Robot* microMouse);


void singleCellFloodFill(Cell** maze, int row, int col, int floodValue);


Cell** runFloodFill(Cell** maze);


Cell** returnFloodFill(Cell** maze, int initialCorner);

void returnToOrigin(Cell** maze, Robot microMouse, Cell** mazeWall, int initialCorner);