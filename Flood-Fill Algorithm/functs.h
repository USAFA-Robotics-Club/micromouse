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
#include <ncurses/ncurses.h>
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

Cell** readFloodFillFile(char filename[]);


// This function creates the maze for floodfill
Cell** initiateFloodFill();


void updateMazeValues(Cell** maze, Robot microMouse, Cell** mazeWall);


Robot initializeRobot(int initialCorner, int initialOrientation);


void moveRobot(Cell** maze, Robot* microMouse);


void singleCellFloodFill(Cell** maze, int row, int col, int floodValue);


Cell** runFloodFill(Cell** maze);


// This function will require information on the robot and the maze
void displayMaze(Cell** maze, Robot microMouse);


Cell** returnFloodFill(Cell** maze, int initialCorner);


void displayReturnMaze(Cell** maze, Robot microMouse);


// This function will return the microMouse to the origin in the quickest path
void returnToOrigin(Cell** maze, Robot microMouse, Cell** mazeWall, int initialCorner);

// DISPLAY SUB FUNCTIONS IN CURSES
void cursesPrintDisplay();