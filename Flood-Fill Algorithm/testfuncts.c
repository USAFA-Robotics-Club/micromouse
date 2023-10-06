/** testfuncts.c
 * =============================================================
 * Name: Victor Chen
 * Section:  T 6/7
 * Project:  MicroMouse Flood Fill Algorithm
 * Purpose:  This program holds the test statements to ensure functionality
 * of the functions within functs.c
 * Documentation Statement:  NONE
 * ============================================================= */

#include "testfuncts.h"
#include "functs.h"


// This function tests the function singleCellFloodFill(), which runs the
// flood fill algorithm on a single cell, based on the current flood fill value
// along with the position and wall values of each cell.
void testSingleCellFloodFill() {


    // Create a maze to run the test on
    Cell** maze = (Cell**)malloc(sizeof(Cell*) * 16);
    for (int row = 0; row < 16; row++) {

        maze[row] = (Cell*)malloc(sizeof(Cell) * 16);

        for (int col = 0; col < 16; col++) {
            maze[row][col].floodfillValue = 255;          // with each floodfillValue initialized to 255
            maze[row][col].topWall = FALSE;             // initialize all wall values to be FALSE
            maze[row][col].bottomWall = FALSE;
            maze[row][col].leftWall = FALSE;
            maze[row][col].rightWall = FALSE;
        }
    }

    maze[5][5].floodfillValue = 10;     // set a single cell to the value of 10

    singleCellFloodFill(maze, 5, 5, 10);    // run flood fill on that cell with floodfillValue of 10

    assert(maze[4][5].floodfillValue == 11);
    assert(maze[6][5].floodfillValue == 11);
    assert(maze[5][4].floodfillValue == 11);
    assert(maze[5][6].floodfillValue == 11);

    printf("singleCellFloodFill Tests Complete\n\n");

}


// This functions tests the moveRobot() function, which allows
// the microMouse robot to decide on which cell it can and will
// travel to, based on wall limitations and floodfill values
// of surround cells.
void testMoveRobot() {
    
    Cell** theMaze = initiateFloodFill();
    Robot theMicroMouse = initializeRobot(3, 0);

    theMaze[15][0].topWall = TRUE;

    moveRobot(theMaze, &theMicroMouse);

    assert(theMicroMouse.orientation == 1);
    assert(theMicroMouse.robotCol == 1);
    assert(theMicroMouse.robotRow == 15);

    printf("moveRobot Tests Complete\n\n");

}


// This function requires visual inspection
// This function displays a sample run of the flood fill algorithm
// on a 16 x 16 maze design with walls filled in. This essentially
// displays all the wall values, along with flood fill values, if the 
// robot were to identify each and every single wall.
void testFloodFillOnSampleMaze() {

    int userInput = 0;

    printf("Type 1 to initialize a flood fill sample: ");

    scanf("%d", &userInput);

    if (userInput == 1) {

        initscr();

        //Cell** theMaze = initiateFloodFill();
        Cell** theMazeWall = readFloodFillFile("maze2.txt");
        Robot theMicroMouse = initializeRobot(3, 0);

        theMazeWall = runFloodFill(theMazeWall);
        displayMaze(theMazeWall, theMicroMouse);

        getch();
        endwin();

        printf("Flood Fill Display Complete\n\n");

    }
    else {
        printf("Skipped flood fill sample\n\n");
    }

}


// This function combines all tests to run together
void runAllTests() {

    printf("Initiate Final Project Tests . . .\n\n");

    testSingleCellFloodFill();

    testMoveRobot();

    testFloodFillOnSampleMaze();

    printf("All Final Project Tests Complete\n");

}