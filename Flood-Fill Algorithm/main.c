/** main.c
 * =============================================================
 * Name: Victor Chen
 * Section:  T 6/7
 * Project:  MicroMouse Flood Fill Algorithm
 * Purpose:  This program is a simulation of a micromouse robot trying to
 * solve a 16 x 16 maze by using a flood fill algorithm to reach the center.
 * Documentation Statement:  NONE
 * ============================================================= */



#include "functs.h"
#include "testfuncts.h"

#define WHITE_BLACK 1
#define RED_BLACK 2

// This function runs through the entire program
void runThroughMaze() {

    char userFileInput[20];
    int startingCorner;
    int startingOrientation;

    printf("\n\tWELCOME TO A SIMULATION OF A MICROMOUSE FLOODFILL ALGORITHM\n\n");
    printf("\t   This program uses Curses to display a 16x16 micromouse maze\n");
    printf("\twith the micromouse depicted as 'A','V', '<', '>', to present the\n");
    printf("\torientation of the micromouse robot.\n\n");

    printf(" Maze Options:           [maze2.txt]   [maze3.txt]\n");
    printf(" Starting Positions:    [0 - topleft] [1 - topright] [2 - bottomright] [3 - bottomleft]\n");
    printf(" Starting Orientations:  [0 - north]    [1 - east]      [2 - south]       [3 - west]\n\n");

    printf("Enter maze file name to use (no brackets): ");
    scanf("%s", userFileInput);
    printf("Enter micromouse starting corner position: ");
    scanf("%d", &startingCorner);
    printf("Enter micromouse starting orientation: ");
    scanf("%d", &startingOrientation);


    initscr();

    start_color();

    // Define Color Pairs Here
    init_pair(WHITE_BLACK, COLOR_WHITE, COLOR_BLACK);
    init_pair(RED_BLACK, COLOR_RED, COLOR_BLACK);

    wbkgd(stdscr, COLOR_PAIR(WHITE_BLACK));

    attron(COLOR_PAIR(WHITE_BLACK));

    cursesPrintDisplay();

    Cell** theMaze = initiateFloodFill();
    Cell** theMazeWall = readFloodFillFile(userFileInput);
    Robot theMicroMouse = initializeRobot(startingCorner, startingOrientation);

    // First Display Maze Wall Map
    displayMaze(theMazeWall, theMicroMouse);
    move(11, 80);
    printw("Entire Maze File  ");

    // Then Display the Entire Maze with Flood Fill
    theMazeWall = runFloodFill(theMazeWall);
    displayMaze(theMazeWall, theMicroMouse);
    move(11, 80);
    printw("Maze w Flood Fill ");

    // Display the Initial Empty Map
    displayMaze(theMaze, theMicroMouse);
    move(11, 80);
    printw("Maze w Known Walls");

    int totalCellsTraveled = 0;

    move(6, 91);
    printw("%3d", totalCellsTraveled);

    updateMazeValues(theMaze, theMicroMouse, theMazeWall);

    displayMaze(theMaze, theMicroMouse);

    totalCellsTraveled++;

    move(6, 91);
    printw("%3d", totalCellsTraveled);

    // This while loop is where the micromouse runs through the maze, moving, updating,
    // floodfilling, and displaying progress, until it reaches the center of the maze
    while(theMaze[theMicroMouse.robotRow][theMicroMouse.robotCol].floodfillValue != 0) {
        
        moveRobot(theMaze, &theMicroMouse);

        updateMazeValues(theMaze, theMicroMouse, theMazeWall);

        theMaze = runFloodFill(theMaze);

        displayMaze(theMaze, theMicroMouse);

        totalCellsTraveled++;

        move(6, 91);
        printw("%3d", totalCellsTraveled);

    }

    attron(A_STANDOUT);
    move(12, 69);
    printw("MAZE COMPLETE");
    attroff(A_STANDOUT);
    attron(A_BLINK);
    move(13, 69);
    printw("Press Key to Return to Start");
    attroff(A_BLINK);

    move(19, 69);
    returnToOrigin(theMaze, theMicroMouse, theMazeWall, startingCorner);
    move(21, 69);
    attron(A_STANDOUT);
    move(12, 69);
    printw("RETURNED TO ORIGIN");
    attroff(A_STANDOUT);

    move(34, 66);
    getch();
    endwin();

}

int main() {

    // Uncomment to Run Through Example Maze
    runThroughMaze();

    // Uncomment to Run Test Functions
    //runAllTests();

}
