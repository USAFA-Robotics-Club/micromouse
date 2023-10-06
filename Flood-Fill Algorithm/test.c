

#include "functs.h"
#include "testfuncts.h"


void runThroughMaze() {

    initscr();

    Cell** theMaze = initiateFloodFill();
    Cell** theMazeWall = readFloodFillFile("maze2.txt");
    Robot theMicroMouse = initializeRobot(3, 0);

    displayMaze(theMazeWall, theMicroMouse);

    theMazeWall = runFloodFill(theMazeWall);
    displayMaze(theMazeWall, theMicroMouse);

    move(12, 69);
    printw("MAIN TEST POINT 0");

    displayMaze(theMaze, theMicroMouse);

    move(11, 69);
    printw("MAIN TEST POINT 1");

    updateMazeValues(theMaze, theMicroMouse, theMazeWall);

    displayMaze(theMaze, theMicroMouse);

    while(theMaze[theMicroMouse.robotRow][theMicroMouse.robotCol].floodfillValue != 0) {
        
        moveRobot(theMaze, &theMicroMouse);

        updateMazeValues(theMaze, theMicroMouse, theMazeWall);

        theMaze = runFloodFill(theMaze);

        displayMaze(theMaze, theMicroMouse);

        move(8, 69);
        printw("%d %d", theMaze[10][0].rightWall, theMaze[10][1].leftWall);
        move(7, 69);
        printw("%d %d", theMaze[1][7].bottomWall, theMaze[2][7].topWall);

    }

    endwin();

}

int main() {

    // Uncomment to Run Through Example Maze
    //runThroughMaze();

    // Uncomment to Run Test Functions
    runAllTests();

}
