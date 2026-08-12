#pragma once

#include <Arduino.h>
#include "Robot.hpp"

namespace mtrn3100 {

class AutonomousMapping {
public:

    enum Direction {
        NORTH = 0,
        EAST  = 1,
        SOUTH = 2,
        WEST  = 3
    };

    struct Cell {
        bool visited = false;

        bool wallNorth = false;
        bool wallEast  = false;
        bool wallSouth = false;
        bool wallWest  = false;
    };

    AutonomousMapping(Robot& robot)
        : robot(robot) {}

    void begin(
        uint8_t startRow,
        uint8_t startCol,
        Direction startDirection,
        uint8_t goalRow,
        uint8_t goalCol
    ) {
        currentRow = startRow;
        currentCol = startCol;
        currentDirection = startDirection;

        this->startRow = startRow;
        this->startCol = startCol;

        this->goalRow = goalRow;
        this->goalCol = goalCol;

        visitedCells = 0;

        clearMap();

        Serial.println("4.3 Autonomous Mapping started");
    }

    void update() {

        // Later we will replace this with the full state machine.

        if (!maze[currentRow][currentCol].visited) {
            maze[currentRow][currentCol].visited = true;
            visitedCells++;
        }

        printMap();
    }

private:

    static constexpr uint8_t MAZE_SIZE = 9;

    Robot& robot;

    Cell maze[MAZE_SIZE][MAZE_SIZE];

    uint8_t currentRow = 0;
    uint8_t currentCol = 0;

    uint8_t startRow = 0;
    uint8_t startCol = 0;

    uint8_t goalRow = 0;
    uint8_t goalCol = 0;

    Direction currentDirection = NORTH;

    uint8_t visitedCells = 0;

    void clearMap() {

        for (uint8_t row = 0; row < MAZE_SIZE; row++) {
            for (uint8_t col = 0; col < MAZE_SIZE; col++) {

                maze[row][col].visited = false;

                maze[row][col].wallNorth = false;
                maze[row][col].wallEast = false;
                maze[row][col].wallSouth = false;
                maze[row][col].wallWest = false;
            }
        }
    }

    float getCompletionPercentage() {

        return
            (static_cast<float>(visitedCells) /
             static_cast<float>(MAZE_SIZE * MAZE_SIZE))
            * 100.0f;
    }

    void printMap() {

        Serial.println();
        Serial.println("----- MAZE MAP -----");

        for (uint8_t row = 0; row < MAZE_SIZE; row++) {

            for (uint8_t col = 0; col < MAZE_SIZE; col++) {

                if (row == currentRow && col == currentCol) {
                    Serial.print("R ");
                }
                else if (row == startRow && col == startCol) {
                    Serial.print("S ");
                }
                else if (row == goalRow && col == goalCol) {
                    Serial.print("G ");
                }
                else if (maze[row][col].visited) {
                    Serial.print(". ");
                }
                else {
                    Serial.print("? ");
                }
            }

            Serial.println();
        }

        Serial.print("Visited cells: ");
        Serial.print(visitedCells);
        Serial.print(" / ");
        Serial.println(MAZE_SIZE * MAZE_SIZE);

        Serial.print("Map completion: ");
        Serial.print(getCompletionPercentage(), 1);
        Serial.println("%");

        Serial.println("--------------------");
    }
};

}