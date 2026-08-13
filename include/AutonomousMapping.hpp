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
        // The Nano only has 2 KB of SRAM. Store all five flags in one byte
        // instead of using five separate bool bytes per maze cell.
        uint8_t visited   : 1;
        uint8_t wallNorth : 1;
        uint8_t wallEast  : 1;
        uint8_t wallSouth : 1;
        uint8_t wallWest  : 1;
        uint8_t reserved  : 3;
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
        state = READY;
        goalReached = false;

        clearMap();

        Serial.println("4.3 Autonomous Mapping started");
    }

    void update() {
        if (!maze[currentRow][currentCol].visited) {
            maze[currentRow][currentCol].visited = true;
            visitedCells++;
            printMap();
        }

        // Robot motor, encoder and IMU controllers must be serviced on every
        // pass through loop(), including while a mapping action is running.
        robot.update();

        if (goalReached) {
            return;
        }

        if (currentRow == goalRow && currentCol == goalCol) {
            robot.stop();
            goalReached = true;
            Serial.println("Autonomous mapping goal reached");
            printMap();
            return;
        }

        if (!robot.isFinished()) {
            return;
        }

        if (state == MOVING) {
            if (robot.wasStoppedByFrontObstacle()) {
                addWall(currentRow, currentCol, currentDirection);
                Serial.print(F("Wall added at row "));
                Serial.print(currentRow);
                Serial.print(F(", column "));
                Serial.print(currentCol);
                Serial.print(F(", direction "));
                Serial.println(static_cast<uint8_t>(currentDirection));
                state = READY;
                printMap();
                return;
            }

            currentRow = nextRow;
            currentCol = nextCol;
            state = READY;
            return;
        }

        if (state == TURNING) {
            currentDirection = targetDirection;
            state = READY;
            return;
        }

        if (!chooseNextDirection(targetDirection)) {
            robot.stop();
            goalReached = true;
            Serial.println(F("Mapping stopped: no open neighbouring cell"));
            return;
        }

        if (currentDirection != targetDirection) {
            int8_t quarterTurns =
                static_cast<int8_t>(targetDirection) -
                static_cast<int8_t>(currentDirection);

            if (quarterTurns > 2) quarterTurns -= 4;
            if (quarterTurns < -2) quarterTurns += 4;

            robot.startTurn(quarterTurns * 90.0f);
            state = TURNING;
            return;
        }

        nextRow = currentRow;
        nextCol = currentCol;

        switch (currentDirection) {
            case NORTH: nextRow--; break;
            case EAST:  nextCol++; break;
            case SOUTH: nextRow++; break;
            case WEST:  nextCol--; break;
        }

        robot.startStraightLine(CELL_DISTANCE_MM, DRIVE_PWM);
        state = MOVING;
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

    enum State {
        READY,
        TURNING,
        MOVING
    };

    State state = READY;
    Direction targetDirection = NORTH;
    uint8_t nextRow = 0;
    uint8_t nextCol = 0;
    bool goalReached = false;

    static constexpr float CELL_DISTANCE_MM = 180.0f;
    static constexpr int16_t DRIVE_PWM = 130;

    bool getNeighbour(uint8_t row, uint8_t col, Direction direction,
                      uint8_t& neighbourRow, uint8_t& neighbourCol) const {
        neighbourRow = row;
        neighbourCol = col;

        switch (direction) {
            case NORTH:
                if (row == 0) return false;
                neighbourRow--;
                break;
            case EAST:
                if (col + 1 >= MAZE_SIZE) return false;
                neighbourCol++;
                break;
            case SOUTH:
                if (row + 1 >= MAZE_SIZE) return false;
                neighbourRow++;
                break;
            case WEST:
                if (col == 0) return false;
                neighbourCol--;
                break;
        }

        return true;
    }

    bool hasWall(uint8_t row, uint8_t col, Direction direction) const {
        const Cell& cell = maze[row][col];

        switch (direction) {
            case NORTH: return cell.wallNorth;
            case EAST:  return cell.wallEast;
            case SOUTH: return cell.wallSouth;
            case WEST:  return cell.wallWest;
        }

        return true;
    }

    void setWallFlag(uint8_t row, uint8_t col, Direction direction) {
        Cell& cell = maze[row][col];

        switch (direction) {
            case NORTH: cell.wallNorth = true; break;
            case EAST:  cell.wallEast = true;  break;
            case SOUTH: cell.wallSouth = true; break;
            case WEST:  cell.wallWest = true;  break;
        }
    }

    void addWall(uint8_t row, uint8_t col, Direction direction) {
        setWallFlag(row, col, direction);

        uint8_t neighbourRow;
        uint8_t neighbourCol;
        if (getNeighbour(row, col, direction, neighbourRow, neighbourCol)) {
            Direction opposite =
                static_cast<Direction>((static_cast<uint8_t>(direction) + 2) % 4);
            setWallFlag(neighbourRow, neighbourCol, opposite);
        }
    }

    bool chooseNextDirection(Direction& chosenDirection) const {
        bool found = false;
        uint16_t bestScore = 0xFFFF;

        for (uint8_t value = NORTH; value <= WEST; value++) {
            Direction direction = static_cast<Direction>(value);
            if (hasWall(currentRow, currentCol, direction)) continue;

            uint8_t row;
            uint8_t col;
            if (!getNeighbour(currentRow, currentCol, direction, row, col)) {
                continue;
            }

            // Prefer cells closer to the goal. An unvisited cell receives a
            // strong preference so the robot explores around known walls
            // instead of immediately returning to the previous cell.
            uint16_t score =
                abs(static_cast<int16_t>(row) - goalRow) +
                abs(static_cast<int16_t>(col) - goalCol);

            if (maze[row][col].visited) score += MAZE_SIZE * MAZE_SIZE;

            if (!found || score < bestScore) {
                found = true;
                bestScore = score;
                chosenDirection = direction;
            }
        }

        return found;
    }

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
