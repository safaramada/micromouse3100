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


    AutonomousMapping(Robot& robot)
        : robot(robot) {}


    void begin(
        uint8_t startRow,
        uint8_t startCol,
        Direction startDirection,
        uint8_t goalRow,
        uint8_t goalCol
    ) {

        this->startRow = startRow;
        this->startCol = startCol;

        this->goalRow = goalRow;
        this->goalCol = goalCol;

        currentRow = startRow;
        currentCol = startCol;

        currentDirection = startDirection;

        clearMap();

        visited[currentRow][currentCol] = true;
        visitedCount = 1;

        state = OBSERVE;

        Serial.println();
        Serial.println("================================");
        Serial.println("Task 4.3 Autonomous Mapping");
        Serial.println("================================");

        printMap();
    }


    void update() {

        switch (state) {

            case OBSERVE:
                observeCell();
                break;


            case DECIDE:
                chooseNextMove();
                break;


            case TURNING:
                robot.update();

                if (robot.isFinished()) {

                    currentDirection = targetDirection;

                    Serial.println("Turn complete");

                    delay(100);

                    startForwardMovement();
                }

                break;


            case MOVING:
                robot.update();

                if (robot.isFinished()) {

                    currentRow = targetRow;
                    currentCol = targetCol;

                    if (!visited[currentRow][currentCol]) {

                        visited[currentRow][currentCol] = true;
                        visitedCount++;
                    }

                    Serial.print("Arrived at cell (");
                    Serial.print(currentRow);
                    Serial.print(",");
                    Serial.print(currentCol);
                    Serial.println(")");

                    printMap();

                    delay(150);

                    state = OBSERVE;
                }

                break;


            case STOPPED:
                robot.stop();
                break;
        }
    }


private:

    // ============================================================
    // MAZE
    // ============================================================

    static constexpr uint8_t MAZE_SIZE = 9;

    // Distance below this value is considered an adjacent wall.
    // This WILL probably need tuning on the physical maze.
    static constexpr uint16_t WALL_THRESHOLD_MM = 130;

    static constexpr float CELL_DISTANCE_MM = 180.0f;

    static constexpr int16_t FORWARD_PWM = 120;


    enum State {
        OBSERVE,
        DECIDE,
        TURNING,
        MOVING,
        STOPPED
    };


    struct Cell {

        bool northWall = false;
        bool eastWall = false;
        bool southWall = false;
        bool westWall = false;

        bool northKnown = false;
        bool eastKnown = false;
        bool southKnown = false;
        bool westKnown = false;
    };


    Robot& robot;

    Cell maze[MAZE_SIZE][MAZE_SIZE];

    bool visited[MAZE_SIZE][MAZE_SIZE];


    uint8_t startRow = 0;
    uint8_t startCol = 0;

    uint8_t goalRow = 0;
    uint8_t goalCol = 0;

    uint8_t currentRow = 0;
    uint8_t currentCol = 0;

    uint8_t targetRow = 0;
    uint8_t targetCol = 0;


    Direction currentDirection = NORTH;
    Direction targetDirection = NORTH;

    State state = STOPPED;

    uint8_t visitedCount = 0;


    // ============================================================
    // CLEAR MAP
    // ============================================================

    void clearMap() {

        for (uint8_t row = 0; row < MAZE_SIZE; row++) {

            for (uint8_t col = 0; col < MAZE_SIZE; col++) {

                visited[row][col] = false;

                maze[row][col] = Cell();
            }
        }


        // Outer maze boundaries are always walls.

        for (uint8_t i = 0; i < MAZE_SIZE; i++) {

            maze[0][i].northWall = true;
            maze[0][i].northKnown = true;

            maze[MAZE_SIZE - 1][i].southWall = true;
            maze[MAZE_SIZE - 1][i].southKnown = true;

            maze[i][0].westWall = true;
            maze[i][0].westKnown = true;

            maze[i][MAZE_SIZE - 1].eastWall = true;
            maze[i][MAZE_SIZE - 1].eastKnown = true;
        }
    }


    // ============================================================
    // SENSE CURRENT CELL
    // ============================================================

    void observeCell() {

        Serial.println();
        Serial.println("Reading LiDARs...");


        uint16_t frontDistance =
            robot.getFrontLidarDistance();

        bool frontValid =
            robot.frontLidarValid();


        uint16_t leftDistance =
            robot.getLeftLidarDistance();

        bool leftValid =
            robot.leftLidarValid();


        uint16_t rightDistance =
            robot.getRightLidarDistance();

        bool rightValid =
            robot.rightLidarValid();


        Serial.print("Front: ");
        Serial.print(frontDistance);
        Serial.print(" mm");

        Serial.print(" | Left: ");
        Serial.print(leftDistance);
        Serial.print(" mm");

        Serial.print(" | Right: ");
        Serial.print(rightDistance);
        Serial.println(" mm");


        if (frontValid) {

            bool wall =
                frontDistance < WALL_THRESHOLD_MM;

            setWall(
                currentDirection,
                wall
            );
        }


        if (leftValid) {

            Direction leftDirection =
                rotateLeft(currentDirection);

            bool wall =
                leftDistance < WALL_THRESHOLD_MM;

            setWall(
                leftDirection,
                wall
            );
        }


        if (rightValid) {

            Direction rightDirection =
                rotateRight(currentDirection);

            bool wall =
                rightDistance < WALL_THRESHOLD_MM;

            setWall(
                rightDirection,
                wall
            );
        }


        state = DECIDE;
    }


    // ============================================================
    // CHOOSE NEXT CELL
    // ============================================================

    void chooseNextMove() {

        /*
         * For the first version use:
         *
         * LEFT
         * FORWARD
         * RIGHT
         * BACK
         *
         * and prefer cells we have not visited.
         */

        Direction options[4] = {

            rotateLeft(currentDirection),

            currentDirection,

            rotateRight(currentDirection),

            opposite(currentDirection)
        };


        // First try unvisited cells.

        for (uint8_t i = 0; i < 4; i++) {

            Direction direction = options[i];

            if (canMove(direction) &&
                neighbourUnvisited(direction)) {

                beginMove(direction);

                return;
            }
        }


        // If everything nearby has already been visited,
        // move through any available opening.

        for (uint8_t i = 0; i < 4; i++) {

            Direction direction = options[i];

            if (canMove(direction)) {

                beginMove(direction);

                return;
            }
        }


        Serial.println("No available movement!");

        state = STOPPED;
    }


    // ============================================================
    // BEGIN MOVEMENT
    // ============================================================

    void beginMove(Direction direction) {

        if (!getNeighbour(
                direction,
                targetRow,
                targetCol)) {

            Serial.println("Invalid target cell");

            state = STOPPED;

            return;
        }


        targetDirection = direction;


        Serial.print("Moving toward cell (");
        Serial.print(targetRow);
        Serial.print(",");
        Serial.print(targetCol);
        Serial.println(")");


        if (direction == currentDirection) {

            startForwardMovement();

            return;
        }


        int turnAmount =
            getTurnAmount(
                currentDirection,
                direction
            );


        Serial.print("Turning: ");
        Serial.println(turnAmount);


        robot.startTurn(
            static_cast<float>(turnAmount)
        );

        state = TURNING;
    }


    // ============================================================
    // START FORWARD
    // ============================================================

    void startForwardMovement() {

        robot.startStraightLine(
            CELL_DISTANCE_MM,
            FORWARD_PWM
        );

        state = MOVING;
    }


    // ============================================================
    // DIRECTION HELPERS
    // ============================================================

    Direction rotateLeft(Direction direction) {

        return static_cast<Direction>(
            (direction + 3) % 4
        );
    }


    Direction rotateRight(Direction direction) {

        return static_cast<Direction>(
            (direction + 1) % 4
        );
    }


    Direction opposite(Direction direction) {

        return static_cast<Direction>(
            (direction + 2) % 4
        );
    }


    int getTurnAmount(
        Direction from,
        Direction to
    ) {

        int difference =
            (static_cast<int>(to) -
             static_cast<int>(from) +
             4) % 4;


        /*
         * IMPORTANT:
         *
         * Your existing Robot.hpp uses:
         *
         * +90 = LEFT
         * -90 = RIGHT
         */

        if (difference == 1) {

            return -90;
        }

        if (difference == 3) {

            return 90;
        }

        if (difference == 2) {

            return 180;
        }

        return 0;
    }


    // ============================================================
    // NEIGHBOUR
    // ============================================================

    bool getNeighbour(
        Direction direction,
        uint8_t& row,
        uint8_t& col
    ) {

        int newRow = currentRow;
        int newCol = currentCol;


        switch (direction) {

            case NORTH:
                newRow--;
                break;

            case EAST:
                newCol++;
                break;

            case SOUTH:
                newRow++;
                break;

            case WEST:
                newCol--;
                break;
        }


        if (newRow < 0 ||
            newRow >= MAZE_SIZE ||
            newCol < 0 ||
            newCol >= MAZE_SIZE) {

            return false;
        }


        row = static_cast<uint8_t>(newRow);
        col = static_cast<uint8_t>(newCol);

        return true;
    }


    bool neighbourUnvisited(
        Direction direction
    ) {

        uint8_t row;
        uint8_t col;


        if (!getNeighbour(
                direction,
                row,
                col)) {

            return false;
        }


        return !visited[row][col];
    }


    // ============================================================
    // WALL HANDLING
    // ============================================================

    bool canMove(Direction direction) {

        Cell& cell =
            maze[currentRow][currentCol];


        switch (direction) {

            case NORTH:
                return cell.northKnown &&
                       !cell.northWall;

            case EAST:
                return cell.eastKnown &&
                       !cell.eastWall;

            case SOUTH:
                return cell.southKnown &&
                       !cell.southWall;

            case WEST:
                return cell.westKnown &&
                       !cell.westWall;
        }


        return false;
    }


    void setWall(
        Direction direction,
        bool wall
    ) {

        Cell& cell =
            maze[currentRow][currentCol];


        switch (direction) {

            case NORTH:
                cell.northWall = wall;
                cell.northKnown = true;
                break;

            case EAST:
                cell.eastWall = wall;
                cell.eastKnown = true;
                break;

            case SOUTH:
                cell.southWall = wall;
                cell.southKnown = true;
                break;

            case WEST:
                cell.westWall = wall;
                cell.westKnown = true;
                break;
        }


        // Also update the same wall from
        // the neighbouring cell's perspective.

        uint8_t row;
        uint8_t col;


        if (!getNeighbour(
                direction,
                row,
                col)) {

            return;
        }


        Cell& neighbour =
            maze[row][col];


        switch (direction) {

            case NORTH:
                neighbour.southWall = wall;
                neighbour.southKnown = true;
                break;

            case EAST:
                neighbour.westWall = wall;
                neighbour.westKnown = true;
                break;

            case SOUTH:
                neighbour.northWall = wall;
                neighbour.northKnown = true;
                break;

            case WEST:
                neighbour.eastWall = wall;
                neighbour.eastKnown = true;
                break;
        }
    }


    // ============================================================
    // SERIAL MAP
    // ============================================================

    void printMap() {

        Serial.println();
        Serial.println("--------- MAP ---------");


        for (uint8_t row = 0; row < MAZE_SIZE; row++) {

            for (uint8_t col = 0; col < MAZE_SIZE; col++) {

                if (row == currentRow &&
                    col == currentCol) {

                    Serial.print("R ");

                }

                else if (row == goalRow &&
                         col == goalCol) {

                    Serial.print("G ");

                }

                else if (row == startRow &&
                         col == startCol) {

                    Serial.print("S ");

                }

                else if (visited[row][col]) {

                    Serial.print(". ");

                }

                else {

                    Serial.print("? ");
                }
            }

            Serial.println();
        }


        Serial.print("Visited: ");

        Serial.print(visitedCount);

        Serial.print("/81 (");

        Serial.print(
            (visitedCount * 100) / 81
        );

        Serial.println("%)");

        Serial.println("-----------------------");
    }
};

} // namespace mtrn3100