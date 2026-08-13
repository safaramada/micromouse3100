#pragma once

#include <Arduino.h>
#include <U8x8lib.h>

#include "Robot.hpp"

namespace mtrn3100 {

// Task 4.3 controller: map the maze, return to the start through DFS
// backtracking, build a shortest route, then run that route to the goal.
class AutonomousMapping {
public:
    enum Direction {
        NORTH = 0,
        EAST  = 1,
        SOUTH = 2,
        WEST  = 3
    };

    enum CompletionReason {
        NOT_FINISHED,
        GOAL_REACHED,
        MAP_COMPLETE,
        INVALID_START_OR_GOAL,
        SENSOR_FAILURE,
        TURN_TIMEOUT,
        DRIVE_TIMEOUT,
        NO_PATH_TO_GOAL,
        UNEXPECTED_FRONT_WALL
    };

    explicit AutonomousMapping(Robot& robot)
        : robot(robot) {}

    void begin(uint8_t startRow,
               uint8_t startCol,
               Direction startDirection,
               uint8_t goalRow,
               uint8_t goalCol) {
        robot.stop();
        clearMap();

        if (!positionInBounds(startRow, startCol) ||
            !positionInBounds(goalRow, goalCol)) {
            finish(INVALID_START_OR_GOAL);
            return;
        }

        this->startRow = startRow;
        this->startCol = startCol;
        this->goalRow = goalRow;
        this->goalCol = goalCol;

        currentRow = startRow;
        currentCol = startCol;
        currentDirection = startDirection;
        targetRow = startRow;
        targetCol = startCol;
        targetDirection = startDirection;

        visitedCount = 0;
        sensingAttempts = 0;
        verificationAttempts = 0;
        goalReached = false;
        phase = EXPLORING;
        completionReason = NOT_FINISHED;
        displayDirty = true;

        markVisited(currentRow, currentCol, NORTH, false);
        enterState(SENSING, sensorSettleTimeMs);
    }

    void update() {
        if (state == FINISHED || state == ERROR_STATE) {
            return;
        }

        const unsigned long now = millis();
        if (!timeReached(now, stateReadyAtMs)) {
            return;
        }

        switch (state) {
            case SENSING:
                observeCell();
                break;
            case DECIDING:
                chooseNextMove();
                break;
            case TURNING:
                updateTurn();
                break;
            case VERIFYING_MOVE:
                verifyForwardPath();
                break;
            case MOVING:
                updateMovement();
                break;
            case WAITING_TO_SENSE:
                enterState(SENSING, 0);
                break;
            case FINISHED:
            case ERROR_STATE:
            default:
                break;
        }
    }

    bool isFinished() const {
        return state == FINISHED || state == ERROR_STATE;
    }

    bool succeeded() const {
        return state == FINISHED;
    }

    bool hasReachedGoal() const {
        return goalReached;
    }

    uint8_t getVisitedCount() const {
        return visitedCount;
    }

    CompletionReason getCompletionReason() const {
        return completionReason;
    }

    bool displayNeedsUpdate() const {
        return displayDirty;
    }

    // U8x8 draws one 8 x 8 tile at a time, so this visualization needs only
    // eight temporary bytes rather than a 1024-byte OLED framebuffer.
    void renderToOled(U8X8& oled) {
        for (uint8_t tileY = 0; tileY < 8; tileY++) {
            for (uint8_t tileX = 0; tileX < 8; tileX++) {
                uint8_t tile[8] = {0};
                for (uint8_t localX = 0; localX < 8; localX++) {
                    for (uint8_t localY = 0; localY < 8; localY++) {
                        const uint8_t pixelX = tileX * 8 + localX;
                        const uint8_t pixelY = tileY * 8 + localY;
                        if (mapPixelIsSet(pixelX, pixelY)) {
                            tile[localX] |= static_cast<uint8_t>(1U << localY);
                        }
                    }
                }
                oled.drawTile(tileX, tileY, 1, tile);
            }
        }

        for (uint8_t row = 0; row < 8; row++) {
            oled.drawString(8, row, "        ");
        }

        char number[4];
        oled.drawString(8, 0, "MAP");
        utoa(static_cast<uint16_t>(visitedCount) * 100 /
             (mazeSize * mazeSize), number, 10);
        oled.drawString(12, 0, number);
        oled.drawString(15, 0, "%");

        if (state == FINISHED) {
            oled.drawString(8, 1, "DONE");
        } else if (state == ERROR_STATE) {
            oled.drawString(8, 1, "STOPPED");
        } else if (phase == SHORTEST_RUN) {
            oled.drawString(8, 1, "SHORTEST");
        } else {
            oled.drawString(8, 1, "EXPLORE");
        }

        oled.drawString(8, 2, "BOT");
        number[0] = static_cast<char>('0' + currentRow);
        number[1] = ',';
        number[2] = static_cast<char>('0' + currentCol);
        number[3] = '\0';
        oled.drawString(12, 2, number);

        oled.drawString(8, 3, "GOAL");
        number[0] = static_cast<char>('0' + goalRow);
        number[1] = ',';
        number[2] = static_cast<char>('0' + goalCol);
        oled.drawString(13, 3, number);

        oled.drawString(8, 4, goalReached ? "FOUND" : "SEARCH");
        if (state == ERROR_STATE) {
            oled.drawString(8, 6, "ERR");
            utoa(static_cast<uint8_t>(completionReason), number, 10);
            oled.drawString(12, 6, number);
        }

        displayDirty = false;
    }

private:
    static const uint8_t mazeSize = 9;
    static const uint8_t wallMask = 0x0F;
    static const uint8_t visitedFlag = 0x10;
    static const uint8_t routeValidFlag = 0x20;
    static const uint8_t routeShift = 6;
    static const uint8_t routeMask = 0xC0;
    static const uint8_t bfsVisitedFlag = 0x10;

    static const uint8_t maxSensingAttempts = 3;
    static const uint16_t wallThresholdMm = 130;
    static const uint16_t movingFrontStopMm = 80;
    static const int16_t forwardPwm = 120;
    static const unsigned long sensorSettleTimeMs = 120;
    static const unsigned long retryDelayMs = 80;
    static const unsigned long turnSettleTimeMs = 100;
    static const unsigned long arrivalSettleTimeMs = 150;
    static const unsigned long turnTimeoutMs = 4000;
    static const unsigned long movementTimeoutMs = 5000;

    enum Phase {
        EXPLORING,
        SHORTEST_RUN
    };

    enum State {
        SENSING,
        DECIDING,
        TURNING,
        VERIFYING_MOVE,
        MOVING,
        WAITING_TO_SENSE,
        FINISHED,
        ERROR_STATE
    };

    // Four wall bits, visited, route-valid, and a two-bit route direction fit
    // in data. known uses four wall-known bits: the full map is 162 bytes.
    struct Cell {
        uint8_t data;
        uint8_t known;
    };

    Robot& robot;
    Cell maze[mazeSize][mazeSize];

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
    Phase phase = EXPLORING;
    State state = ERROR_STATE;
    CompletionReason completionReason = NOT_FINISHED;

    uint8_t visitedCount = 0;
    uint8_t sensingAttempts = 0;
    uint8_t verificationAttempts = 0;
    bool goalReached = false;
    bool displayDirty = true;

    unsigned long stateStartedAtMs = 0;
    unsigned long stateReadyAtMs = 0;

    static uint8_t directionBit(Direction direction) {
        return static_cast<uint8_t>(1U << static_cast<uint8_t>(direction));
    }

    static Direction rotateLeft(Direction direction) {
        return static_cast<Direction>((static_cast<uint8_t>(direction) + 3) % 4);
    }

    static Direction rotateRight(Direction direction) {
        return static_cast<Direction>((static_cast<uint8_t>(direction) + 1) % 4);
    }

    static Direction opposite(Direction direction) {
        return static_cast<Direction>((static_cast<uint8_t>(direction) + 2) % 4);
    }

    static bool timeReached(unsigned long now, unsigned long target) {
        return static_cast<long>(now - target) >= 0;
    }

    void enterState(State newState, unsigned long waitMs) {
        state = newState;
        stateStartedAtMs = millis();
        stateReadyAtMs = stateStartedAtMs + waitMs;
    }

    bool positionInBounds(int row, int col) const {
        return row >= 0 && row < mazeSize && col >= 0 && col < mazeSize;
    }

    void clearMap() {
        for (uint8_t row = 0; row < mazeSize; row++) {
            for (uint8_t col = 0; col < mazeSize; col++) {
                maze[row][col].data = 0;
                maze[row][col].known = 0;
            }
        }

        for (uint8_t index = 0; index < mazeSize; index++) {
            setCellWall(0, index, NORTH, true);
            setCellWall(mazeSize - 1, index, SOUTH, true);
            setCellWall(index, 0, WEST, true);
            setCellWall(index, mazeSize - 1, EAST, true);
        }
    }

    void markVisited(uint8_t row,
                     uint8_t col,
                     Direction parentDirection,
                     bool hasParent) {
        Cell& cell = maze[row][col];
        if ((cell.data & visitedFlag) != 0) {
            return;
        }

        cell.data |= visitedFlag;
        if (hasParent) {
            setRoute(cell, parentDirection);
        }
        visitedCount++;
        displayDirty = true;
    }

    bool isVisited(uint8_t row, uint8_t col) const {
        return (maze[row][col].data & visitedFlag) != 0;
    }

    bool hasRoute(const Cell& cell) const {
        return (cell.data & routeValidFlag) != 0;
    }

    Direction getRoute(const Cell& cell) const {
        return static_cast<Direction>((cell.data & routeMask) >> routeShift);
    }

    void clearRoute(Cell& cell) {
        cell.data &= static_cast<uint8_t>(~(routeValidFlag | routeMask));
    }

    void setRoute(Cell& cell, Direction direction) {
        clearRoute(cell);
        cell.data |= static_cast<uint8_t>(
            routeValidFlag | (static_cast<uint8_t>(direction) << routeShift)
        );
    }

    bool getNeighbour(uint8_t fromRow,
                      uint8_t fromCol,
                      Direction direction,
                      uint8_t& row,
                      uint8_t& col) const {
        int nextRow = fromRow;
        int nextCol = fromCol;

        switch (direction) {
            case NORTH: nextRow--; break;
            case EAST:  nextCol++; break;
            case SOUTH: nextRow++; break;
            case WEST:  nextCol--; break;
        }

        if (!positionInBounds(nextRow, nextCol)) {
            return false;
        }

        row = static_cast<uint8_t>(nextRow);
        col = static_cast<uint8_t>(nextCol);
        return true;
    }

    void setCellWall(uint8_t row,
                     uint8_t col,
                     Direction direction,
                     bool wall) {
        Cell& cell = maze[row][col];
        const uint8_t bit = directionBit(direction);
        const uint8_t oldData = cell.data;
        const uint8_t oldKnown = cell.known;

        cell.known |= bit;
        if (wall) {
            cell.data |= bit;
        } else {
            cell.data &= static_cast<uint8_t>(~bit);
        }

        if (cell.data != oldData || cell.known != oldKnown) {
            displayDirty = true;
        }
    }

    void setWall(Direction direction, bool wall) {
        uint8_t neighbourRow = 0;
        uint8_t neighbourCol = 0;
        if (!getNeighbour(currentRow, currentCol, direction,
                          neighbourRow, neighbourCol)) {
            setCellWall(currentRow, currentCol, direction, true);
            return;
        }

        setCellWall(currentRow, currentCol, direction, wall);
        setCellWall(neighbourRow, neighbourCol, opposite(direction), wall);
    }

    bool wallIsKnown(Direction direction) const {
        return (maze[currentRow][currentCol].known & directionBit(direction)) != 0;
    }

    bool canTraverse(uint8_t row,
                     uint8_t col,
                     Direction direction) const {
        const uint8_t bit = directionBit(direction);
        const Cell& cell = maze[row][col];
        if ((cell.known & bit) == 0 || (cell.data & wallMask & bit) != 0) {
            return false;
        }

        uint8_t neighbourRow = 0;
        uint8_t neighbourCol = 0;
        return getNeighbour(row, col, direction, neighbourRow, neighbourCol);
    }

    bool canMove(Direction direction) const {
        return canTraverse(currentRow, currentCol, direction);
    }

    bool neighbourIsUnvisited(Direction direction) const {
        uint8_t row = 0;
        uint8_t col = 0;
        return getNeighbour(currentRow, currentCol, direction, row, col) &&
               !isVisited(row, col);
    }

    bool hasUnknownDirection() const {
        return (maze[currentRow][currentCol].known & wallMask) != wallMask;
    }

    void observeCell() {
        bool frontWall = false;
        bool leftWall = false;
        bool rightWall = false;
        const bool frontValid = robot.senseFrontWall(wallThresholdMm, frontWall);
        const bool leftValid = robot.senseLeftWall(wallThresholdMm, leftWall);
        const bool rightValid = robot.senseRightWall(wallThresholdMm, rightWall);

        const Direction frontDirection = currentDirection;
        const Direction leftDirection = rotateLeft(currentDirection);
        const Direction rightDirection = rotateRight(currentDirection);

        if (frontValid) setWall(frontDirection, frontWall);
        if (leftValid) setWall(leftDirection, leftWall);
        if (rightValid) setWall(rightDirection, rightWall);

        const bool visibleDirectionsKnown =
            wallIsKnown(frontDirection) &&
            wallIsKnown(leftDirection) &&
            wallIsKnown(rightDirection);

        if (!visibleDirectionsKnown &&
            sensingAttempts + 1 < maxSensingAttempts) {
            sensingAttempts++;
            enterState(SENSING, retryDelayMs);
            return;
        }

        sensingAttempts = 0;
        enterState(DECIDING, 0);
    }

    void chooseNextMove() {
        if (phase == SHORTEST_RUN) {
            chooseShortestMove();
            return;
        }

        if (currentRow == goalRow && currentCol == goalCol) {
            goalReached = true;
            displayDirty = true;
        }

        // Left/front/right/back DFS preference. The two-bit route field stores
        // each new cell's parent until exploration has returned to the root.
        const Direction options[4] = {
            rotateLeft(currentDirection),
            currentDirection,
            rotateRight(currentDirection),
            opposite(currentDirection)
        };

        for (uint8_t index = 0; index < 4; index++) {
            const Direction direction = options[index];
            if (canMove(direction) && neighbourIsUnvisited(direction)) {
                beginMove(direction);
                return;
            }
        }

        if (hasUnknownDirection()) {
            finish(SENSOR_FAILURE);
            return;
        }

        const Cell& currentCell = maze[currentRow][currentCol];
        if (hasRoute(currentCell)) {
            beginMove(getRoute(currentCell));
            return;
        }

        // A DFS reaches this point only after it has backtracked to its root.
        if (currentRow != startRow || currentCol != startCol) {
            finish(NO_PATH_TO_GOAL);
            return;
        }
        buildShortestRoutes();
    }

    void buildShortestRoutes() {
        if (!goalReached) {
            finish(NO_PATH_TO_GOAL);
            return;
        }

        // Reuse the old DFS-parent bits for the shortest direction. The BFS
        // queue is temporary stack memory, so shortest-path storage costs no
        // additional persistent SRAM.
        for (uint8_t row = 0; row < mazeSize; row++) {
            for (uint8_t col = 0; col < mazeSize; col++) {
                clearRoute(maze[row][col]);
                maze[row][col].known &= static_cast<uint8_t>(~bfsVisitedFlag);
            }
        }

        uint8_t queue[mazeSize * mazeSize];
        uint8_t head = 0;
        uint8_t tail = 0;
        queue[tail++] = static_cast<uint8_t>(goalRow * mazeSize + goalCol);
        maze[goalRow][goalCol].known |= bfsVisitedFlag;

        while (head < tail) {
            const uint8_t index = queue[head++];
            const uint8_t row = index / mazeSize;
            const uint8_t col = index % mazeSize;

            for (uint8_t value = 0; value < 4; value++) {
                const Direction direction = static_cast<Direction>(value);
                uint8_t neighbourRow = 0;
                uint8_t neighbourCol = 0;
                if (!canTraverse(row, col, direction) ||
                    !getNeighbour(row, col, direction,
                                  neighbourRow, neighbourCol) ||
                    !isVisited(neighbourRow, neighbourCol) ||
                    (maze[neighbourRow][neighbourCol].known &
                     bfsVisitedFlag) != 0) {
                    continue;
                }

                maze[neighbourRow][neighbourCol].known |= bfsVisitedFlag;
                setRoute(maze[neighbourRow][neighbourCol],
                         opposite(direction));
                queue[tail++] = static_cast<uint8_t>(
                    neighbourRow * mazeSize + neighbourCol
                );
            }
        }

        for (uint8_t row = 0; row < mazeSize; row++) {
            for (uint8_t col = 0; col < mazeSize; col++) {
                maze[row][col].known &= static_cast<uint8_t>(~bfsVisitedFlag);
            }
        }

        if (currentRow == goalRow && currentCol == goalCol) {
            phase = SHORTEST_RUN;
            finish(GOAL_REACHED);
            return;
        }

        if (!hasRoute(maze[currentRow][currentCol])) {
            finish(NO_PATH_TO_GOAL);
            return;
        }

        phase = SHORTEST_RUN;
        displayDirty = true;
        enterState(DECIDING, arrivalSettleTimeMs);
    }

    void chooseShortestMove() {
        if (currentRow == goalRow && currentCol == goalCol) {
            finish(GOAL_REACHED);
            return;
        }

        const Cell& cell = maze[currentRow][currentCol];
        if (!hasRoute(cell)) {
            finish(NO_PATH_TO_GOAL);
            return;
        }
        beginMove(getRoute(cell));
    }

    int16_t getTurnAmount(Direction from, Direction to) const {
        const uint8_t difference = static_cast<uint8_t>(
            (static_cast<uint8_t>(to) - static_cast<uint8_t>(from) + 4) % 4
        );
        if (difference == 1) return -90;
        if (difference == 2) return 180;
        if (difference == 3) return 90;
        return 0;
    }

    void beginMove(Direction direction) {
        if (!canMove(direction) ||
            !getNeighbour(currentRow, currentCol, direction,
                          targetRow, targetCol)) {
            finish(SENSOR_FAILURE);
            return;
        }

        targetDirection = direction;
        verificationAttempts = 0;
        if (targetDirection == currentDirection) {
            enterState(VERIFYING_MOVE, 0);
            return;
        }

        robot.startTurn(static_cast<float>(
            getTurnAmount(currentDirection, targetDirection)
        ));
        enterState(TURNING, 0);
    }

    void updateTurn() {
        robot.update();
        if (millis() - stateStartedAtMs > turnTimeoutMs) {
            robot.stop();
            finish(TURN_TIMEOUT);
            return;
        }
        if (!robot.isFinished()) return;

        currentDirection = targetDirection;
        displayDirty = true;
        enterState(VERIFYING_MOVE, turnSettleTimeMs);
    }

    void verifyForwardPath() {
        bool frontWall = false;
        if (!robot.senseFrontWall(wallThresholdMm, frontWall)) {
            if (verificationAttempts + 1 < maxSensingAttempts) {
                verificationAttempts++;
                enterState(VERIFYING_MOVE, retryDelayMs);
            } else {
                finish(SENSOR_FAILURE);
            }
            return;
        }

        verificationAttempts = 0;
        setWall(targetDirection, frontWall);
        if (frontWall) {
            if (phase == SHORTEST_RUN) {
                buildShortestRoutes();
            } else {
                enterState(DECIDING, retryDelayMs);
            }
            return;
        }

        robot.startStraightLine(180.0f, forwardPwm, movingFrontStopMm);
        enterState(MOVING, 0);
    }

    void updateMovement() {
        robot.update();
        if (millis() - stateStartedAtMs > movementTimeoutMs) {
            robot.stop();
            finish(DRIVE_TIMEOUT);
            return;
        }
        if (!robot.isFinished()) return;

        const bool stoppedForWall = robot.stoppedForFrontWall();
        if (stoppedForWall &&
            robot.getTravelledDistanceMM() < 135.0f) {
            finish(UNEXPECTED_FRONT_WALL);
            return;
        }

        currentRow = targetRow;
        currentCol = targetCol;
        if (!isVisited(currentRow, currentCol)) {
            markVisited(currentRow, currentCol,
                        opposite(targetDirection), true);
        }

        if (stoppedForWall) {
            // The emergency reading occurred after entering the target cell;
            // it therefore describes the target cell's forward wall.
            setWall(currentDirection, true);
        }

        if (currentRow == goalRow && currentCol == goalCol) {
            goalReached = true;
        }
        displayDirty = true;

        if (phase == SHORTEST_RUN) {
            if (currentRow == goalRow && currentCol == goalCol) {
                finish(GOAL_REACHED);
            } else {
                enterState(DECIDING, arrivalSettleTimeMs);
            }
        } else {
            sensingAttempts = 0;
            enterState(WAITING_TO_SENSE, arrivalSettleTimeMs);
        }
    }

    bool mapPixelIsSet(uint8_t x, uint8_t y) const {
        const bool verticalGrid = (x % 7) == 0;
        const bool horizontalGrid = (y % 7) == 0;
        if (verticalGrid && horizontalGrid) return true;

        if (horizontalGrid) {
            const uint8_t boundary = y / 7;
            if (boundary == 0 || boundary == mazeSize) return true;
            const uint8_t col = x / 7;
            const Cell& cell = maze[boundary - 1][col];
            const uint8_t bit = directionBit(SOUTH);
            return (cell.known & bit) != 0 && (cell.data & bit) != 0;
        }

        if (verticalGrid) {
            const uint8_t boundary = x / 7;
            if (boundary == 0 || boundary == mazeSize) return true;
            const uint8_t row = y / 7;
            const Cell& cell = maze[row][boundary - 1];
            const uint8_t bit = directionBit(EAST);
            return (cell.known & bit) != 0 && (cell.data & bit) != 0;
        }

        const uint8_t row = y / 7;
        const uint8_t col = x / 7;
        const uint8_t localX = x % 7;
        const uint8_t localY = y % 7;
        if (row == currentRow && col == currentCol) {
            return (localX == 3 && localY >= 2 && localY <= 4) ||
                   (localY == 3 && localX >= 2 && localX <= 4);
        }
        if (row == goalRow && col == goalCol) {
            return (localX == 2 || localX == 4) &&
                   (localY == 2 || localY == 4);
        }
        return isVisited(row, col) && localX == 3 && localY == 3;
    }

    void finish(CompletionReason reason) {
        robot.stop();
        completionReason = reason;
        state = (reason == GOAL_REACHED) ? FINISHED : ERROR_STATE;
        displayDirty = true;
    }
};

}  // namespace mtrn3100
