#pragma once

// AI-assisted implementation: autonomous maze exploration, shortest-path
// planning, and framebuffer-free OLED map rendering for MTRN3100 Task 4.3.

#include <Arduino.h>
#include <U8x8lib.h>

#include "Lidar.hpp"
#include "Robot.hpp"

// The existing hardware objects live in the sketch's global namespace.  The
// one-argument constructor keeps main.cpp unchanged; the explicit-sensor
// overload below is available for tests or differently named hardware.
extern mtrn3100::Lidar front_lidar;
extern mtrn3100::Lidar left_lidar;
extern mtrn3100::Lidar right_lidar;

namespace mtrn3100 {

class AutonomousMapping {
public:
    enum Direction : uint8_t {
        NORTH = 0,
        EAST  = 1,
        SOUTH = 2,
        WEST  = 3
    };

    enum CompletionReason : uint8_t {
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

    // When E7 is shown, the digit after the decimal identifies the exact
    // planner check which failed.  Keeping this separate preserves the public
    // CompletionReason numbers used by the OLED and any existing tests.
    enum NoPathDetail : uint8_t {
        NO_PATH_DETAIL_NONE,
        START_CELL_HAS_PARENT,
        DFS_PARENT_MISSING,
        GOAL_NOT_VISITED,
        RETURN_POSE_MISMATCH,
        SHORTEST_ROUTE_BUILD_FAILED,
        SHORTEST_ROUTE_MISSING,
        PLANNED_EDGE_BLOCKED,
        NEIGHBOUR_OUT_OF_BOUNDS
    };

    explicit AutonomousMapping(Robot& robot)
        : AutonomousMapping(
              robot,
              ::front_lidar,
              ::left_lidar,
              ::right_lidar
          ) {}

    AutonomousMapping(Robot& robot,
                      Lidar& frontLidar,
                      Lidar& leftLidar,
                      Lidar& rightLidar)
        : robot(robot),
          frontLidar(frontLidar),
          leftLidar(leftLidar),
          rightLidar(rightLidar) {}

    void begin(uint8_t startRow,
               uint8_t startCol,
               Direction startDirection,
               uint8_t goalRow,
               uint8_t goalCol) {
        robot.stop();
        robot.enableEmergencyProtection(
            true,
            emergencyFrontDistanceMm,
            emergencySideDistanceMm
        );
        clearMap();

        if (!positionInBounds(startRow, startCol) ||
            !positionInBounds(goalRow, goalCol) ||
            !directionIsValid(startDirection)) {
            finish(INVALID_START_OR_GOAL);
            return;
        }

        this->startRow = startRow;
        this->startCol = startCol;
        this->startDirection = startDirection;
        this->goalRow = goalRow;
        this->goalCol = goalCol;

        currentRow = startRow;
        currentCol = startCol;
        currentDirection = startDirection;
        targetDirection = startDirection;

        visitedCount = 0;
        phase = EXPLORING;
        state = ERROR_STATE;
        turnPurpose = MOVE_TURN;
        clearSourceRouteAfterMove = false;
        completionReason = NOT_FINISHED;
        completionDetail = NO_PATH_DETAIL_NONE;
        displayDirty = true;

        markVisited(currentRow, currentCol, NORTH, false);
        beginSensing(sensorSettleTimeMs);
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
        return isVisited(goalRow, goalCol);
    }

    uint8_t getVisitedCount() const {
        return visitedCount;
    }

    CompletionReason getCompletionReason() const {
        return completionReason;
    }

    uint8_t getCompletionDetail() const {
        return completionDetail;
    }

    bool displayNeedsUpdate() const {
        return displayDirty;
    }

    // U8x8 writes one 8 x 8 tile directly to the OLED.  The visualization
    // therefore uses an eight-byte scratch tile and no 1024-byte framebuffer.
    void renderToOled(U8X8& oled) {
        uint8_t tile[8];

        for (uint8_t tileY = 0; tileY < 8; tileY++) {
            for (uint8_t tileX = 0; tileX < 8; tileX++) {
                for (uint8_t index = 0; index < 8; index++) {
                    tile[index] = 0;
                }

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

        // Column 8 separates the 64-pixel map from its compact status glyphs.
        for (uint8_t index = 0; index < 8; index++) {
            tile[index] = 0;
        }
        for (uint8_t tileY = 0; tileY < 8; tileY++) {
            oled.drawTile(8, tileY, 1, tile);
        }

        // Fixed-position glyphs avoid RAM-backed strings and number buffers.
        for (uint8_t row = 0; row < 8; row++) {
            for (uint8_t col = 9; col < 16; col++) {
                oled.drawGlyph(col, row, ' ');
            }
        }
        oled.drawGlyph(9, 0, phaseGlyph());

        const uint8_t percentage = static_cast<uint8_t>(
            (static_cast<uint16_t>(visitedCount) * 100U) / mazeCellCount
        );
        oled.drawGlyph(9, 2, 'P');
        drawThreeDigits(oled, 10, 2, percentage);
        oled.drawGlyph(13, 2, '%');

        oled.drawGlyph(9, 4, 'V');
        drawThreeDigits(oled, 10, 4, visitedCount);

        if (state == ERROR_STATE) {
            oled.drawGlyph(9, 6, 'E');
            oled.drawGlyph(
                10,
                6,
                static_cast<uint8_t>('0' + completionReason)
            );
            if (completionDetail != 0) {
                oled.drawGlyph(11, 6, '.');
                oled.drawGlyph(
                    12,
                    6,
                    static_cast<uint8_t>('0' + completionDetail)
                );
            }
        }

        displayDirty = false;
    }

private:
    // The maze occupies a 9 x 9 coordinate grid, with a three-cell triangular
    // cut-out at each corner: row widths are 5, 7, 9, 9, 9, 9, 9, 7, 5.
    static constexpr uint8_t mazeRows = 9;
    static constexpr uint8_t mazeColumns = 9;
    static constexpr uint8_t mazeCellCount = 69;
    static constexpr uint8_t cellPixels = 7;

    static constexpr uint8_t visitedFlag = 0x10;
    static constexpr uint8_t routeValidFlag = 0x20;
    static constexpr uint8_t routeShift = 6;
    static constexpr uint8_t routeMask = 0xC0;
    static constexpr uint8_t bfsVisitedFlag = 0x10;

    static constexpr uint8_t samplesPerWall = 3;
    static constexpr uint8_t maxSensingPasses = 7;
    static constexpr uint16_t wallThresholdMm = 130;
    static constexpr uint16_t emergencyFrontDistanceMm = 45;
    static constexpr uint16_t emergencySideDistanceMm = 25;
    static constexpr int16_t forwardPwm = 130;
    static constexpr float minimumFrontArrivalDistanceMm = 135.0f;
    static constexpr unsigned long sensorSettleTimeMs = 120;
    static constexpr unsigned long sensorRetryDelayMs = 40;
    static constexpr unsigned long arrivalSettleTimeMs = 150;
    static constexpr unsigned long turnTimeoutMs = 5000;
    static constexpr unsigned long movementTimeoutMs = 6000;

    enum Phase : uint8_t {
        EXPLORING,
        RETURNING_TO_START,
        SHORTEST_RUN
    };

    enum State : uint8_t {
        SENSING,
        DECIDING,
        TURNING,
        VERIFYING_MOVE,
        MOVING,
        FINISHED,
        ERROR_STATE
    };

    enum TurnPurpose : uint8_t {
        MOVE_TURN,
        SCAN_TURN,
        START_ALIGNMENT
    };

    // Four wall bits, visited, route-valid, and a two-bit route direction are
    // packed into data.  known holds four wall-known bits plus one temporary
    // BFS bit, so the complete persistent maze costs 162 bytes.
    struct Cell {
        uint8_t data;
        uint8_t known;
    };

    Robot& robot;
    Lidar& frontLidar;
    Lidar& leftLidar;
    Lidar& rightLidar;
    Cell maze[mazeRows][mazeColumns];

    uint8_t startRow = 0;
    uint8_t startCol = 0;
    uint8_t goalRow = 0;
    uint8_t goalCol = 0;
    uint8_t currentRow = 0;
    uint8_t currentCol = 0;

    Direction startDirection = NORTH;
    Direction currentDirection = NORTH;
    Direction targetDirection = NORTH;
    Phase phase = EXPLORING;
    State state = ERROR_STATE;
    TurnPurpose turnPurpose = MOVE_TURN;
    CompletionReason completionReason = NOT_FINISHED;
    uint8_t completionDetail = NO_PATH_DETAIL_NONE;

    uint8_t visitedCount = 0;
    uint8_t senseMask = 0;
    uint8_t sampleCounts = 0;
    uint8_t wallVotes = 0;
    uint8_t sensingPasses = 0;
    uint8_t forwardSampleCount = 0;
    uint8_t forwardWallVotes = 0;
    uint8_t forwardSensingPasses = 0;
    bool displayDirty = true;
    bool clearSourceRouteAfterMove = false;

    // Robot::startCommandString stores this pointer until the command ends,
    // so the one-cell command must have object lifetime rather than stack
    // lifetime.  This selects the exact Task 1 maze-driving control path.
    const char forwardCommand[2] = {'f', '\0'};

    unsigned long stateStartedAtMs = 0;
    unsigned long stateReadyAtMs = 0;

    static uint8_t directionBit(Direction direction) {
        return static_cast<uint8_t>(1U << static_cast<uint8_t>(direction));
    }

    static bool directionIsValid(Direction direction) {
        return static_cast<uint8_t>(direction) <= static_cast<uint8_t>(WEST);
    }

    static Direction rotateLeft(Direction direction) {
        return static_cast<Direction>(
            (static_cast<uint8_t>(direction) + 3U) % 4U
        );
    }

    static Direction rotateRight(Direction direction) {
        return static_cast<Direction>(
            (static_cast<uint8_t>(direction) + 1U) % 4U
        );
    }

    static Direction opposite(Direction direction) {
        return static_cast<Direction>(
            (static_cast<uint8_t>(direction) + 2U) % 4U
        );
    }

    static bool timeReached(unsigned long now, unsigned long target) {
        return static_cast<long>(now - target) >= 0;
    }

    void enterState(State newState, unsigned long waitMs) {
        state = newState;
        stateStartedAtMs = millis();
        stateReadyAtMs = stateStartedAtMs + waitMs;
    }

    static bool positionInArray(int row, int col) {
        return row >= 0 && row < mazeRows &&
               col >= 0 && col < mazeColumns;
    }

    static bool cellIsActive(int row, int col) {
        if (!positionInArray(row, col)) {
            return false;
        }

        const uint8_t edgeInset =
            (row == 0 || row == mazeRows - 1) ? 2U :
            (row == 1 || row == mazeRows - 2) ? 1U : 0U;
        return col >= edgeInset && col < mazeColumns - edgeInset;
    }

    bool positionInBounds(int row, int col) const {
        return cellIsActive(row, col);
    }

    void clearMap() {
        for (uint8_t row = 0; row < mazeRows; row++) {
            for (uint8_t col = 0; col < mazeColumns; col++) {
                maze[row][col].data = 0;
                maze[row][col].known = 0;
            }
        }

        // Every edge from a usable cell to an omitted corner or outside the
        // 9 x 9 coordinate grid is a known perimeter wall. This forms the
        // stepped outline without storing a separate 81-byte active-cell mask.
        for (uint8_t row = 0; row < mazeRows; row++) {
            for (uint8_t col = 0; col < mazeColumns; col++) {
                if (!cellIsActive(row, col)) {
                    continue;
                }
                if (!cellIsActive(static_cast<int>(row) - 1, col)) {
                    setCellWall(row, col, NORTH, true);
                }
                if (!cellIsActive(row, static_cast<int>(col) + 1)) {
                    setCellWall(row, col, EAST, true);
                }
                if (!cellIsActive(static_cast<int>(row) + 1, col)) {
                    setCellWall(row, col, SOUTH, true);
                }
                if (!cellIsActive(row, static_cast<int>(col) - 1)) {
                    setCellWall(row, col, WEST, true);
                }
            }
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
        if (visitedCount < mazeCellCount) {
            visitedCount++;
        }
        displayDirty = true;
    }

    bool isVisited(uint8_t row, uint8_t col) const {
        return (maze[row][col].data & visitedFlag) != 0;
    }

    bool hasRoute(const Cell& cell) const {
        return (cell.data & routeValidFlag) != 0;
    }

    Direction getRoute(const Cell& cell) const {
        return static_cast<Direction>(
            (cell.data & routeMask) >> routeShift
        );
    }

    void clearRoute(Cell& cell) {
        cell.data &= static_cast<uint8_t>(
            ~(routeValidFlag | routeMask)
        );
    }

    void setRoute(Cell& cell, Direction direction) {
        clearRoute(cell);
        cell.data |= static_cast<uint8_t>(
            routeValidFlag |
            (static_cast<uint8_t>(direction) << routeShift)
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

    bool cellWallIsKnown(uint8_t row,
                         uint8_t col,
                         Direction direction) const {
        return (maze[row][col].known & directionBit(direction)) != 0;
    }

    bool cellHasWall(uint8_t row,
                     uint8_t col,
                     Direction direction) const {
        return (maze[row][col].data & directionBit(direction)) != 0;
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

    void setWall(Direction direction, bool sensedWall) {
        uint8_t neighbourRow = 0;
        uint8_t neighbourCol = 0;
        const bool hasNeighbour = getNeighbour(
            currentRow,
            currentCol,
            direction,
            neighbourRow,
            neighbourCol
        );

        if (!hasNeighbour) {
            setCellWall(currentRow, currentCol, direction, true);
            return;
        }

        const Direction reciprocal = opposite(direction);

        // AI-assisted consistency rule: confirmed walls are immutable.  Open
        // edges normally remain unchanged, but a stationary forward check may
        // safely upgrade open to wall before motion starts.
        if (cellWallIsKnown(currentRow, currentCol, direction)) {
            if (!cellWallIsKnown(neighbourRow, neighbourCol, reciprocal)) {
                setCellWall(
                    neighbourRow,
                    neighbourCol,
                    reciprocal,
                    cellHasWall(currentRow, currentCol, direction)
                );
            }
            return;
        }

        bool wall = sensedWall;
        if (cellWallIsKnown(neighbourRow, neighbourCol, reciprocal)) {
            wall = cellHasWall(neighbourRow, neighbourCol, reciprocal);
        }

        setCellWall(currentRow, currentCol, direction, wall);
        setCellWall(neighbourRow, neighbourCol, reciprocal, wall);
    }

    void confirmWall(Direction direction) {
        uint8_t neighbourRow = 0;
        uint8_t neighbourCol = 0;
        setCellWall(currentRow, currentCol, direction, true);
        if (getNeighbour(currentRow, currentCol, direction,
                         neighbourRow, neighbourCol)) {
            setCellWall(
                neighbourRow,
                neighbourCol,
                opposite(direction),
                true
            );
        }
    }

    bool wallIsKnown(Direction direction) const {
        return cellWallIsKnown(currentRow, currentCol, direction);
    }

    bool canTraverse(uint8_t row,
                     uint8_t col,
                     Direction direction) const {
        if (!cellWallIsKnown(row, col, direction) ||
            cellHasWall(row, col, direction)) {
            return false;
        }

        uint8_t neighbourRow = 0;
        uint8_t neighbourCol = 0;
        if (!getNeighbour(row, col, direction,
                          neighbourRow, neighbourCol)) {
            return false;
        }

        const Direction reciprocal = opposite(direction);
        return cellWallIsKnown(neighbourRow, neighbourCol, reciprocal) &&
               !cellHasWall(neighbourRow, neighbourCol, reciprocal);
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

    static uint8_t packedValue(uint8_t packed, Direction direction) {
        const uint8_t shift = static_cast<uint8_t>(direction) * 2U;
        return static_cast<uint8_t>((packed >> shift) & 0x03U);
    }

    static void incrementPacked(uint8_t& packed, Direction direction) {
        const uint8_t shift = static_cast<uint8_t>(direction) * 2U;
        packed = static_cast<uint8_t>(packed + (1U << shift));
    }

    void beginSensing(unsigned long waitMs) {
        senseMask = 0;
        sampleCounts = 0;
        wallVotes = 0;
        sensingPasses = 0;

        const Direction visible[3] = {
            currentDirection,
            rotateLeft(currentDirection),
            rotateRight(currentDirection)
        };
        for (uint8_t index = 0; index < 3; index++) {
            if (!wallIsKnown(visible[index])) {
                senseMask |= directionBit(visible[index]);
            }
        }

        enterState(SENSING, waitMs);
    }

    bool readWall(Lidar& lidar, bool& wall) {
        const uint16_t distanceMm = lidar.readDistance();
        if (lidar.isReadingValid()) {
            wall = distanceMm <= wallThresholdMm;
            return true;
        }

        if (!lidar.isReady() || lidar.timedOut()) {
            return false;
        }

        // The unmodified VL6180X driver rejects all non-zero range statuses.
        // Repeated no-target/overflow samples safely describe open space at a
        // 130 mm wall threshold; underflow describes a very close wall.
        switch (lidar.getRangeStatus()) {
            case 6:   // early convergence, no valid target detected
            case 7:   // maximum convergence, no valid target detected
            case 13:  // raw range overflow
            case 15:  // range overflow
                wall = false;
                return true;
            case 12:  // raw underflow, target extremely close
            case 14:  // underflow, target extremely close
                wall = true;
                return true;
            default:
                return false;
        }
    }

    void sampleWall(Direction direction, Lidar& lidar) {
        const uint8_t bit = directionBit(direction);
        if ((senseMask & bit) == 0 ||
            packedValue(sampleCounts, direction) >= samplesPerWall) {
            return;
        }

        bool wall = false;
        if (!readWall(lidar, wall)) {
            return;
        }

        incrementPacked(sampleCounts, direction);
        if (wall) {
            incrementPacked(wallVotes, direction);
        }
    }

    bool allSamplesReady() const {
        for (uint8_t value = 0; value < 4; value++) {
            const Direction direction = static_cast<Direction>(value);
            if ((senseMask & directionBit(direction)) != 0 &&
                packedValue(sampleCounts, direction) < samplesPerWall) {
                return false;
            }
        }
        return true;
    }

    void commitSamples() {
        for (uint8_t value = 0; value < 4; value++) {
            const Direction direction = static_cast<Direction>(value);
            if ((senseMask & directionBit(direction)) == 0) {
                continue;
            }

            setWall(
                direction,
                packedValue(wallVotes, direction) >= 2U
            );
        }
    }

    void observeCell() {
        if (senseMask == 0) {
            enterState(DECIDING, 0);
            return;
        }

        sampleWall(currentDirection, frontLidar);
        sampleWall(rotateLeft(currentDirection), leftLidar);
        sampleWall(rotateRight(currentDirection), rightLidar);
        sensingPasses++;

        if (allSamplesReady()) {
            commitSamples();
            enterState(DECIDING, 0);
            return;
        }

        if (sensingPasses >= maxSensingPasses) {
            finish(SENSOR_FAILURE);
            return;
        }

        stateReadyAtMs = millis() + sensorRetryDelayMs;
    }

    bool findUnknownDirection(Direction& direction) const {
        const Direction options[4] = {
            currentDirection,
            rotateLeft(currentDirection),
            rotateRight(currentDirection),
            opposite(currentDirection)
        };

        for (uint8_t index = 0; index < 4; index++) {
            if (!wallIsKnown(options[index])) {
                direction = options[index];
                return true;
            }
        }
        return false;
    }

    void chooseNextMove() {
        if (phase == SHORTEST_RUN) {
            chooseShortestMove();
            return;
        }

        // A newly entered cell already knows its rear edge.  At an arbitrary
        // interior start pose it does not, so turn and scan any remaining edge
        // before DFS is allowed to make a route decision.
        Direction unknownDirection = NORTH;
        if (findUnknownDirection(unknownDirection)) {
            beginTurn(unknownDirection, SCAN_TURN);
            return;
        }

        // AI-assisted DFS: each new cell stores only its parent direction in
        // two bits.  This explores every reachable cell and naturally returns
        // the physical robot to the root without a recursion stack.
        const Direction options[4] = {
            rotateLeft(currentDirection),
            currentDirection,
            rotateRight(currentDirection),
            opposite(currentDirection)
        };

        for (uint8_t index = 0; index < 4; index++) {
            const Direction direction = options[index];
            if (canMove(direction) && neighbourIsUnvisited(direction)) {
                beginMove(direction, false);
                return;
            }
        }

        Cell& currentCell = maze[currentRow][currentCol];
        if (hasRoute(currentCell)) {
            if (currentRow == startRow && currentCol == startCol) {
                finishNoPath(START_CELL_HAS_PARENT);
                return;
            }

            const Direction parentDirection = getRoute(currentCell);
            // Do not erase the DFS parent until the movement completes. A
            // front stop can interrupt the action before cell arrival; deleting
            // the parent early would strand the planner and produce E7.
            beginMove(parentDirection, true);
            return;
        }

        if (currentRow != startRow || currentCol != startCol) {
            finishNoPath(DFS_PARENT_MISSING);
            return;
        }

        prepareShortestRun();
    }

    void prepareShortestRun() {
        phase = RETURNING_TO_START;
        displayDirty = true;

        if (!hasReachedGoal()) {
            finishNoPath(GOAL_NOT_VISITED);
            return;
        }

        if (currentRow != startRow || currentCol != startCol) {
            finishNoPath(RETURN_POSE_MISMATCH);
            return;
        }

        if (currentDirection != startDirection) {
            beginTurn(startDirection, START_ALIGNMENT);
            return;
        }

        startShortestRun();
    }

    bool buildShortestRoutes() {
        // Reuse the DFS parent bits for BFS directions.  A linear scan over the
        // temporary visited bits avoids an 81-byte queue on the Nano's stack.
        for (uint8_t row = 0; row < mazeRows; row++) {
            for (uint8_t col = 0; col < mazeColumns; col++) {
                clearRoute(maze[row][col]);
                maze[row][col].known &= static_cast<uint8_t>(
                    ~bfsVisitedFlag
                );
            }
        }

        if (!isVisited(goalRow, goalCol)) {
            return false;
        }

        maze[goalRow][goalCol].known |= bfsVisitedFlag;

        bool changed;
        do {
            changed = false;
            for (uint8_t row = 0; row < mazeRows; row++) {
                for (uint8_t col = 0; col < mazeColumns; col++) {
                    if ((maze[row][col].known & bfsVisitedFlag) == 0) {
                        continue;
                    }

                    for (uint8_t value = 0; value < 4; value++) {
                        const Direction direction =
                            static_cast<Direction>(value);
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

                        maze[neighbourRow][neighbourCol].known |=
                            bfsVisitedFlag;
                        setRoute(
                            maze[neighbourRow][neighbourCol],
                            opposite(direction)
                        );
                        changed = true;
                    }
                }
            }
        } while (changed);

        for (uint8_t row = 0; row < mazeRows; row++) {
            for (uint8_t col = 0; col < mazeColumns; col++) {
                maze[row][col].known &= static_cast<uint8_t>(
                    ~bfsVisitedFlag
                );
            }
        }

        return currentRow == goalRow && currentCol == goalCol
            ? true
            : hasRoute(maze[currentRow][currentCol]);
    }

    void startShortestRun() {
        if (!buildShortestRoutes()) {
            finishNoPath(SHORTEST_ROUTE_BUILD_FAILED);
            return;
        }

        phase = SHORTEST_RUN;
        displayDirty = true;

        if (currentRow == goalRow && currentCol == goalCol) {
            finish(GOAL_REACHED);
            return;
        }

        enterState(DECIDING, arrivalSettleTimeMs);
    }

    void chooseShortestMove() {
        if (currentRow == goalRow && currentCol == goalCol) {
            finish(GOAL_REACHED);
            return;
        }

        const Cell& cell = maze[currentRow][currentCol];
        if (!hasRoute(cell)) {
            finishNoPath(SHORTEST_ROUTE_MISSING);
            return;
        }

        beginMove(getRoute(cell), false);
    }

    int16_t getTurnAmount(Direction from, Direction to) const {
        const uint8_t difference = static_cast<uint8_t>(
            (static_cast<uint8_t>(to) -
             static_cast<uint8_t>(from) + 4U) % 4U
        );

        // The finalized Task 1 controller uses positive yaw for left turns.
        if (difference == 1U) return -90;
        if (difference == 2U) return 180;
        if (difference == 3U) return 90;
        return 0;
    }

    void beginTurn(Direction direction, TurnPurpose purpose) {
        targetDirection = direction;
        turnPurpose = purpose;

        if (targetDirection == currentDirection) {
            completeTurnPurpose();
            return;
        }

        robot.startTurn(static_cast<float>(
            getTurnAmount(currentDirection, targetDirection)
        ));
        enterState(TURNING, 0);
    }

    void completeTurnPurpose() {
        switch (turnPurpose) {
            case MOVE_TURN:
                beginForwardVerification();
                break;
            case SCAN_TURN:
                beginSensing(sensorSettleTimeMs);
                break;
            case START_ALIGNMENT:
                startShortestRun();
                break;
        }
    }

    void beginMove(Direction direction, bool backtracking) {
        clearSourceRouteAfterMove = backtracking;

        uint8_t neighbourRow = 0;
        uint8_t neighbourCol = 0;
        if (!canMove(direction)) {
            finishNoPath(PLANNED_EDGE_BLOCKED);
            return;
        }
        if (!getNeighbour(currentRow, currentCol, direction,
                          neighbourRow, neighbourCol)) {
            finishNoPath(NEIGHBOUR_OUT_OF_BOUNDS);
            return;
        }

        targetDirection = direction;
        if (targetDirection == currentDirection) {
            beginForwardVerification();
            return;
        }

        beginTurn(targetDirection, MOVE_TURN);
    }

    void beginForwardVerification() {
        forwardSampleCount = 0;
        forwardWallVotes = 0;
        forwardSensingPasses = 0;
        enterState(VERIFYING_MOVE, sensorSettleTimeMs);
    }

    void verifyForwardPath() {
        bool wall = false;
        if (readWall(frontLidar, wall)) {
            forwardSampleCount++;
            if (wall) {
                forwardWallVotes++;
            }
        }
        forwardSensingPasses++;

        if (forwardSampleCount >= samplesPerWall) {
            if (forwardWallVotes >= 2U) {
                confirmWall(targetDirection);
                clearSourceRouteAfterMove = false;

                if (phase == SHORTEST_RUN) {
                    finish(UNEXPECTED_FRONT_WALL);
                } else {
                    enterState(DECIDING, sensorRetryDelayMs);
                }
                return;
            }

            startForwardMove();
            return;
        }

        if (forwardSensingPasses >= maxSensingPasses) {
            finish(SENSOR_FAILURE);
            return;
        }

        stateReadyAtMs = millis() + sensorRetryDelayMs;
    }

    void startForwardMove() {
        // Movement, IMU, LiDAR avoidance, and PID behavior remain wholly in
        // the finalized Task 1 Robot controller.  The "f" command invokes its
        // tested 180 mm approach slowdown and post-LiDAR encoder checks.
        startOneCell(robot, forwardCommand, forwardPwm, 0);
        enterState(MOVING, 0);
    }

    template <typename Controller>
    static auto startOneCell(Controller& controller,
                             const char* command,
                             int16_t pwm,
                             int)
        -> decltype(controller.startCommandString(command, pwm), void()) {
        controller.startCommandString(command, pwm);
    }

    // Compatibility for a stripped Task 3 Robot facade.  The overload above
    // is selected whenever the finalized Task 1 command API is present.
    template <typename Controller>
    static void startOneCell(Controller& controller,
                             const char*,
                             int16_t pwm,
                             long) {
        controller.startStraightLine(180.0f, pwm);
    }

    void updateTurn() {
        robot.update();
        if (robot.isFinished()) {
            currentDirection = targetDirection;
            displayDirty = true;
            completeTurnPurpose();
        } else if (millis() - stateStartedAtMs > turnTimeoutMs) {
            finish(TURN_TIMEOUT);
        }
    }

    void updateMovement() {
        robot.update();
        if (!robot.isFinished()) {
            if (millis() - stateStartedAtMs > movementTimeoutMs) {
                finish(DRIVE_TIMEOUT);
            }
            return;
        }

        const EmergencyStopReason emergencyReason =
            robot.getEmergencyStopReason();
        if (emergencyReason != EMERGENCY_NONE) {
            handleMovementEmergency(emergencyReason);
            return;
        }

        completeCellArrival(false);
    }

    void completeCellArrival(bool frontWallAhead) {
        uint8_t nextRow = 0;
        uint8_t nextCol = 0;
        if (!getNeighbour(currentRow, currentCol, targetDirection,
                          nextRow, nextCol)) {
            finishNoPath(NEIGHBOUR_OUT_OF_BOUNDS);
            return;
        }

        // A DFS parent belongs to the cell being left.  Clear it only after
        // the physical move has succeeded (or a late front stop has safely
        // completed that cell arrival).
        if (clearSourceRouteAfterMove) {
            clearRoute(maze[currentRow][currentCol]);
        }

        currentRow = nextRow;
        currentCol = nextCol;
        currentDirection = targetDirection;

        if (phase != SHORTEST_RUN && !isVisited(currentRow, currentCol)) {
            markVisited(
                currentRow,
                currentCol,
                opposite(currentDirection),
                true
            );
        }

        // A close front reading near the end of a cell move describes the far
        // wall of the cell just entered, not the open edge just traversed.
        if (frontWallAhead) {
            confirmWall(currentDirection);
        }

        clearSourceRouteAfterMove = false;

        displayDirty = true;

        if (phase == SHORTEST_RUN) {
            if (currentRow == goalRow && currentCol == goalCol) {
                finish(GOAL_REACHED);
            } else if (frontWallAhead) {
                // The run can continue through an alternate mapped route.
                startShortestRun();
            } else {
                enterState(DECIDING, arrivalSettleTimeMs);
            }
            return;
        }

        beginSensing(arrivalSettleTimeMs);
    }

    void handleMovementEmergency(EmergencyStopReason emergencyReason) {
        const float travelledDistanceMm = robot.getTravelledDistanceMM();

        if (emergencyReason == EMERGENCY_FRONT) {
            handleFrontEmergency(travelledDistanceMm);
        }
    }

    void handleFrontEmergency(float travelledDistanceMm) {
        // Per the requested behaviour, a front emergency never reverses.  A
        // stop late in the 180 mm action is treated as arrival at the next cell
        // followed by discovery of that cell's front wall.  An earlier stop
        // keeps the current logical cell and immediately replans a turn.
        if (travelledDistanceMm >= minimumFrontArrivalDistanceMm) {
            completeCellArrival(true);
            return;
        }

        confirmWall(targetDirection);
        clearSourceRouteAfterMove = false;

        if (phase == SHORTEST_RUN) {
            startShortestRun();
        } else {
            beginSensing(arrivalSettleTimeMs);
        }
    }

    bool robotPixelIsSet(uint8_t localX, uint8_t localY) const {
        switch (currentDirection) {
            case NORTH:
                return (localX == 3 && localY >= 1 && localY <= 4) ||
                       (localY == 2 && (localX == 2 || localX == 4));
            case EAST:
                return (localY == 3 && localX >= 2 && localX <= 5) ||
                       (localX == 4 && (localY == 2 || localY == 4));
            case SOUTH:
                return (localX == 3 && localY >= 2 && localY <= 5) ||
                       (localY == 4 && (localX == 2 || localX == 4));
            case WEST:
                return (localY == 3 && localX >= 1 && localX <= 4) ||
                       (localX == 2 && (localY == 2 || localY == 4));
        }
        return false;
    }

    bool horizontalWallSegmentIsSet(uint8_t boundary,
                                    uint8_t col) const {
        if (col >= mazeColumns || boundary > mazeRows) {
            return false;
        }

        const bool aboveActive = boundary > 0 &&
            cellIsActive(static_cast<int>(boundary) - 1, col);
        const bool belowActive = boundary < mazeRows &&
            cellIsActive(boundary, col);
        if (!aboveActive && !belowActive) {
            return false;
        }
        if (aboveActive != belowActive) {
            return true;
        }

        return (cellWallIsKnown(boundary - 1, col, SOUTH) &&
                cellHasWall(boundary - 1, col, SOUTH)) ||
               (cellWallIsKnown(boundary, col, NORTH) &&
                cellHasWall(boundary, col, NORTH));
    }

    bool verticalWallSegmentIsSet(uint8_t row,
                                  uint8_t boundary) const {
        if (row >= mazeRows || boundary > mazeColumns) {
            return false;
        }

        const bool leftActive = boundary > 0 &&
            cellIsActive(row, static_cast<int>(boundary) - 1);
        const bool rightActive = boundary < mazeColumns &&
            cellIsActive(row, boundary);
        if (!leftActive && !rightActive) {
            return false;
        }
        if (leftActive != rightActive) {
            return true;
        }

        return (cellWallIsKnown(row, boundary - 1, EAST) &&
                cellHasWall(row, boundary - 1, EAST)) ||
               (cellWallIsKnown(row, boundary, WEST) &&
                cellHasWall(row, boundary, WEST));
    }

    bool cornerConnectsToWall(uint8_t boundaryRow,
                              uint8_t boundaryCol) const {
        // A grid intersection is visible only when at least one real wall ends
        // there. This removes the unused dot at every empty cell corner while
        // keeping wall endpoints and junctions visually connected.
        return (boundaryCol > 0 && horizontalWallSegmentIsSet(
                    boundaryRow,
                    boundaryCol - 1
                )) ||
               (boundaryCol < mazeColumns && horizontalWallSegmentIsSet(
                    boundaryRow,
                    boundaryCol
                )) ||
               (boundaryRow > 0 && verticalWallSegmentIsSet(
                    boundaryRow - 1,
                    boundaryCol
                )) ||
               (boundaryRow < mazeRows && verticalWallSegmentIsSet(
                    boundaryRow,
                    boundaryCol
                ));
    }

    bool mapPixelIsSet(uint8_t x, uint8_t y) const {
        const bool verticalGrid = (x % cellPixels) == 0;
        const bool horizontalGrid = (y % cellPixels) == 0;
        if (verticalGrid && horizontalGrid) {
            return cornerConnectsToWall(
                y / cellPixels,
                x / cellPixels
            );
        }

        if (horizontalGrid) {
            const uint8_t boundary = y / cellPixels;
            const uint8_t col = x / cellPixels;
            if (horizontalWallSegmentIsSet(boundary, col)) {
                return true;
            }

            // A midpoint tick denotes an unknown edge only between two usable
            // cells. Missing-corner space contains no decorative grid marks.
            if (boundary > 0 && boundary < mazeRows &&
                cellIsActive(boundary - 1, col) &&
                cellIsActive(boundary, col) &&
                !cellWallIsKnown(boundary - 1, col, SOUTH) &&
                !cellWallIsKnown(boundary, col, NORTH)) {
                return (x % cellPixels) == 3;
            }
            return false;
        }

        if (verticalGrid) {
            const uint8_t boundary = x / cellPixels;
            const uint8_t row = y / cellPixels;
            if (verticalWallSegmentIsSet(row, boundary)) {
                return true;
            }

            if (boundary > 0 && boundary < mazeColumns &&
                cellIsActive(row, boundary - 1) &&
                cellIsActive(row, boundary) &&
                !cellWallIsKnown(row, boundary - 1, EAST) &&
                !cellWallIsKnown(row, boundary, WEST)) {
                return (y % cellPixels) == 3;
            }
            return false;
        }

        const uint8_t row = y / cellPixels;
        const uint8_t col = x / cellPixels;
        const uint8_t localX = x % cellPixels;
        const uint8_t localY = y % cellPixels;

        if (!cellIsActive(row, col)) {
            return false;
        }

        if (row == currentRow && col == currentCol) {
            return robotPixelIsSet(localX, localY);
        }
        if (row == goalRow && col == goalCol) {
            if (isVisited(row, col)) {
                // Uppercase G means the goal cell has been visited.
                return ((localY == 1 || localY == 5) &&
                        localX >= 2 && localX <= 4) ||
                       (localX == 1 && localY >= 2 && localY <= 4) ||
                       (localY == 3 && localX >= 3 && localX <= 4) ||
                       (localX == 4 && localY >= 3 && localY <= 4);
            }

            // Smaller lowercase g, including a descender, means unvisited.
            return ((localY == 2 || localY == 4 || localY == 6) &&
                    localX >= 2 && localX <= 4) ||
                   (localY == 3 && (localX == 2 || localX == 4)) ||
                   (localY == 5 && localX == 4);
        }
        if (row == startRow && col == startCol) {
            if (isVisited(row, col)) {
                // Uppercase S means the start cell has been visited.
                return ((localY == 1 || localY == 3 || localY == 5) &&
                        localX >= 1 && localX <= 4) ||
                       (localX == 1 && localY == 2) ||
                       (localX == 4 && localY == 4);
            }

            // Smaller lowercase s means unvisited.
            return ((localY == 1 || localY == 3 || localY == 5) &&
                    localX >= 2 && localX <= 4) ||
                   (localX == 2 && localY == 2) ||
                   (localX == 4 && localY == 4);
        }

        if (!isVisited(row, col)) {
            // Procedural 3 x 5 question mark: unvisited cells are unmistakable
            // without storing a glyph, string, framebuffer, or per-cell state.
            return (localY == 1 && localX >= 2 && localX <= 4) ||
                   (localX == 4 && localY == 2) ||
                   (localX == 3 && localY == 3) ||
                   (localX == 3 && localY == 5);
        }

        // A single centre pixel is the compact visited-cell marker.
        return localX == 3 && localY == 3;
    }

    uint8_t phaseGlyph() const {
        if (state == ERROR_STATE) return '!';
        if (state == FINISHED) return 'D';
        if (phase == RETURNING_TO_START) return 'R';
        if (phase == SHORTEST_RUN) return 'S';
        return 'M';
    }

    static void drawThreeDigits(U8X8& oled,
                                uint8_t x,
                                uint8_t row,
                                uint8_t value) {
        oled.drawGlyph(
            x,
            row,
            value >= 100 ? static_cast<uint8_t>('0' + value / 100) : ' '
        );
        oled.drawGlyph(
            x + 1,
            row,
            value >= 10
                ? static_cast<uint8_t>('0' + (value / 10) % 10)
                : ' '
        );
        oled.drawGlyph(
            x + 2,
            row,
            static_cast<uint8_t>('0' + value % 10)
        );
    }

    void finishNoPath(NoPathDetail detail) {
        finish(NO_PATH_TO_GOAL, static_cast<uint8_t>(detail));
    }

    void finish(CompletionReason reason, uint8_t detail = 0) {
        robot.stop();
        completionReason = reason;
        completionDetail = detail;
        state = reason == GOAL_REACHED ? FINISHED : ERROR_STATE;
        displayDirty = true;
    }
};

}  // namespace mtrn3100
