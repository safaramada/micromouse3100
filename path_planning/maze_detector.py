import cv2
import numpy as np
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parent
IMAGE_PATH = PROJECT_DIR / "images" / "maze.jpg"
OUTPUT_DIR = PROJECT_DIR / "outputs"


def load_image():
    image = cv2.imread(str(IMAGE_PATH))

    if image is None:
        print("Error: maze image could not be loaded")
        return None

    return image


def convert_to_grayscale(image):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return gray_image


def display_image(window_name, image):
    cv2.imshow(window_name, image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def save_grayscale_image(gray_image):
    OUTPUT_DIR.mkdir(exist_ok=True)

    output_path = OUTPUT_DIR / "grayscale_maze.jpg"
    cv2.imwrite(str(output_path), gray_image)

    print("Grayscale image saved")

def apply_threshold(gray_image):
    # Convert the grayscale image into black and white.
    # Otsu's method automatically chooses the threshold value.
    threshold_value, binary_image = cv2.threshold(
        gray_image,
        0,
        255,
        cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )

    print("Threshold value:", threshold_value)

    return binary_image


def save_threshold_image(binary_image):
    OUTPUT_DIR.mkdir(exist_ok=True)

    output_path = OUTPUT_DIR / "threshold_maze.jpg"
    saved = cv2.imwrite(str(output_path), binary_image)

    if saved:
        print(f"Threshold image saved to: {output_path}")
    else:
        print("Error: threshold image was not saved")

def find_maze_boundary(binary_image):
    # Find all external contours in the threshold image
    contours, hierarchy = cv2.findContours(
        binary_image,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if len(contours) == 0:
        print("Error: no contours were found")
        return None

    # Assume the largest contour is the outside of the maze
    largest_contour = max(contours, key=cv2.contourArea)

    return largest_contour


def draw_maze_boundary(original_image, maze_contour):
    # Make a copy so the original image is not changed
    boundary_image = original_image.copy()

    # Draw the contour in green
    cv2.drawContours(
        boundary_image,
        [maze_contour],
        -1,
        (0, 255, 0),
        5
    )

    return boundary_image


def save_boundary_image(boundary_image):
    OUTPUT_DIR.mkdir(exist_ok=True)

    output_path = OUTPUT_DIR / "maze_boundary.jpg"
    saved = cv2.imwrite(str(output_path), boundary_image)

    if saved:
        print(f"Boundary image saved to: {output_path}")
    else:
        print("Error: boundary image was not saved")

def get_maze_corners(maze_contour):
    # Estimate the outside contour as a four-corner shape
    perimeter = cv2.arcLength(maze_contour, True)

    corners = cv2.approxPolyDP(
        maze_contour,
        0.02 * perimeter,
        True
    )

    if len(corners) != 4:
        print("Error: maze boundary does not have four corners")
        return None

    # Convert from OpenCV contour format into four x-y points
    corners = corners.reshape(4, 2)

    return corners


def order_corner_points(corners):
    # Create space for the ordered corner points
    ordered = np.zeros((4, 2), dtype=np.float32)

    # x + y is smallest at the top-left
    # x + y is largest at the bottom-right
    point_sum = corners.sum(axis=1)

    ordered[0] = corners[np.argmin(point_sum)]
    ordered[2] = corners[np.argmax(point_sum)]

    # y - x is smallest at the top-right
    # y - x is largest at the bottom-left
    point_difference = np.diff(corners, axis=1).flatten()

    ordered[1] = corners[np.argmin(point_difference)]
    ordered[3] = corners[np.argmax(point_difference)]

    return ordered


def warp_maze_image(original_image, corners):
    ordered_corners = order_corner_points(corners)

    top_left = ordered_corners[0]
    top_right = ordered_corners[1]
    bottom_right = ordered_corners[2]
    bottom_left = ordered_corners[3]

    # Use a fixed square output size
    output_size = 900

    destination_points = np.array(
        [
            [0, 0],
            [output_size - 1, 0],
            [output_size - 1, output_size - 1],
            [0, output_size - 1]
        ],
        dtype=np.float32
    )

    transform_matrix = cv2.getPerspectiveTransform(
        ordered_corners,
        destination_points
    )

    warped_image = cv2.warpPerspective(
        original_image,
        transform_matrix,
        (output_size, output_size)
    )

    return warped_image


def draw_corner_points(original_image, corners):
    corner_image = original_image.copy()

    for point in corners:
        x, y = point
        cv2.circle(
            corner_image,
            (int(x), int(y)),
            10,
            (0, 0, 255),
            -1
        )

    return corner_image


def save_corner_image(corner_image):
    OUTPUT_DIR.mkdir(exist_ok=True)

    output_path = OUTPUT_DIR / "maze_corners.jpg"
    cv2.imwrite(str(output_path), corner_image)

    print(f"Corner image saved to: {output_path}")


def save_warped_image(warped_image):
    OUTPUT_DIR.mkdir(exist_ok=True)

    output_path = OUTPUT_DIR / "top_down_maze.jpg"
    cv2.imwrite(str(output_path), warped_image)

    print(f"Top-down maze image saved to: {output_path}")

def draw_maze_grid(warped_image):
    grid_image = warped_image.copy()

    number_of_cells = 9

    image_height, image_width = grid_image.shape[:2]

    cell_width = image_width // number_of_cells
    cell_height = image_height // number_of_cells

    # Draw the vertical grid lines
    for column in range(number_of_cells + 1):
        x = column * cell_width

        cv2.line(
            grid_image,
            (x, 0),
            (x, image_height),
            (255, 0, 0),
            2
        )

    # Draw the horizontal grid lines
    for row in range(number_of_cells + 1):
        y = row * cell_height

        cv2.line(
            grid_image,
            (0, y),
            (image_width, y),
            (255, 0, 0),
            2
        )

    return grid_image

def save_grid_image(grid_image):
    OUTPUT_DIR.mkdir(exist_ok=True)

    output_path = OUTPUT_DIR / "maze_grid.jpg"

    saved = cv2.imwrite(str(output_path), grid_image)

    if saved:
        print(f"Maze grid image saved to: {output_path}")
    else:
        print("Error: maze grid image was not saved")

        