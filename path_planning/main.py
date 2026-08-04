import cv2
from pathlib import Path

from maze_detector import (
    load_image,
    convert_to_grayscale,
    apply_threshold,
    find_maze_boundary,
    draw_maze_boundary,
    get_maze_corners,
    draw_corner_points,
    warp_maze_image,
    display_image,
    save_grayscale_image,
    save_threshold_image,
    save_boundary_image,
    save_corner_image,
    save_warped_image
)

CAMERA_INDEX = 0

PROJECT_DIR = Path(__file__).resolve().parent
IMAGE_DIR = PROJECT_DIR / "images"
IMAGE_PATH = IMAGE_DIR / "maze.jpg"


def capture_maze_image() -> None:
    """Display the webcam feed and save one image when the user presses S."""

    IMAGE_DIR.mkdir(parents=True, exist_ok=True)

    camera = cv2.VideoCapture(CAMERA_INDEX)

    if not camera.isOpened():
        raise RuntimeError(
            f"Could not open camera index {CAMERA_INDEX}. "
            "Try changing CAMERA_INDEX to 0 or 2."
        )

    print("Webcam opened.")
    print("Press S to capture the maze image.")
    print("Press Q to quit without saving.")

    try:
        while True:
            success, frame = camera.read()

            if not success:
                raise RuntimeError("The camera opened, but no frame was received.")

            cv2.imshow("Micromouse Maze Camera", frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("s"):
                saved = cv2.imwrite(str(IMAGE_PATH), frame)

                if not saved:
                    raise RuntimeError(f"Could not save image to {IMAGE_PATH}")

                print(f"Maze image saved to: {IMAGE_PATH}")
                break

            if key == ord("q"):
                print("Capture cancelled.")
                break

    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    capture_maze_image()

    maze_image = load_image()

    if maze_image is not None:
        display_image("Original Maze", maze_image)

        gray_image = convert_to_grayscale(maze_image)
        display_image("Grayscale Maze", gray_image)
        save_grayscale_image(gray_image)

        binary_image = apply_threshold(gray_image)
        display_image("Threshold Maze", binary_image)
        save_threshold_image(binary_image)

        maze_contour = find_maze_boundary(binary_image)

        if maze_contour is not None:
            boundary_image = draw_maze_boundary(
                maze_image,
                maze_contour
            )

            display_image(
                "Detected Maze Boundary",
                boundary_image
            )

            save_boundary_image(boundary_image)

        maze_contour = find_maze_boundary(binary_image)

        if maze_contour is not None:
            boundary_image = draw_maze_boundary(
                maze_image,
                maze_contour
            )

            display_image(
                "Detected Maze Boundary",
                boundary_image
            )

            save_boundary_image(boundary_image)

            corners = get_maze_corners(maze_contour)

            if corners is not None:
                corner_image = draw_corner_points(
                    maze_image,
                    corners
                )

                display_image(
                    "Detected Maze Corners",
                    corner_image
                )

                save_corner_image(corner_image)

                warped_image = warp_maze_image(
                    maze_image,
                    corners
                )

                display_image(
                    "Top Down Maze",
                    warped_image
                )

                save_warped_image(warped_image)

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