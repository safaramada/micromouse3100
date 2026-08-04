import cv2
from pathlib import Path

from maze_detector import (
    load_image,
    convert_to_grayscale,
    display_image,
    save_grayscale_image
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