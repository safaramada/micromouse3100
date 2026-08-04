import cv2
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