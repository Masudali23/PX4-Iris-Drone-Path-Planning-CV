import cv2
import numpy as np
import csv
from scipy.spatial import distance

# Step 1: Load the image and process it (from previous steps)
def process_image(image_path):
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, binary_image = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)
    grid_size = (100, 100)  # Adjust grid size as needed
    grid = cv2.resize(binary_image, grid_size)
    binary_grid = (grid > 128).astype(int)
    return binary_grid, image

# Step 2: Load path from CSV (to mark the points on the image)
def load_path_from_csv(csv_filename):
    path = []
    with open(csv_filename, mode="r") as file:
        reader = csv.reader(file)
        next(reader)  # Skip header
        for row in reader:
            path.append((int(row[0]), int(row[1])))
    return path

# Step 3: Draw the path on the image
def draw_path(image, path):
    # Scale the points to match the original image dimensions
    img_h, img_w, _ = image.shape
    grid_size = (100, 100)

    # Scaling factors
    x_scale = img_w / grid_size[1]
    y_scale = img_h / grid_size[0]

    for i in range(len(path)):
        x, y = int(path[i][1] * x_scale), int(path[i][0] * y_scale)  # Inverting x and y
        # Draw a green circle at each waypoint
        cv2.circle(image, (x, y), radius=int(7.985*x_scale), color=(0, 255, 0), thickness=1)

        if i > 0:
            prev_x, prev_y = int(path[i - 1][1] * x_scale), int(path[i - 1][0] * y_scale)
            # Draw a red line between consecutive waypoints
            cv2.line(image, (prev_x, prev_y), (x, y), color=(0, 0, 255), thickness=2)
    center=(int(12*x_scale),int(12*x_scale))
    cv2.circle(image, center, radius=5, color=(255, 0, 0), thickness=1)
# Step 4: Save the resulting image
def save_image_with_path(image, output_path):
    cv2.imwrite(output_path, image)
    print(f"Image with path saved to {output_path}")

# Step 5: Putting it all together
def main():
    image_path = "/content/map.png" # Replace with your image path
    csv_filename = "collision_free_drone_complete_path1.csv"  # The CSV file with waypoints
    output_image_path = "output_image_with_path3.png"

    # Load and process the image
    grid, image = process_image(image_path)

    # Load the path from the CSV file
    path = load_path_from_csv(csv_filename)
    flight_distance = np.sum([distance.euclidean(path[i], path[i+1]) for i in range(len(path)-1)])
    flight_time = flight_distance/0.5  # 1 meter per second speed
    print(f"Estimated flight time: {flight_time:.2f} seconds")
    # Draw the path on the image
    draw_path(image, path)

    # Save the final image with the path drawn
    save_image_with_path(image, output_image_path)

if __name__ == "__main__":
    main()


