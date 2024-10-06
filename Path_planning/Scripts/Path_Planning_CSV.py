import cv2
from google.colab.patches import cv2_imshow
import numpy as np
import csv
from heapq import heappush, heappop
from math import sqrt, inf
import time

# Step 1: Load the image and process it
def process_image(image_path):
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    _, binary_image = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
    grid_size = (100, 100)  # Adjust grid size as needed

    binary_image = cv2.dilate(binary_image, np.ones((45 , 30), np.uint8), iterations=2)
    binary_grid = cv2.resize(binary_image, grid_size)
    return binary_grid

# Optimized A* Algorithm using heapq
def astar(grid, start, goal):
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance

    open_list = []
    heappush(open_list, (0 + heuristic(start, goal), 0, start, [start]))
    visited = set()

    while open_list:
        _, cost, current, path = heappop(open_list)

        if current in visited:
            continue

        visited.add(current)

        if current == goal:
            return path

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            neighbor = (current[0] + dx, current[1] + dy)

            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                if grid[neighbor[0], neighbor[1]] == 0 and neighbor not in visited:
                    heappush(open_list, (cost + heuristic(neighbor, goal), cost + 1, neighbor, path + [neighbor]))

    return []

def get_direction_changes(path):
    """Function to get the points where the line changes direction, including U-turns."""
    if len(path) < 2:
        return []

    direction_changes = [path[0]]  # Starting point is always included

    for i in range(1, len(path) - 1):
        x1, y1 = path[i - 1]
        x2, y2 = path[i]
        x3, y3 = path[i + 1]

        dx1, dy1 = x2 - x1, y2 - y1  # Direction from (x1, y1) to (x2, y2)
        dx2, dy2 = x3 - x2, y3 - y2  # Direction from (x2, y2) to (x3, y3)

        if (dx1 != dx2 or dy1 != dy2) or (dx1 == -dx2 and dy1 == -dy2):
            direction_changes.append(path[i])

    direction_changes.append(path[-1])  # Add the final point of the path

    return direction_changes

# Step 6: Putting it all together
import numpy as np
import math

def euclidean_distance(p1, p2):
    """Calculate the Euclidean distance between two points."""
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

def angle_between(p1, p2, p3):
    """Calculate the angle between three points (p1, p2, p3) where p2 is the middle point."""
    a = (p2[0] - p1[0], p2[1] - p1[1])
    b = (p3[0] - p2[0], p3[1] - p2[1])
    dot_product = a[0] * b[0] + a[1] * b[1]
    mag_a = math.sqrt(a[0] ** 2 + a[1] ** 2)
    mag_b = math.sqrt(b[0] ** 2 + b[1] ** 2)

    if mag_a == 0 or mag_b == 0:
        return 0

    cos_theta = dot_product / (mag_a * mag_b)
    return math.acos(min(1, max(-1, cos_theta)))  # Clamp value to [-1, 1] to avoid numerical errors

def clean_path(points, distance_threshold=4):
    cleaned_points = [points[0]]  # Start with the first point

    for i in range(1, len(points) - 1):
        prev_point = cleaned_points[-1]
        current_point = points[i]
        next_point = points[i + 1]

        dist = euclidean_distance(prev_point, current_point)

        if dist > distance_threshold:
            cleaned_points.append(current_point)

    cleaned_points.append(points[-1])  # Always include the last point

    return cleaned_points

def smooth_path(path, angle_threshold=0.2):
    """Smooth the path by removing unnecessary turns."""
    smoothed_path = [path[0]]

    for i in range(1, len(path) - 1):
        prev_point = path[i - 1]
        current_point = path[i]
        next_point = path[i + 1]

        angle = angle_between(prev_point, current_point, next_point)

        # If the angle is below the threshold, skip the current point
        if angle < angle_threshold:
            continue

        smoothed_path.append(current_point)

    smoothed_path.append(path[-1])  # Always include the last point
    return smoothed_path

def explore_field(grid, start_point):
    unvisited_points = set((i, j) for i in range(grid.shape[0]) for j in range(grid.shape[1]) if grid[i, j] == 0)

    current_position = start_point
    entire_path = []
    radius = 7.954  # Define the radius for proximity check

    while unvisited_points:
        unvisited_points = {point for point in unvisited_points if euclidean_distance(current_position, point) > radius}

        if not unvisited_points:
            break

        closest_point = min(unvisited_points, key=lambda point: abs(current_position[0] - point[0]) + abs(current_position[1] - point[1]))

        path_segment = astar(grid, current_position, closest_point)
        if path_segment:
            entire_path.extend(path_segment[1:])
            current_position = closest_point

    entire_path.insert(0, start_point)
    return entire_path

# Step 5: Save the path to a CSV file
def save_path_to_csv(path, filename):
    with open(filename, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["x", "y"])
        for point in path:
            writer.writerow(point)
    print(f"Path saved to {filename}")

# Step 6: Putting it all together
def main():
    start_time = time.time()  # Start the timer

    image_path = "/content/map.png"  # Replace with your image path
    grid = process_image(image_path)

    start_point = (70, 50)  # Adjust based on the image or area
    drone_path = explore_field(grid, start_point)
    print("Generated Path:", drone_path)

    # Clean the path to remove unnecessary points
    cleaned_path = clean_path(drone_path)
    smoothed_path = smooth_path(cleaned_path)

    print("Smoothed Path:", smoothed_path)

    csv_filename = "/content/collision_free_drone_path1.csv"
    csv_filename2=  "/content/collision_free_drone_complete_path1.csv"
    end_points = get_direction_changes(smoothed_path)

    print("END POINTS:", end_points)
    save_path_to_csv(end_points, csv_filename)
    save_path_to_csv(drone_path, csv_filename2)
    end_time = time.time()  # End the timer

if __name__ == "__main__":
    main()
