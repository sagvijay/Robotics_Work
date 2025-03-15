import numpy as np
import cv2
from queue import Queue


# Canvas dimensions
canvas_width = 180
canvas_height = 50

# Clearance for obstacles
clearance = 2  



# 8 functions corresponding to each action (8 directions)
def move_up_left(x, y):
    return x - 1, y - 1

def move_up(x, y):
    return x, y - 1

def move_up_right(x, y):
    return x + 1, y - 1

def move_left(x, y):
    return x - 1, y

def move_right(x, y):
    return x + 1, y

def move_down_left(x, y):
    return x - 1, y + 1

def move_down(x, y):
    return x, y + 1

def move_down_right(x, y):
    return x + 1, y + 1


"----------------------All the functions between here to the end of this marker defines the space that constitute the space on and inside the letters and numbers E,N,P,M,6,6,1------------"
def inside_E(x, y, x0=10+20, y0=35, width=10, height=20, mid_width=7, thickness=3):
    # Define the main leg
    if x0 <= x <= x0 + thickness and y0 - height <= y <= y0:
        return True

    # Define the top bar
    if x0 <= x <= x0 + width and y0 - height <= y <= y0 - height + thickness:
        return True

    # Define the middle horizontal bar 
    if x0 <= x <= x0 + mid_width and y0 - height // 2 - thickness // 2 <= y <= y0 - height // 2 + thickness // 2:
        return True

    # Define the bottom horizontal bar
    if x0 <= x <= x0 + width and y0 - thickness <= y <= y0:
        return True

    return False

def inside_N(x, y, x0=25+20, y0=35, width=10, height=20, thickness=3):

    # Left vertical bar
    if x0 <= x <= x0 + thickness and y0 - height <= y <= y0:
        return True

    # Right vertical bar
    if x0 + width - thickness <= x <= x0 + width and y0 - height <= y <= y0:
        return True

    # Diagonal connecting top-left to bottom-right
    slope = (2/3)*height / (width - 2 * thickness)
    y_s = (slope * (x - (x0 + thickness))) + (y0 - height)

    if x0 + thickness <= x <= x0 + width - thickness and y_s <= y <= y_s + height/3:
        return True

    return False




def inside_P(x, y, x0=40+20, y0=35, width=10, height=20, thickness=3):
   
    # Left vertical bar
    if x0 <= x <= x0 + thickness and y0 - height <= y <= y0:
          return True

    # Approximating the curved section using a bounding box (semi-circle)
    curve_x = x0 + thickness
    curve_y = y0 - (3 * height / 4)  # Approximate center of the curve
    curve_r = height / 4
    # Circular region approximation for "P"
    if ((x - curve_x) ** 2 + ((y - curve_y) ** 2) <= curve_r ** 2 and y < y0 - height // 2 and x> x0 + thickness):
        return True

    return False

def inside_M(x, y, x0=50+20, y0=35, width=15, height=20, thickness=3):

    # Left vertical bar
    if x0 <= x <= x0 + thickness and y0 - height <= y <= y0:
        return True

    # Right vertical bar
    if x0 + width - thickness <= x <= x0 + width and y0 - height <= y <= y0:
        return True

    # Left diagonal (bottom-left to middle-bottom)
    slope_left = (height / 2) / (width / 2 - thickness)
    y_left = slope_left * (x - x0 - thickness) + (y0 - height)

    if x0 + thickness <= x <= x0 + width / 2 and y_left  <= y <= y_left + 10:
        return True

    # Right diagonal (middle-bottom to bottom-right)
    slope_right = (-height / 2) / (width / 2 - thickness)
    y_right = slope_right * (x - (x0 + width / 2)) + (y0 - height / 2)

    if x0 + width / 2 <= x <= x0 + width - thickness and y_right <= y <= y_right + 10:
        return True

    return False




def inside_1(x, y, x0=110+20, y0=35, width=5, height=20, thickness=3):

    # Vertical bar of "1"
    if x0 <= x <= x0 + thickness and y0 - height <= y <= y0:
        return True

    return False

def inside_6_second(x, y, x0=(180/2)+20, y0=35, large=20/2, med=13//2, small_radius=8/2, hole_radius=5//2, thickness=3):
    # The cut out that makes the tail of the 6
    top_x = x0 + large - thickness  
    top_y = y0 - large * 1.5  # Positioned higher
    inside_top = ((x - top_x) ** 2 + (y - top_y) ** 2) <= 4 ** 2 and x <= top_x

    # Obstacle that makes the larger semi circle
    mid_x = x0 + large - thickness  # Positioned right
    mid_y = y0 - large  # Centered in middle
    inside_middle = ((x - mid_x) ** 2 + (y - mid_y) ** 2) <= large ** 2 and x <= mid_x

    # Smaller semi circle of the 6
    bottom_x = x0 + large - thickness  # Center slightly right
    bottom_y = y0 - med # Centered lower
    inside_bottom = ((x - bottom_x) ** 2 + (y - bottom_y) ** 2) <= med ** 2 and x >= bottom_x

    #Hole of 6
    hole_x = x0 + med  
    hole_y = y0 - med 
    inside_hole = ((x - hole_x) ** 2 + (y - hole_y) ** 2) <= hole_radius ** 2  # Inside hole

    # Final shape: A point is part of "6" unless it's inside the hole
    if (inside_bottom or inside_middle) and not inside_top and not inside_hole:
        return True

    return False

def inside_6_first(x, y, x0=(145//2)+20, y0=35, large=20/2, med=13//2, small_radius=8/2, hole_radius=5//2, thickness=3):
    # The cut out that makes the tail of the 6
    top_x = x0 + large - thickness  
    top_y = y0 - large * 1.5  # Positioned higher
    inside_top = ((x - top_x) ** 2 + (y - top_y) ** 2) <= 4 ** 2 and x <= top_x

    # Obstacle that makes the larger semi circle
    mid_x = x0 + large - thickness  # Positioned right
    mid_y = y0 - large  # Centered in middle
    inside_middle = ((x - mid_x) ** 2 + (y - mid_y) ** 2) <= large ** 2 and x <= mid_x

    # Smaller semi circle of the 6
    bottom_x = x0 + large - thickness  # Center slightly right
    bottom_y = y0 - med # Centered lower
    inside_bottom = ((x - bottom_x) ** 2 + (y - bottom_y) ** 2) <= med ** 2 and x >= bottom_x

    #Hole of 6
    hole_x = x0 + med  
    hole_y = y0 - med 
    inside_hole = ((x - hole_x) ** 2 + (y - hole_y) ** 2) <= hole_radius ** 2  # Inside hole

    # Final shape: A point is part of "6" unless it's inside the hole
    if (inside_bottom or inside_middle) and not inside_top and not inside_hole:
        return True

    return False
"-------------------------------------x----------------------------"


#Function that generates obstacle and clearance spaces"

def generate_obstacles(grid_width, grid_height, clearance):
    """
    Creates an obstacle mask with a clearance region.
    - Obstacles are BLACK.
    - Clearance is BLUE.
    """
    # Initialize obstacle mask
    obstacle_mask = np.zeros((grid_height, grid_width), dtype=np.uint8)

    # Combine all letter/number obstacle checks into one list
    shapes = [inside_E, inside_N, inside_P, inside_M, inside_1, inside_6_second, inside_6_first]

    # For each shape, mark obstacles
    for y in range(grid_height):
        for x in range(grid_width):
            if any(shape(x, y) for shape in shapes):
                obstacle_mask[y, x] = 255  # Mark obstacle pixels

    # Expand clearance using OpenCV dilation
    kernel = np.ones((clearance * 2, clearance * 2), np.uint8)
    clearance_mask = cv2.dilate(obstacle_mask, kernel, iterations=1)

    # Ensure obstacles remain distinct (do not overwrite obstacles)
    clearance_mask[obstacle_mask == 255] = 255

    return obstacle_mask, clearance_mask


# Generate obstacles
obstacle_mask, clearance_mask = generate_obstacles(canvas_width, canvas_height, clearance)

# Create visualization workspace
workspace = np.ones((canvas_height, canvas_width, 3), dtype=np.uint8) * 255  

# Draw obstacles in BLACK
workspace[np.where(obstacle_mask == 255)] = (0, 0, 0)  # Obstacles black

# Add blue outline clearance around the edges of the canvas
outline_thickness = 2  # Thickness of the blue outline

# Draw the top and bottom outlines
workspace[0:outline_thickness, :] = (255, 0, 0)  # Top outline
workspace[-outline_thickness:, :] = (255, 0, 0)  # Bottom outline

# Draw the left and right outlines
workspace[:, 0:outline_thickness] = (255, 0, 0)  # Left outline
workspace[:, -outline_thickness:] = (255, 0, 0)  # Right outline

# Mark the clearance area around obstacles in blue
clearance_region = np.where((clearance_mask == 255) & (obstacle_mask == 0))

# Set clearance region to BLUE
workspace[clearance_region] = (255, 0, 0)

# Create binary masks for obstacles and clearance
obstacle_mask_bgr = np.all(workspace == [0, 0, 0], axis=-1)   # Black obstacles needed for bfs
clearance_mask_bgr = np.all(workspace == [255, 0, 0], axis=-1) # Blue clearance needed for bfs 

# BFS Pathfinding Algorithm
def bfs(start, goal):
    visited = np.zeros((canvas_height, canvas_width), dtype=bool)
    parent = {}

    queue = Queue()
    queue.put(start)
    visited[start[1], start[0]] = True

    while not queue.empty():
        x, y = queue.get()

        if (x, y) == goal:# gets to goal, then it goes into this condition and backtracks
            path = []
            while (x, y) != start:
                path.append((x, y))
                x, y = parent[(x, y)]
            path.append(start)
            return path[::-1]

        # Check the new node after each action and if it's within bounds and not blocked by obstacles or clearance
        for action in [move_up_left, move_up, move_up_right, move_left, move_right, move_down_left, move_down, move_down_right]:
            nx, ny = action(x, y)

            if 0 <= nx < canvas_width and 0 <= ny < canvas_height:
                # Prevent BFS from entering obstacles & clearance areas
                if obstacle_mask_bgr[ny, nx] or clearance_mask_bgr[ny, nx]:  
                    continue  

                if not visited[ny, nx]:
                    visited[ny, nx] = True
                    parent[(nx, ny)] = (x, y)
                    queue.put((nx, ny))

                    workspace[ny, nx] = (200, 200, 200)  # Mark visited nodes in gray
                    cv2.imshow("BFS Pathfinding", cv2.resize(workspace, (720, 200)))
                    cv2.waitKey(1)

    return None  

# Define start and goal points
start_x = int(input("Enter start node x coordinate : "))
start_y = int(input("Enter start node y coordinate : "))
goal_x = int(input("Enter goal node x coordinate : "))
goal_y = int(input("Enter goal node y coordinate : "))


# Check if start and goal are inside obstacles or clearance area
if np.array_equal(workspace[canvas_height-start_y, start_x], [0, 0, 0]) or np.array_equal(workspace[canvas_height-start_y, start_x], [255, 0, 0]):# y value has been adjusted to be set w.r.t bottom left corner as origin
    print("Start is inside an obstacle or clearance area!")
elif np.array_equal(workspace[canvas_height-goal_y, goal_x], [0, 0, 0]) or np.array_equal(workspace[canvas_height-goal_y, goal_x], [255, 0, 0]):#y value has been adjusted to be set w.r.t bottom left corner as origin
    print("Goal is inside an obstacle or clearance area!")
else:
    # Mark start and goal positions
    workspace[canvas_height-start_y, start_x] = (0, 255, 0)  # Green start point, y value has been adjusted to be set w.r.t bottom left corner as origin
    workspace[canvas_height-goal_y, goal_x] = (0, 0, 255)    # Red goal point, y value has been adjusted to be set w.r.t bottom left corner as origin

    # Run BFS
    path = bfs((start_x,canvas_height-start_y),(goal_x,canvas_height-goal_y))#y value has been adjusted to be set w.r.t bottom left corner as origin

    # Draw final path in red if found
    if path:
        for x, y in path:
            workspace[y, x] = (238, 130, 238)  
            cv2.imshow("BFS Pathfinding", cv2.resize(workspace, (720, 200)))
            cv2.waitKey(1)
    else:
        print("No path found!")

cv2.waitKey(0)
cv2.destroyAllWindows()
