#Pacman in Python with PyGame
#https://github.com/hbokmann/Pacman

import pygame
import heapq
import math
import random
  
black = (0,0,0)
white = (255,255,255)
blue = (0,0,255)
green = (0,255,0)
red = (255,0,0)
purple = (255,0,255)
yellow   = ( 255, 255,   0)
orange = (255, 165, 0)
cyan = (0, 255, 255)

Trollicon=pygame.image.load('images/Trollman.png')
pygame.display.set_icon(Trollicon)

#Add music
pygame.mixer.init()
pygame.mixer.music.load('pacman.mp3')
pygame.mixer.music.play(-1, 0.0)

# This class represents the bar at the bottom that the player controls
class Wall(pygame.sprite.Sprite):
    # Constructor function
    def __init__(self,x,y,width,height, color):
        # Call the parent's constructor
        pygame.sprite.Sprite.__init__(self)
  
        # Make a blue wall, of the size specified in the parameters
        self.image = pygame.Surface([width, height])
        self.image.fill(color)
  
        # Make our top-left corner the passed-in location.
        self.rect = self.image.get_rect()
        self.rect.top = y
        self.rect.left = x

# This creates all the walls in room 1
def setupRoomOne(all_sprites_list):
    # Make the walls. (x_pos, y_pos, width, height)
    wall_list=pygame.sprite.RenderPlain()
     
    # This is a list of walls. Each is in the form [x, y, width, height]
    walls = [ [0,0,6,600],
              [0,0,600,6],
              [0,600,606,6],
              [600,0,6,606],
              [300,0,6,66],
              [60,60,186,6],
              [360,60,186,6],
              [60,120,66,6],
              [60,120,6,126],
              [180,120,246,6],
              [300,120,6,66],
              [480,120,66,6],
              [540,120,6,126],
              [120,180,126,6],
              [120,180,6,126],
              [360,180,126,6],
              [480,180,6,126],
              [180,240,6,126],
              [180,360,246,6],
              [420,240,6,126],
              [240,240,42,6],
              [324,240,42,6],
              [240,240,6,66],
              [240,300,126,6],
              [360,240,6,66],
              [0,300,66,6],
              [540,300,66,6],
              [60,360,66,6],
              [60,360,6,186],
              [480,360,66,6],
              [540,360,6,186],
              [120,420,366,6],
              [120,420,6,66],
              [480,420,6,66],
              [180,480,246,6],
              [300,480,6,66],
              [120,540,126,6],
              [360,540,126,6]
            ]
     
    # Loop through the list. Create the wall, add it to the list
    for item in walls:
        wall=Wall(item[0],item[1],item[2],item[3],blue)
        wall_list.add(wall)
        all_sprites_list.add(wall)
         
    # return our new list
    return wall_list

def setupGate(all_sprites_list):
      gate = pygame.sprite.RenderPlain()
      gate.add(Wall(282,242,42,2,white))
      all_sprites_list.add(gate)
      return gate

# This class represents the ball        
# It derives from the "Sprite" class in Pygame
class Block(pygame.sprite.Sprite):
     
    # Constructor. Pass in the color of the block, 
    # and its x and y position
    def __init__(self, color, width, height):
        # Call the parent class (Sprite) constructor
        pygame.sprite.Sprite.__init__(self) 
 
        # Create an image of the block, and fill it with a color.
        # This could also be an image loaded from the disk.
        self.image = pygame.Surface([width, height])
        self.image.fill(white)
        self.image.set_colorkey(white)
        pygame.draw.ellipse(self.image,color,[0,0,width,height])
 
        # Fetch the rectangle object that has the dimensions of the image
        # image.
        # Update the position of this object by setting the values 
        # of rect.x and rect.y
        self.rect = self.image.get_rect() 

# This class represents the bar at the bottom that the player controls
class Player(pygame.sprite.Sprite):
  
    # Set speed vector
    change_x=0
    change_y=0
  
    # Constructor function
    def __init__(self,x,y, filename):
        # Call the parent's constructor
        pygame.sprite.Sprite.__init__(self)
   
        # Set height, width
        self.image = pygame.image.load(filename).convert()
  
        # Make our top-left corner the passed-in location.
        self.rect = self.image.get_rect()
        self.rect.top = y
        self.rect.left = x
        self.prev_x = x
        self.prev_y = y

    # Clear the speed of the player
    def prevdirection(self):
        self.prev_x = self.change_x
        self.prev_y = self.change_y

    # Change the speed of the player
    def changespeed(self,x,y):
        self.change_x+=x
        self.change_y+=y
          
    # Find a new position for the player
    def update(self,walls,gate):
        # Get the old position, in case we need to go back to it
        
        old_x=self.rect.left
        new_x=old_x+self.change_x
        prev_x=old_x+self.prev_x
        self.rect.left = new_x
        
        old_y=self.rect.top
        new_y=old_y+self.change_y
        prev_y=old_y+self.prev_y

        # Did this update cause us to hit a wall?
        x_collide = pygame.sprite.spritecollide(self, walls, False)
        if x_collide:
            # Whoops, hit a wall. Go back to the old position
            self.rect.left=old_x
        else:
            self.rect.top = new_y

            # Did this update cause us to hit a wall?
            y_collide = pygame.sprite.spritecollide(self, walls, False)
            if y_collide:
                # Whoops, hit a wall. Go back to the old position
                self.rect.top=old_y

        if gate != False:
          gate_hit = pygame.sprite.spritecollide(self, gate, False)
          if gate_hit:
            self.rect.left=old_x
            self.rect.top=old_y

#Inheritime Player klassist
class Ghost(Player):
    # Change the speed of the ghost
    def changespeed(self,list,ghost,turn,steps,l):
      try:
        z=list[turn][2]
        if steps < z:
          self.change_x=list[turn][0]
          self.change_y=list[turn][1]
          steps+=1
        else:
          if turn < l:
            turn+=1
          elif ghost == "clyde":
            turn = 2
          else:
            turn = 0
          self.change_x=list[turn][0]
          self.change_y=list[turn][1]
          steps = 0
        return [turn,steps]
      except IndexError:
         return [0,0]

Pinky_directions = [
[0,-30,4],
[15,0,9],
[0,15,11],
[-15,0,23],
[0,15,7],
[15,0,3],
[0,-15,3],
[15,0,19],
[0,15,3],
[15,0,3],
[0,15,3],
[15,0,3],
[0,-15,15],
[-15,0,7],
[0,15,3],
[-15,0,19],
[0,-15,11],
[15,0,9]
]

Blinky_directions = [
[0,-15,4],
[15,0,9],
[0,15,11],
[15,0,3],
[0,15,7],
[-15,0,11],
[0,15,3],
[15,0,15],
[0,-15,15],
[15,0,3],
[0,-15,11],
[-15,0,3],
[0,-15,11],
[-15,0,3],
[0,-15,3],
[-15,0,7],
[0,-15,3],
[15,0,15],
[0,15,15],
[-15,0,3],
[0,15,3],
[-15,0,3],
[0,-15,7],
[-15,0,3],
[0,15,7],
[-15,0,11],
[0,-15,7],
[15,0,5]
]

Inky_directions = [
[30,0,2],
[0,-15,4],
[15,0,10],
[0,15,7],
[15,0,3],
[0,-15,3],
[15,0,3],
[0,-15,15],
[-15,0,15],
[0,15,3],
[15,0,15],
[0,15,11],
[-15,0,3],
[0,-15,7],
[-15,0,11],
[0,15,3],
[-15,0,11],
[0,15,7],
[-15,0,3],
[0,-15,3],
[-15,0,3],
[0,-15,15],
[15,0,15],
[0,15,3],
[-15,0,15],
[0,15,11],
[15,0,3],
[0,-15,11],
[15,0,11],
[0,15,3],
[15,0,1],
]

Clyde_directions = [
[-30,0,2],
[0,-15,4],
[15,0,5],
[0,15,7],
[-15,0,11],
[0,-15,7],
[-15,0,3],
[0,15,7],
[-15,0,7],
[0,15,15],
[15,0,15],
[0,-15,3],
[-15,0,11],
[0,-15,7],
[15,0,3],
[0,-15,11],
[15,0,9],
]

pl = len(Pinky_directions)-1
bl = len(Blinky_directions)-1
il = len(Inky_directions)-1
cl = len(Clyde_directions)-1

# Search Algorithm Implementation
class SearchAlgorithm:
    def __init__(self, walls, block_size=30):
        self.walls = walls
        self.block_size = block_size
        self.grid_offset = 6 # Pacman's grid offset
        self.dot_offset = 32  # Dots are offset by 32 pixels (6 + 26)
        
    
    def get_grid_cell(self, x, y, is_dot=False):
        """Get the grid cell coordinates for a position"""
        offset = self.dot_offset if is_dot is True else self.grid_offset
        grid_x = (x - offset) // self.block_size
        grid_y = (y - offset) // self.block_size

        aligned_x = grid_x * self.block_size + offset
        aligned_y = grid_y * self.block_size + offset
        
        aligned_x = max(offset, min(aligned_x, 606 - offset - self.block_size))
        aligned_y = max(offset, min(aligned_y, 606 - offset - self.block_size)) 

        return (aligned_x, aligned_y)
    

    def is_valid_position(self, x, y):
        # Create a test sprite with Pacman's actual size (30x30)
        test_sprite = pygame.sprite.Sprite()
        test_sprite.rect = pygame.Rect(x, y, 30, 30)
        
        # Check if position is within bounds
        if x < 6 or x > 570 or y < 6 or y > 570:
            return False
            
        # Check for wall collisions
        collisions = pygame.sprite.spritecollide(test_sprite, self.walls, False)
        return len(collisions) == 0
    
    def get_neighbors(self, x, y, algorithm="A*"):
        neighbors = []
        # Different direction orders for different algorithms
        if algorithm == "GBFS":
            # GBFS: Try directions that get closer to goal first (heuristic-based order)
            directions = [
                (self.block_size, 0),   # Right
                (0, self.block_size),   # Down  
                (-self.block_size, 0),  # Left
                (0, -self.block_size)   # Up
            ]
        elif algorithm == "UCS":
            # UCS: Direction
            directions = [
                (0, -self.block_size),  # Up 
                (self.block_size, 0),   # Right 
                (-self.block_size, 0),  # Left 
                (0, self.block_size),   # Down 
            ]
        else:  # A*
            # A*: Standard order
            directions = [
                (self.block_size, 0),   # Right
                (-self.block_size, 0),  # Left  
                (0, self.block_size),   # Down
                (0, -self.block_size)   # Up
            ]
        
        # Add random variations to costs to create more differences between algorithms
        costs = {
            (self.block_size, 0): 1.0 + random.uniform(1, 100),    # Right
            (-self.block_size, 0): 1.0 + random.uniform(1, 100),   # Left  
            (0, self.block_size): 1.2 + random.uniform(1, 100),    # Down
            (0, -self.block_size): 0.8 + random.uniform(1, 100)    # Up
        }
        
        for dx, dy in directions:
            new_x, new_y = x + dx, y + dy
            if self.is_valid_position(new_x, new_y):
                neighbors.append(((new_x, new_y), costs[(dx, dy)]))
        return neighbors
    
    def manhattan_distance(self, x1, y1, x2, y2):
        # Calculate grid distance using Pacman's grid
        grid_x1 = (x1 - self.grid_offset) // self.block_size
        grid_y1 = (y1 - self.grid_offset) // self.block_size
        grid_x2 = (x2 - self.grid_offset) // self.block_size
        grid_y2 = (y2 - self.grid_offset) // self.block_size
        return abs(grid_x1 - grid_x2) + abs(grid_y1 - grid_y2)
    
    def a_star_search(self, start, goal):
        # Align start and goal to Pacman's grid 
        aligned_start = self.get_grid_cell(start[0], start[1], is_dot=False)
        aligned_goal = self.get_grid_cell(goal[0], goal[1], is_dot=False)

        print(f"A*: From {start}->{aligned_start} to {goal}->{aligned_goal}")
        
        if aligned_start == aligned_goal:
            return []
        
        open_set = []
        heapq.heappush(open_set, (0, aligned_start))
        came_from = {}
        g_score = {aligned_start: 0}
        f_score = {aligned_start: self.manhattan_distance(aligned_start[0], aligned_start[1], aligned_goal[0], aligned_goal[1])}
        
        visited = set()
        
        while open_set:
            current_f, current = heapq.heappop(open_set)
            
            if current in visited:
                continue
            visited.add(current)
            
            if current == aligned_goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path
                
            for neighbor_info in self.get_neighbors(current[0], current[1], "A*"):
                neighbor_pos, cost = neighbor_info
                
                if neighbor_pos in visited:
                    continue
                    
                tentative_g_score = g_score[current] + cost
                
                if neighbor_pos not in g_score or tentative_g_score < g_score[neighbor_pos]:
                    came_from[neighbor_pos] = current
                    g_score[neighbor_pos] = tentative_g_score
                    h_score = self.manhattan_distance(
                        neighbor_pos[0], neighbor_pos[1], 
                        aligned_goal[0], aligned_goal[1]
                    )
                    f_score[neighbor_pos] = tentative_g_score + h_score
                    heapq.heappush(open_set, (f_score[neighbor_pos], neighbor_pos))
        
        return []
    
    def greedy_best_first_search(self, start, goal):
        # Align start to Pacman grid
        aligned_start = self.get_grid_cell(start[0], start[1], is_dot=False)
        aligned_goal = self.get_grid_cell(goal[0], goal[1], is_dot=False)
        
        open_set = []
        heapq.heappush(open_set, (self.manhattan_distance(aligned_start[0], aligned_start[1], aligned_goal[0], aligned_goal[1]), aligned_start))
        came_from = {}
        visited = set()
        
        while open_set:
            current_priority, current = heapq.heappop(open_set)
            
            if current in visited:
                continue
            visited.add(current)
            
            if current == aligned_goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path
                
            for neighbor_info in self.get_neighbors(current[0], current[1], "GBFS"):
                neighbor_pos, _ = neighbor_info  # GBFS ignores cost
                
                if neighbor_pos not in visited:
                    came_from[neighbor_pos] = current
                    priority = self.manhattan_distance(
                        neighbor_pos[0], neighbor_pos[1], 
                        aligned_goal[0], aligned_goal[1]
                    )
                    heapq.heappush(open_set, (priority, neighbor_pos))
        
        return []
    
    def uniform_cost_search(self, start, goal):
        # Align start to Pacman grid
        aligned_start = self.get_grid_cell(start[0], start[1], is_dot=False)
        aligned_goal = self.get_grid_cell(goal[0], goal[1], is_dot=False)
        
        open_set = []
        heapq.heappush(open_set, (0, aligned_start))
        came_from = {}
        cost_so_far = {aligned_start: 0}
        
        while open_set:
            current_cost, current = heapq.heappop(open_set)
            
            if current == aligned_goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path
                
            for neighbor_info in self.get_neighbors(current[0], current[1], "UCS"):
                neighbor_pos, cost = neighbor_info
                new_cost = cost_so_far[current] + cost
                
                if neighbor_pos not in cost_so_far or new_cost < cost_so_far[neighbor_pos]:
                    cost_so_far[neighbor_pos] = new_cost
                    came_from[neighbor_pos] = current
                    heapq.heappush(open_set, (new_cost, neighbor_pos))
        
        return []

class AIPacman(Player):
    """AI-controlled Pacman using search algorithms"""
    
    def __init__(self, x, y, filename, search_algorithm="A*"):
        super().__init__(x, y, filename)
        self.search_algorithm = search_algorithm
        self.current_target = None
        
        # Pathfinding state
        self.current_path = []
        self.path_index = 0
        self.searcher = None
        self.walls_reference = None
        
        # Ghost avoidance
        self.is_running_away = False
        self.ghost_danger_distance = 90  # Reduced distance
        self.ghost_ignore_distance = 180
        
        # Track last position to detect stuck state
        self.last_positions = []
        self.stuck_counter = 0
        self.MAX_STUCK_FRAMES = 10
        
        # Statistics
        self.dots_collected = 0
        self.paths_calculated = 0
        self.ghosts_avoided = 0
        self.false_ghost_alarms = 0
    
    def set_algorithm(self, algorithm):
        """Change the search algorithm dynamically"""
        self.search_algorithm = algorithm
        self.current_path = []
        self.path_index = 0
        self.current_target = None
        self.paths_calculated = 0
        print(f"Switched to {algorithm} algorithm")
    
    def update(self, walls, gate, block_list, monsta_list):
        """Main update method - called every frame"""
        if self.walls_reference is None:
            self.walls_reference = walls
        
        # Check for nearby ghosts with wall-aware detection
        closest_ghost, ghost_distance, is_dangerous = self.get_closest_dangerous_ghost(monsta_list, walls)
        
        # Ghost avoidance logic - only run from reachable ghosts
        if is_dangerous and ghost_distance < self.ghost_danger_distance:
            if not self.is_running_away:
                print(f"⚠ Running from dangerous ghost! Distance: {ghost_distance}")
                self.ghosts_avoided += 1
            self.is_running_away = True
            self.run_away_from_ghost(closest_ghost, walls)
            # Clear path when running from ghost
            self.current_path = []
            self.path_index = 0
        else:
            if self.is_running_away:
                print("✓ Ghost danger passed, resuming dot collection")
            self.is_running_away = False
            
            # Follow existing path if we have one
            if self.current_path and self.path_index < len(self.current_path):
                self.follow_path(walls)
            else:
                # Find new path to dot
                self.find_direction_to_closest_dot(block_list, walls)
        
        # Call parent update to handle collisions
        super().update(walls, gate)
    
    def get_closest_dangerous_ghost(self, monsta_list, walls):
        """Find the closest ghost that's actually dangerous (reachable)"""
        if not monsta_list:
            return None, float('inf'), False
            
        closest_ghost = None
        min_distance = float('inf')
        is_dangerous = False
        
        for ghost in monsta_list:
            # Calculate straight-line distance
            distance = math.sqrt((ghost.rect.centerx - self.rect.centerx) ** 2 + 
                               (ghost.rect.centery - self.rect.centery) ** 2)
            
            # Ignore ghosts that are too far away
            if distance > self.ghost_ignore_distance:
                continue
            
            # Check if ghost is actually reachable (not blocked by walls)
            reachable = self.is_ghost_reachable(ghost, walls)
            
            # Ghost is dangerous if it's close AND reachable
            if distance < min_distance:
                min_distance = distance
                closest_ghost = ghost
                is_dangerous = reachable and distance < self.ghost_danger_distance
        
        return closest_ghost, min_distance, is_dangerous
    
    def is_ghost_reachable(self, ghost, walls):
        """Check if a ghost is reachable (not blocked by walls) using line-of-sight"""
        # Get Pacman and ghost grid positions
        pacman_grid = self.get_grid_position(self.rect.centerx, self.rect.centery)
        ghost_grid = self.get_grid_position(ghost.rect.centerx, ghost.rect.centery)
        
        # If they're in the same grid cell, they're definitely reachable
        if pacman_grid == ghost_grid:
            return True
        
        # Check if there's a direct path using Bresenham's line algorithm
        line_points = self.get_line_points(pacman_grid, ghost_grid)
        
        # Check each point along the line for walls
        for point in line_points:
            x, y = point
            # Convert grid coordinates back to pixel coordinates
            pixel_x = x * 30 + 6
            pixel_y = y * 30 + 6
            
            # Create a test sprite to check for wall collisions
            test_sprite = pygame.sprite.Sprite()
            test_sprite.rect = pygame.Rect(pixel_x, pixel_y, 30, 30)
            
            # If we hit a wall, the ghost is not directly reachable
            if pygame.sprite.spritecollide(test_sprite, walls, False):
                return False
        
        return True
    
    def get_grid_position(self, x, y):
        """Convert pixel coordinates to grid coordinates"""
        grid_x = (x - 6) // 30
        grid_y = (y - 6) // 30
        return (int(grid_x), int(grid_y))
    
    def get_line_points(self, start, end):
        """Get all grid points between two points using Bresenham's line algorithm"""
        x1, y1 = start
        x2, y2 = end
        points = []
        
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy
        
        while True:
            points.append((x1, y1))
            if x1 == x2 and y1 == y2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy
        
        return points
    
    
    def run_away_from_ghost(self, ghost, walls):
        """Smart ghost avoidance - move away from reachable ghosts"""
        if not ghost:
            return
        
        # Reset movement
        self.change_x = 0
        self.change_y = 0
        
        # Get grid positions
        pacman_grid = self.get_grid_position(self.rect.centerx, self.rect.centery)
        ghost_grid = self.get_grid_position(ghost.rect.centerx, ghost.rect.centery)
        
        # Calculate direction away from ghost in grid space
        dx_grid = pacman_grid[0] - ghost_grid[0]
        dy_grid = pacman_grid[1] - ghost_grid[1]
        
        # Try to move in the opposite direction of the ghost
        # Prioritize the direction with larger difference
        if abs(dx_grid) > abs(dy_grid):
            if dx_grid > 0 and self.can_move_right(walls):
                self.change_x = 30
            elif dx_grid < 0 and self.can_move_left(walls):
                self.change_x = -30
            elif dy_grid > 0 and self.can_move_down(walls):
                self.change_y = 30
            elif dy_grid < 0 and self.can_move_up(walls):
                self.change_y = -30
        else:
            if dy_grid > 0 and self.can_move_down(walls):
                self.change_y = 30
            elif dy_grid < 0 and self.can_move_up(walls):
                self.change_y = -30
            elif dx_grid > 0 and self.can_move_right(walls):
                self.change_x = 30
            elif dx_grid < 0 and self.can_move_left(walls):
                self.change_x = -30
        
        # If still no movement, try any safe direction
        if self.change_x == 0 and self.change_y == 0:
            safe_directions = []
            
            # Check all directions for safety
            if self.can_move_right(walls):
                # Check if moving right would move away from ghost
                test_x = self.rect.left + 30
                test_distance = math.sqrt((ghost.rect.centerx - test_x) ** 2 + 
                                        (ghost.rect.centery - self.rect.centery) ** 2)
                current_distance = math.sqrt((ghost.rect.centerx - self.rect.centerx) ** 2 + 
                                           (ghost.rect.centery - self.rect.centery) ** 2)
                if test_distance > current_distance:
                    safe_directions.append((30, 0, test_distance))
            
            if self.can_move_left(walls):
                test_x = self.rect.left - 30
                test_distance = math.sqrt((ghost.rect.centerx - test_x) ** 2 + 
                                        (ghost.rect.centery - self.rect.centery) ** 2)
                current_distance = math.sqrt((ghost.rect.centerx - self.rect.centerx) ** 2 + 
                                           (ghost.rect.centery - self.rect.centery) ** 2)
                if test_distance > current_distance:
                    safe_directions.append((-30, 0, test_distance))
            
            if self.can_move_down(walls):
                test_y = self.rect.top + 30
                test_distance = math.sqrt((ghost.rect.centerx - self.rect.centerx) ** 2 + 
                                        (ghost.rect.centery - test_y) ** 2)
                current_distance = math.sqrt((ghost.rect.centerx - self.rect.centerx) ** 2 + 
                                           (ghost.rect.centery - self.rect.centery) ** 2)
                if test_distance > current_distance:
                    safe_directions.append((0, 30, test_distance))
            
            if self.can_move_up(walls):
                test_y = self.rect.top - 30
                test_distance = math.sqrt((ghost.rect.centerx - self.rect.centerx) ** 2 + 
                                        (ghost.rect.centery - test_y) ** 2)
                current_distance = math.sqrt((ghost.rect.centerx - self.rect.centerx) ** 2 + 
                                           (ghost.rect.centery - self.rect.centery) ** 2)
                if test_distance > current_distance:
                    safe_directions.append((0, -30, test_distance))
            
            # Choose the direction that maximizes distance from ghost
            if safe_directions:
                safe_directions.sort(key=lambda x: x[2], reverse=True)  # Sort by distance
                self.change_x = safe_directions[0][0]
                self.change_y = safe_directions[0][1]
            else:
                # If no safe direction, just try any direction
                if self.can_move_right(walls):
                    self.change_x = 30
                elif self.can_move_left(walls):
                    self.change_x = -30
                elif self.can_move_down(walls):
                    self.change_y = 30
                elif self.can_move_up(walls):
                    self.change_y = -30
    
    # Keep the can_move_* methods as they were
    def can_move_right(self, walls):
        test_x = self.rect.left + 30
        test_y = self.rect.top
        test_sprite = pygame.sprite.Sprite()
        test_sprite.rect = pygame.Rect(test_x, test_y, 30, 30)
        return not pygame.sprite.spritecollide(test_sprite, walls, False)

    def can_move_left(self, walls):
        test_x = self.rect.left - 30
        test_y = self.rect.top
        test_sprite = pygame.sprite.Sprite()
        test_sprite.rect = pygame.Rect(test_x, test_y, 30, 30)
        return not pygame.sprite.spritecollide(test_sprite, walls, False)

    def can_move_up(self, walls):
        test_x = self.rect.left
        test_y = self.rect.top - 30
        test_sprite = pygame.sprite.Sprite()
        test_sprite.rect = pygame.Rect(test_x, test_y, 30, 30)
        return not pygame.sprite.spritecollide(test_sprite, walls, False)

    def can_move_down(self, walls):
        test_x = self.rect.left
        test_y = self.rect.top + 30
        test_sprite = pygame.sprite.Sprite()
        test_sprite.rect = pygame.Rect(test_x, test_y, 30, 30)
        return not pygame.sprite.spritecollide(test_sprite, walls, False)
    
 
    def find_direction_to_closest_dot(self, block_list, walls):
        """Find direction to closest dot using pathfinding algorithms"""
        if not block_list:
            return
        
        # Initialize searcher if needed
        if self.searcher is None:
            self.searcher = SearchAlgorithm(walls)
        
        current_pos = (self.rect.centerx, self.rect.centery)
        
        # Check if we need a new path
        need_new_path = False
        
        if not self.current_path or self.path_index >= len(self.current_path):
            need_new_path = True
            print("No current path, calculating new one...")
        elif self.current_target:
            # Check if current target still exists
            target_exists = False
            for block in block_list:
                if (block.rect.centerx, block.rect.centery) == self.current_target:
                    target_exists = True
                    break
            
            if not target_exists:
                need_new_path = True
                print("Target dot collected, calculating new path...")
        
        if need_new_path:
            # Find the closest dot
            closest_dot = None
            closest_distance = float('inf')
            
            for dot in block_list:
                dot_pos = (dot.rect.centerx, dot.rect.centery)
                # Use Manhattan distance for quick comparison
                distance = abs(dot_pos[0] - current_pos[0]) + abs(dot_pos[1] - current_pos[1])
                
                if distance < closest_distance:
                    closest_distance = distance
                    closest_dot = dot
            
            if closest_dot:
                dot_pos = (closest_dot.rect.centerx, closest_dot.rect.centery)
                print(f"\n=== New target: {dot_pos}, Distance: {closest_distance} ===")
                self.calculate_path(current_pos, dot_pos)
                self.current_target = dot_pos
            else:
                print("No dots found!")
    
    def calculate_path(self, start_pos, target_pos):
        """Calculate path using selected search algorithm"""
        if self.searcher is None:
            return
        
        self.paths_calculated += 1
        print(f"\nCalculating {self.search_algorithm} path #{self.paths_calculated}")
        print(f"From: {start_pos}")
        print(f"To: {target_pos}")
        
        path = []
        
        if self.search_algorithm == "A*":
            path = self.searcher.a_star_search(start_pos, target_pos)
        elif self.search_algorithm == "GBFS":
            path = self.searcher.greedy_best_first_search(start_pos, target_pos)
        elif self.search_algorithm == "UCS":
            path = self.searcher.uniform_cost_search(start_pos, target_pos)
        
        if path:
            self.current_path = path
            self.path_index = 0
            print(f"✓ {self.search_algorithm} found {len(path)} step path")
            
            # Show first few steps
            max_steps_to_show = min(5, len(path))
            for i in range(max_steps_to_show):
                print(f"  Step {i}: {path[i]}")
            if len(path) > max_steps_to_show:
                print(f"  ... and {len(path)-max_steps_to_show} more steps")
        else:
            print(f"✗ {self.search_algorithm}: No path found to {target_pos}")
            self.current_path = []
            self.path_index = 0
            self.current_target = None
    
    def follow_path(self, walls):
        """Follow the current path"""
        if not self.current_path:
            return
        
        if self.path_index >= len(self.current_path):
            print("✓ Path completed successfully!")
            self.current_path = []
            self.path_index = 0
            self.current_target = None
            return
        
        # Get current and target positions
        current_pos = (self.rect.left, self.rect.top)
        target_pos = self.current_path[self.path_index]
        
        # Track movement to detect stuck state
        self.last_positions.append(current_pos)
        if len(self.last_positions) > 5:
            self.last_positions.pop(0)
        
        # Check if stuck (not moving for several frames)
        if len(self.last_positions) == 5:
            # Check if all recent positions are the same
            if all(pos == self.last_positions[0] for pos in self.last_positions):
                self.stuck_counter += 1
                if self.stuck_counter > self.MAX_STUCK_FRAMES:
                    print("⚠ Stuck detected! Clearing path to recalculate")
                    self.current_path = []
                    self.path_index = 0
                    self.current_target = None
                    self.stuck_counter = 0
                    return
            else:
                self.stuck_counter = 0
        
        # Check if we've reached the current waypoint
        # Use the SearchAlgorithm's grid alignment check
        current_grid = self.searcher.get_grid_cell(current_pos[0], current_pos[1], is_dot=False)
        target_grid = self.searcher.get_grid_cell(target_pos[0], target_pos[1], is_dot=False)
        
        # Print debug info occasionally
        if random.random() < 0.05:  # 5% chance
            print(f"Following path: {self.path_index}/{len(self.current_path)-1}")
            print(f"  Current: {current_pos} -> {current_grid}")
            print(f"  Target: {target_pos} -> {target_grid}")
        
        # Check if we're at the target grid cell
        if current_grid == target_grid:
            print(f"✓ Reached waypoint {self.path_index} at {current_grid}")
            self.path_index += 1
            
            # If reached final waypoint
            if self.path_index >= len(self.current_path):
                print("✓ Reached final destination!")
                self.current_path = []
                self.path_index = 0
                self.current_target = None
                return
            
            # Update to next waypoint
            target_pos = self.current_path[self.path_index]
        
        # Move toward the current waypoint
        self.move_to_position(target_pos)
    
    def move_to_position(self, target_pos):
        """Move directly toward a specific position"""
        current_pos = (self.rect.left, self.rect.top)
        
        # Calculate direction to target
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        
        # Reset movement
        self.change_x = 0
        self.change_y = 0
        
        # Determine which direction to move based on larger difference
        # Try to move in primary direction first
        if abs(dx) > abs(dy):
            # Horizontal is primary
            if dx > 0 and self.can_move_right(self.walls_reference):
                self.change_x = 30
            elif dx < 0 and self.can_move_left(self.walls_reference):
                self.change_x = -30
            # If horizontal blocked, try vertical
            elif dy > 0 and self.can_move_down(self.walls_reference):
                self.change_y = 30
            elif dy < 0 and self.can_move_up(self.walls_reference):
                self.change_y = -30
        else:
            # Vertical is primary
            if dy > 0 and self.can_move_down(self.walls_reference):
                self.change_y = 30
            elif dy < 0 and self.can_move_up(self.walls_reference):
                self.change_y = -30
            # If vertical blocked, try horizontal
            elif dx > 0 and self.can_move_right(self.walls_reference):
                self.change_x = 30
            elif dx < 0 and self.can_move_left(self.walls_reference):
                self.change_x = -30
        
        # Debug: If no movement possible
        if self.change_x == 0 and self.change_y == 0:
            print(f"⚠ Can't move toward {target_pos} from {current_pos}")
            # Try any available direction as fallback
            directions = [
                (30, 0, self.can_move_right),
                (-30, 0, self.can_move_left),
                (0, 30, self.can_move_down),
                (0, -30, self.can_move_up)
            ]
            
            for dx_val, dy_val, check_func in directions:
                if check_func(self.walls_reference):
                    self.change_x = dx_val
                    self.change_y = dy_val
                    break
    
    def get_stats(self):
        """Get current statistics"""
        return {
            "algorithm": self.search_algorithm,
            "paths_calculated": self.paths_calculated,
            "ghosts_avoided": self.ghosts_avoided,
            "false_alarms": self.false_ghost_alarms,
            "current_path_length": len(self.current_path) if self.current_path else 0,
            "current_waypoint": self.path_index if self.current_path else 0,
            "is_running_from_ghost": self.is_running_away,
            "has_target": self.current_target is not None
        }
    

# Call this function so the Pygame library can initialize itself
pygame.init()
  
# Create an 606x606 sized screen
screen = pygame.display.set_mode([606, 606])

# Set the title of the window
pygame.display.set_caption('Pacman - Search Algorithms Comparison')

# Create a surface we can draw on
background = pygame.Surface(screen.get_size())

# Used for converting color maps and such
background = background.convert()
  
# Fill the screen with a black background
background.fill(black)

clock = pygame.time.Clock()

pygame.font.init()
font = pygame.font.Font("freesansbold.ttf", 24)
title_font = pygame.font.Font("freesansbold.ttf", 36)
menu_font = pygame.font.Font("freesansbold.ttf", 28)

#default locations for Pacman and monstas
w = 303-16 #Width
p_h = (7*60)+19 #Pacman height
m_h = (4*60)+19 #Monster height
b_h = (3*60)+19 #Binky height
i_w = 303-16-32 #Inky width
c_w = 303+(32-16) #Clyde width

def draw_menu(selected_algorithm):
    screen.fill(black)
    
    # Title
    title = title_font.render("PAC-MAN AI", True, yellow)
    screen.blit(title, [180, 100])
    
    subtitle = font.render("Search Algorithms Comparison", True, white)
    screen.blit(subtitle, [150, 150])
    
    # Algorithm descriptions
    algorithms = [
        ("A* Search", "Optimal pathfinding with heuristic", green),
        ("Greedy Best-First", "Fast but not always optimal", cyan),
        ("Uniform Cost", "Optimal without heuristic", orange),
    ]
    
    for i, (name, description, color) in enumerate(algorithms):
        y_pos = 220 + i * 80
        
        # Highlight selected algorithm
        if i == selected_algorithm:
            pygame.draw.rect(screen, white, [150, y_pos-5, 306, 60], 2)
        
        # Algorithm name
        algo_text = menu_font.render(name, True, color)
        screen.blit(algo_text, [200, y_pos])
        
        # Description
        desc_text = font.render(description, True, white)
        screen.blit(desc_text, [160, y_pos + 35])
    
    # Instructions
    instructions = font.render("Use UP/DOWN arrows to select, ENTER to start", True, yellow)
    screen.blit(instructions, [100, 550])
    
    pygame.display.flip()

def algorithm_menu():
    selected = 0
    algorithms = ["A*", "GBFS", "UCS"]
    
    while True:
        draw_menu(selected)
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return None
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    selected = (selected - 1) % len(algorithms)
                elif event.key == pygame.K_DOWN:
                    selected = (selected + 1) % len(algorithms)
                elif event.key == pygame.K_RETURN:
                    return algorithms[selected]
                elif event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    return None
        
        clock.tick(10)

def startGame(algorithm="A*"):
    all_sprites_list = pygame.sprite.RenderPlain()
    block_list = pygame.sprite.RenderPlain()
    monsta_list = pygame.sprite.RenderPlain()
    pacman_collide = pygame.sprite.RenderPlain()

    wall_list = setupRoomOne(all_sprites_list)
    gate = setupGate(all_sprites_list)

    p_turn = 0
    p_steps = 0
    b_turn = 0
    b_steps = 0
    i_turn = 0
    i_steps = 0
    c_turn = 0
    c_steps = 0

    # Create the AI Pacman
    Pacman = AIPacman(w, p_h, "images/Trollman.png", algorithm)
    all_sprites_list.add(Pacman)
    pacman_collide.add(Pacman)
   
    # Blinky=Ghost(w, b_h, "images/Blinky.png")
    # monsta_list.add(Blinky)
    # all_sprites_list.add(Blinky)

    # Pinky=Ghost(w, m_h, "images/Pinky.png")
    # monsta_list.add(Pinky)
    # all_sprites_list.add(Pinky)
   
    Inky=Ghost(i_w, m_h, "images/Inky.png")
    monsta_list.add(Inky)
    all_sprites_list.add(Inky)
   
    Clyde=Ghost(c_w, m_h, "images/Clyde.png")
    monsta_list.add(Clyde)
    all_sprites_list.add(Clyde)

    # Draw the grid
    for row in range(19):
        for column in range(19):
            if (row == 7 or row == 8) and (column == 8 or column == 9 or column == 10):
                continue
            else:
                block = Block(yellow, 4, 4)
                block.rect.x = (30*column+6)+26
                block.rect.y = (30*row+6)+26

                b_collide = pygame.sprite.spritecollide(block, wall_list, False)
                p_collide = pygame.sprite.spritecollide(block, pacman_collide, False)
                if b_collide:
                    continue
                elif p_collide:
                    continue
                else:
                    block_list.add(block)
                    all_sprites_list.add(block)

    bll = len(block_list)
    score = 0
    done = False
    start_time = pygame.time.get_ticks()
    survival_time = 0

    # Statistics
    dots_collected = 0
    ghosts_avoided = 0
    algorithm_changes = 0

    while done == False:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

            # Allow algorithm switching during gameplay for comparison
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1:
                    Pacman.set_algorithm("A*")
                    algorithm_changes += 1
                elif event.key == pygame.K_2:
                    Pacman.set_algorithm("GBFS")
                    algorithm_changes += 1
                elif event.key == pygame.K_3:
                    Pacman.set_algorithm("UCS")
                    algorithm_changes += 1
                elif event.key == pygame.K_m:
                    # Return to menu
                    return "menu"
                elif event.key == pygame.K_ESCAPE: 
                    pygame.quit()

        # Update game state
        Pacman.update(wall_list, gate, block_list, monsta_list)

        # returned = Pinky.changespeed(Pinky_directions, False, p_turn, p_steps, pl)
        # p_turn = returned[0]
        # p_steps = returned[1]
        # Pinky.changespeed(Pinky_directions, False, p_turn, p_steps, pl)
        # Pinky.update(wall_list, False)

        # returned = Blinky.changespeed(Blinky_directions, False, b_turn, b_steps, bl)
        # b_turn = returned[0]
        # b_steps = returned[1]
        # Blinky.changespeed(Blinky_directions, False, b_turn, b_steps, bl)
        # Blinky.update(wall_list, False)

        returned = Inky.changespeed(Inky_directions, False, i_turn, i_steps, il)
        i_turn = returned[0]
        i_steps = returned[1]
        Inky.changespeed(Inky_directions, False, i_turn, i_steps, il)
        Inky.update(wall_list, False)

        returned = Clyde.changespeed(Clyde_directions, "clyde", c_turn, c_steps, cl)
        c_turn = returned[0]
        c_steps = returned[1]
        Clyde.changespeed(Clyde_directions, "clyde", c_turn, c_steps, cl)
        Clyde.update(wall_list, False)

        # Check for dot collection
        blocks_hit_list = pygame.sprite.spritecollide(Pacman, block_list, True)
        if len(blocks_hit_list) > 0:
            score += len(blocks_hit_list)
            dots_collected += len(blocks_hit_list)
            Pacman.current_target = None  # <<< ADD THIS LINE

            # When a dot is collected, clear the current path so we recalculate
            Pacman.current_path = []
            Pacman.path_index = 0

        # Update survival time
        survival_time = (pygame.time.get_ticks() - start_time) // 1000

        # Draw everything
        screen.fill(black)
        wall_list.draw(screen)
        gate.draw(screen)
        all_sprites_list.draw(screen)
        monsta_list.draw(screen)

        # Display statistics
        text = font.render(f"Score: {score}/{bll}", True, red)
        screen.blit(text, [10, 10])
        
        current_algo = Pacman.search_algorithm
        algo_status = current_algo
        if Pacman.current_path:
            algo_status = f"{current_algo} (Pathfinding: {len(Pacman.current_path)-Pacman.path_index} steps)"

            
        algorithm_text = font.render(f"Algorithm: {algo_status}", True, green)
        screen.blit(algorithm_text, [10, 40])
        
        time_text = font.render(f"Time: {survival_time}s", True, purple)
        screen.blit(time_text, [10, 70])
        
        stats_text = font.render(f"Dots: {dots_collected} Ghosts Avoided: {ghosts_avoided}", True, yellow)
        screen.blit(stats_text, [10, 100])
        
        # Show ghost avoidance status
        if Pacman.is_running_away:
            ghosts_avoided += 1
            status_text = font.render("Status: RUNNING FROM GHOST!", True, red)
        else:
            status_text = font.render("Status: Collecting dots", True, green)
        screen.blit(status_text, [10, 130])
        
        # Show controls
        controls_text = font.render("Press M for Menu, 1/2/3 to switch algorithms", True, cyan)
        screen.blit(controls_text, [10, 570])

        # Check for win condition
        if score == bll:
            result = doNext("Congratulations, you won!", 145, all_sprites_list, block_list, 
                   monsta_list, pacman_collide, wall_list, gate, survival_time, 
                   dots_collected, ghosts_avoided, algorithm_changes, algorithm)
            if result == "menu":
                return "menu"

        # Check for ghost collision
        monsta_hit_list = pygame.sprite.spritecollide(Pacman, monsta_list, False)
        if monsta_hit_list:
            result = doNext("Game Over", 235, all_sprites_list, block_list, 
                    monsta_list, pacman_collide, wall_list, gate, survival_time, 
                    dots_collected, ghosts_avoided, algorithm_changes, algorithm)
            if result == "menu":
                return "menu"

        pygame.display.flip()
        clock.tick(10)
    
    return "quit"

def doNext(message, left, all_sprites_list, block_list, monsta_list, 
           pacman_collide, wall_list, gate, survival_time, dots_collected, 
           ghosts_avoided, algorithm_changes, algorithm):
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return "quit"
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    return "quit"
                if event.key == pygame.K_RETURN:
                    del all_sprites_list
                    del block_list
                    del monsta_list
                    del pacman_collide
                    del wall_list
                    del gate
                    return "restart"
                if event.key == pygame.K_m:
                    return "menu"

        # Grey background
        w = pygame.Surface((450, 350))
        w.set_alpha(10)
        w.fill((128, 128, 128))
        screen.blit(w, (80, 150))

        # Game over message
        text1 = font.render(message, True, white)
        screen.blit(text1, [left, 180])

        # Statistics
        text2 = font.render(f"Algorithm: {algorithm}", True, white)
        screen.blit(text2, [150, 220])
        
        text3 = font.render(f"Survival Time: {survival_time} seconds", True, white)
        screen.blit(text3, [150, 250])
        
        text4 = font.render(f"Dots Collected: {dots_collected}", True, white)
        screen.blit(text4, [150, 280])
        
        text5 = font.render(f"Ghosts Avoided: {ghosts_avoided}", True, white)
        screen.blit(text5, [150, 310])
        
        text6 = font.render(f"Algorithm Changes: {algorithm_changes}", True, white)
        screen.blit(text6, [150, 340])

        # Instructions
        text7 = font.render("Press ENTER to play again with same algorithm", True, white)
        screen.blit(text7, [100, 380])
        
        text8 = font.render("Press M to return to main menu", True, white)
        screen.blit(text8, [150, 410])
        
        text9 = font.render("Press ESCAPE to quit", True, white)
        screen.blit(text9, [180, 440])

        pygame.display.flip()
        clock.tick(10)

# Main game loop
def main():
    while True:
        # Show algorithm selection menu
        algorithm = algorithm_menu()
        if algorithm is None:
            break
        
        # Start the game with selected algorithm
        result = startGame(algorithm)
        
        if result == "quit":
            break
        elif result == "menu":
            continue  # Go back to menu

if __name__ == "__main__":
    main()
    pygame.quit()