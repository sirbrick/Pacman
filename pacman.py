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
# Create grid for Pathfinding AI implementation below
def generate_grid(wall_list, cell_size=30, grid_size=19):
   grid = [[0 for i in range(grid_size)] for j in range(grid_size)]
   for walls in wall_list:
      x = walls.rect.left//cell_size
      y = walls.rect.top//cell_size
      #below creates walls
      if 0 <= x < grid_size and 0 <= y < grid_size:
         grid[y][x] = 1 
      return grid
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
        self.grid_offset = 6  # Pacman's grid offset
        self.dot_offset = 32  # Dots are offset by 32 pixels (6 + 26)
        
    def align_to_grid(self, x, y, is_dot=False):
        """Align coordinates to the nearest valid grid position"""
        # Use different offset for dots vs Pacman
        offset = self.dot_offset if is_dot else self.grid_offset
        
        # Convert to grid coordinates
        grid_x = (x - offset) // self.block_size  # Use floor division
        grid_y = (y - offset) // self.block_size
        
        # Convert back to pixel coordinates
        aligned_x = grid_x * self.block_size + offset
        aligned_y = grid_y * self.block_size + offset
        
        # Ensure it's within bounds
        aligned_x = max(6, min(aligned_x, 570))
        aligned_y = max(6, min(aligned_y, 570))
        
        return (aligned_x, aligned_y)
    
    def get_grid_cell(self, x, y, is_dot=False):
        """Get the grid cell coordinates for a position"""
        offset = self.dot_offset if is_dot else self.grid_offset
        grid_x = (x - offset) // self.block_size
        grid_y = (y - offset) // self.block_size
        return (grid_x, grid_y)
    
    def convert_dot_to_pacman_grid(self, dot_pos):
        """Convert dot position to the nearest Pacman-accessible position"""
        # Get the grid cell the dot is in (using dot offset)
        dot_grid_x, dot_grid_y = self.get_grid_cell(dot_pos[0], dot_pos[1], is_dot=True)
        
        # Convert to Pacman's grid (same grid cell, Pacman offset)
        pacman_x = dot_grid_x * self.block_size + self.grid_offset
        pacman_y = dot_grid_y * self.block_size + self.grid_offset
        
        return (pacman_x, pacman_y)
    
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
    
    def get_neighbors(self, x, y):
        neighbors = []
        directions = [
            (self.block_size, 0),   # Right
            (-self.block_size, 0),  # Left  
            (0, self.block_size),   # Down
            (0, -self.block_size)   # Up
        ]
        
        for dx, dy in directions:
            new_x, new_y = x + dx, y + dy
            if self.is_valid_position(new_x, new_y):
                neighbors.append((new_x, new_y))
        return neighbors
    
    def manhattan_distance(self, x1, y1, x2, y2):
        # Calculate grid distance using Pacman's grid
        grid_x1 = (x1 - self.grid_offset) // self.block_size
        grid_y1 = (y1 - self.grid_offset) // self.block_size
        grid_x2 = (x2 - self.grid_offset) // self.block_size
        grid_y2 = (y2 - self.grid_offset) // self.block_size
        return abs(grid_x1 - grid_x2) + abs(grid_y1 - grid_y2)
    
    def a_star_search(self, start, goal):
        # Align start and goal to Pacman's grid (not dot grid)
        aligned_start = self.align_to_grid(start[0], start[1], is_dot=False)
        
        # Check if goal is a dot position - if so, convert it to Pacman grid
        # We'll check by seeing if it aligns with dot offset
        if abs((goal[0] - self.dot_offset) % self.block_size) < 5 or \
           abs((goal[1] - self.dot_offset) % self.block_size) < 5:
            # This looks like a dot position, convert it
            aligned_goal = self.convert_dot_to_pacman_grid(goal)
        else:
            # Already a Pacman grid position
            aligned_goal = self.align_to_grid(goal[0], goal[1], is_dot=False)
        
        print(f"A*: From {start}->{aligned_start} to {goal}->{aligned_goal}")
        
        # If start and goal are the same or very close, return empty path
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
                
            for neighbor in self.get_neighbors(current[0], current[1]):
                if neighbor in visited:
                    continue
                    
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.manhattan_distance(
                        neighbor[0], neighbor[1], aligned_goal[0], aligned_goal[1])
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return []  # No path found
    
    def greedy_best_first_search(self, start, goal):
        # Align start to Pacman grid
        aligned_start = self.align_to_grid(start[0], start[1], is_dot=False)
        
        # Convert goal if it's a dot position
        if abs((goal[0] - self.dot_offset) % self.block_size) < 5 or \
           abs((goal[1] - self.dot_offset) % self.block_size) < 5:
            aligned_goal = self.convert_dot_to_pacman_grid(goal)
        else:
            aligned_goal = self.align_to_grid(goal[0], goal[1], is_dot=False)
        
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
                
            for neighbor in self.get_neighbors(current[0], current[1]):
                if neighbor not in visited:
                    came_from[neighbor] = current
                    priority = self.manhattan_distance(neighbor[0], neighbor[1], aligned_goal[0], aligned_goal[1])
                    heapq.heappush(open_set, (priority, neighbor))
        
        return []  # No path found
    
    def uniform_cost_search(self, start, goal):
        # Align start to Pacman grid
        aligned_start = self.align_to_grid(start[0], start[1], is_dot=False)
        
        # Convert goal if it's a dot position
        if abs((goal[0] - self.dot_offset) % self.block_size) < 5 or \
           abs((goal[1] - self.dot_offset) % self.block_size) < 5:
            aligned_goal = self.convert_dot_to_pacman_grid(goal)
        else:
            aligned_goal = self.align_to_grid(goal[0], goal[1], is_dot=False)
        
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
                
            for neighbor in self.get_neighbors(current[0], current[1]):
                new_cost = cost_so_far[current] + 1
                
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    came_from[neighbor] = current
                    heapq.heappush(open_set, (new_cost, neighbor))
        
        return []  # No path found

class AIPacman(Player):
    def __init__(self, x, y, filename, algorithm=None):
        super().__init__(x, y, filename)
        self.algorithm = algorithm
        self.last_path_update = 0
        self.path_update_delay = 100
        self.current_target = None
        self.last_positions = []  # Track recent positions to detect oscillation
        self.last_direction = None  # Track last movement direction
        self.ghost_danger_distance = 120  # Distance at which ghosts become dangerous
        self.is_running_away = False  # Whether Pacman is currently running from ghosts
        self.safe_return_timer = 0  # Timer to return to normal after escaping
        self.current_path = []  # Store the current path from search algorithm
        self.path_index = 0  # Current position in the path
        self.oscillation_cooldown = 0  # Cooldown timer for oscillation detection
        self.stuck_counter = 0  # Counter to detect when stuck
        self.last_successful_move = None  # Last direction that worked
        self.dot_collection_distance = 15  # Distance at which we consider a dot "collected"

        
    def set_algorithm(self, algorithm):
        """Change the search algorithm"""
        self.algorithm = algorithm
        self.current_path = []  # Clear current path when algorithm changes
        self.path_index = 0
        print(f"Algorithm changed to {algorithm}")
        
    def update(self, walls, gate, block_list, monsta_list):
        current_time = pygame.time.get_ticks()
        
        # Update timers
        if self.oscillation_cooldown > 0:
            self.oscillation_cooldown -= 1
            
        # Update safe return timer
        if self.is_running_away:
            self.safe_return_timer += 1
            if self.safe_return_timer > 30:  # Run away for 30 updates (~3 seconds)
                self.is_running_away = False
                self.safe_return_timer = 0
                self.current_path = []  # Clear path when returning to normal
                self.path_index = 0
                print("Returning to normal dot collection")
        
        # Check for nearby ghosts
        closest_ghost, ghost_distance = self.get_closest_ghost(monsta_list)
        
        # If ghost is too close, run away
        if ghost_distance < self.ghost_danger_distance:
            if not self.is_running_away:
                print(f"GHOST DANGER! Ghost at distance {ghost_distance}, running away!")
                self.is_running_away = True
                self.safe_return_timer = 0
                self.current_path = []  # Clear path when running away
                self.path_index = 0
            
            # Run away from the closest ghost
            self.run_away_from_ghost(closest_ghost, walls)
        else:
            # If we have a path, follow it
            if self.current_path and self.path_index < len(self.current_path):
                self.follow_path(walls)
            else:
                # ALWAYS try to find a direction when not running away and no path
                # Remove the timer restriction so Pacman keeps moving
                if not self.is_running_away:
                    self.find_direction_to_closest_dot(block_list, walls)
                    # Still update the timer for UI purposes, but don't rely on it for movement
                    if current_time - self.last_path_update > self.path_update_delay:
                        self.last_path_update = current_time
        
        # Track position for oscillation detection
        current_pos = (self.rect.left, self.rect.top)
        self.last_positions.append(current_pos)
        if len(self.last_positions) > 15:  # Keep last 15 positions
            self.last_positions.pop(0)
        
        # Call parent update to handle collisions
        super().update(walls, gate)
    
    def follow_path(self, walls):
        """Follow the current path from search algorithm"""
        if self.path_index >= len(self.current_path):
            return
            
        target_pos = self.current_path[self.path_index]
        current_pos = (self.rect.left, self.rect.top)
        
        # Calculate distance to target
        dist_x = abs(target_pos[0] - current_pos[0])
        dist_y = abs(target_pos[1] - current_pos[1])
        
        # Increase tolerance (from 5 to 10 pixels) and consider the direction
        # If we're moving in the right direction and close enough, advance to next waypoint
        if dist_x < 15 and dist_y < 15:
            self.path_index += 1
            if self.path_index >= len(self.current_path):
                self.current_path = []  # Path completed
                self.path_index = 0
                return
        
        # If still have a target, move toward it
        if self.path_index < len(self.current_path):
            target_pos = self.current_path[self.path_index]
            self.move_to_position(target_pos)
    
    def get_closest_ghost(self, monsta_list):
        """Find the closest ghost and its distance"""
        if not monsta_list:
            return None, float('inf')
            
        closest_ghost = None
        min_distance = float('inf')
        
        for ghost in monsta_list:
            ghost_pos = (ghost.rect.centerx, ghost.rect.centery)
            current_pos = (self.rect.centerx, self.rect.centery)
            distance = abs(ghost_pos[0] - current_pos[0]) + abs(ghost_pos[1] - current_pos[1])
            
            if distance < min_distance:
                min_distance = distance
                closest_ghost = ghost
        
        return closest_ghost, min_distance
    
    def run_away_from_ghost(self, ghost, walls):
        """Run away from the closest ghost"""
        if not ghost:
            return
            
        ghost_pos = (ghost.rect.centerx, ghost.rect.centery)
        current_pos = (self.rect.centerx, self.rect.centery)
        
        # Calculate direction AWAY from the ghost
        dx = current_pos[0] - ghost_pos[0]  # Opposite of direction to ghost
        dy = current_pos[1] - ghost_pos[1]
        
        # Reset movement
        self.change_x = 0
        self.change_y = 0
        
        # Try to move in the safest direction (away from ghost)
        moved = False
        
        # Prefer the direction that puts most distance between us and the ghost
        if abs(dx) > abs(dy):
            # Try to move horizontally away from ghost
            if dx > 0:  # Ghost is to our left, move right
                if self.can_move_right(walls):
                    self.change_x = 30
                    moved = True
                    self.last_direction = "right"
                    self.last_successful_move = "right"
            else:  # Ghost is to our right, move left
                if self.can_move_left(walls):
                    self.change_x = -30
                    moved = True
                    self.last_direction = "left"
                    self.last_successful_move = "left"
            
            # If horizontal escape blocked, try vertical
            if not moved:
                if dy > 0:  # Ghost is above us, move down
                    if self.can_move_down(walls):
                        self.change_y = 30
                        moved = True
                        self.last_direction = "down"
                        self.last_successful_move = "down"
                else:  # Ghost is below us, move up
                    if self.can_move_up(walls):
                        self.change_y = -30
                        moved = True
                        self.last_direction = "up"
                        self.last_successful_move = "up"
        else:
            # Try to move vertically away from ghost
            if dy > 0:  # Ghost is above us, move down
                if self.can_move_down(walls):
                    self.change_y = 30
                    moved = True
                    self.last_direction = "down"
                    self.last_successful_move = "down"
            else:  # Ghost is below us, move up
                if self.can_move_up(walls):
                    self.change_y = -30
                    moved = True
                    self.last_direction = "up"
                    self.last_successful_move = "up"
            
            # If vertical escape blocked, try horizontal
            if not moved:
                if dx > 0:  # Ghost is to our left, move right
                    if self.can_move_right(walls):
                        self.change_x = 30
                        moved = True
                        self.last_direction = "right"
                        self.last_successful_move = "right"
                else:  # Ghost is to our right, move left
                    if self.can_move_left(walls):
                        self.change_x = -30
                        moved = True
                        self.last_direction = "left"
                        self.last_successful_move = "left"
        
        # If we couldn't move directly away, try any safe direction
        if not moved:
            self.find_safe_direction(ghost, walls)
    
    def find_safe_direction(self, ghost, walls):
        """Find the safest direction when directly away is blocked"""
        ghost_pos = (ghost.rect.centerx, ghost.rect.centery)
        current_pos = (self.rect.centerx, self.rect.centery)
        
        # Try all directions and pick the one that maximizes distance from ghost
        best_direction = None
        max_distance = -1
        
        directions = [
            (30, 0, "right"),
            (-30, 0, "left"),
            (0, 30, "down"),
            (0, -30, "up")
        ]
        
        for dx, dy, direction_name in directions:
            test_x = self.rect.left + dx
            test_y = self.rect.top + dy
            test_sprite = pygame.sprite.Sprite()
            test_sprite.rect = pygame.Rect(test_x, test_y, 30, 30)
            
            # Check if this direction is valid (no walls)
            if not pygame.sprite.spritecollide(test_sprite, walls, False):
                # Calculate distance from ghost if we move here
                new_pos = (test_x + 15, test_y + 15)  # Center of new position
                distance = abs(new_pos[0] - ghost_pos[0]) + abs(new_pos[1] - ghost_pos[1])
                
                # Bonus for directions different from last successful move
                if direction_name != self.last_successful_move:
                    distance += 20  # Bonus for trying something different
                
                if distance > max_distance:
                    max_distance = distance
                    best_direction = (dx, dy, direction_name)
        
        # Move in the safest direction
        if best_direction:
            dx, dy, direction_name = best_direction
            self.change_x = dx
            self.change_y = dy
            self.last_direction = direction_name
            self.last_successful_move = direction_name
            print(f"Moving {direction_name} to escape ghost (distance: {max_distance})")
        else:
            # No safe direction, try any direction as last resort
            self.try_any_direction(walls)
    
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
        if self.is_running_away:
            return  # Don't look for dots while running away
            
        if not block_list:
            self.current_target = None
            return
            
        current_pos = (self.rect.left, self.rect.top)
        closest_dot = None
        min_distance = float('inf')
        
        # Find closest dot
        for dot in block_list:
            dot_pos = (dot.rect.centerx, dot.rect.centery)
            distance = abs(dot_pos[0] - current_pos[0]) + abs(dot_pos[1] - current_pos[1])
            
            if distance < min_distance:
                min_distance = distance
                closest_dot = dot_pos
        
        if closest_dot:
            self.current_target = closest_dot
            
            # Check for oscillation - if we're moving back and forth
            if self.is_oscillating():
                print("Detected oscillation! Trying different approach...")
                self.avoid_oscillation(walls)
                return
            
            # Use search algorithm if one is selected
            if self.algorithm and self.algorithm in ["A*", "GBFS", "UCS"]:
                self.use_search_algorithm(current_pos, closest_dot, walls)
            else:
                # Use simple movement if no algorithm selected
                self.simple_move_toward_dot(current_pos, closest_dot, walls)
        else:
            self.current_target = None
    
    def use_search_algorithm(self, current_pos, target_dot_pos, walls):
        """Use the selected search algorithm to find a path to a dot"""
        searcher = SearchAlgorithm(walls)
        
        print(f"Looking for path from Pacman at {current_pos} to dot at {target_dot_pos}")
        
        path = []
        
        if self.algorithm == "A*":
            path = searcher.a_star_search(current_pos, target_dot_pos)
        elif self.algorithm == "GBFS":
            path = searcher.greedy_best_first_search(current_pos, target_dot_pos)
        elif self.algorithm == "UCS":
            path = searcher.uniform_cost_search(current_pos, target_dot_pos)
        
        if path and len(path) > 0:
            self.current_path = path
            self.path_index = 0
            print(f"{self.algorithm} found path with {len(path)} steps")
        else:
            # Check if we're already close to the dot
            current_center = (self.rect.centerx, self.rect.centery)
            distance_to_dot = abs(current_center[0] - target_dot_pos[0]) + abs(current_center[1] - target_dot_pos[1])
            
            if distance_to_dot < 40:
                print(f"Already close to dot ({distance_to_dot} pixels), not using pathfinding")
                self.current_path = []
                self.path_index = 0
            else:
                print(f"{self.algorithm} found no path, using simple movement")
                self.simple_move_toward_dot(current_pos, target_dot_pos, walls)
    
    def find_direction_to_closest_dot(self, block_list, walls):
        if self.is_running_away:
            return
            
        if not block_list:
            self.current_target = None
            return
            
        current_pos = (self.rect.centerx, self.rect.centery)
        closest_dot = None
        closest_dot_pos = None
        min_distance = float('inf')
        
        # Find closest dot that's not too close (otherwise we should already collect it)
        for dot in block_list:
            dot_pos = (dot.rect.centerx, dot.rect.centery)
            distance = abs(dot_pos[0] - current_pos[0]) + abs(dot_pos[1] - current_pos[1])
            
            # Skip dots that are very close (should be collected by collision)
            if distance < self.dot_collection_distance:
                continue
                
            # Skip dots that are too far to bother with pathfinding
            if distance > 300:  # Optional: limit search radius
                continue
                
            if distance < min_distance:
                min_distance = distance
                closest_dot = dot
                closest_dot_pos = dot_pos
        
        if closest_dot_pos:
            self.current_target = closest_dot_pos
            
            # Check if we're already very close
            if min_distance < 30:
                print(f"Very close to dot ({min_distance} pixels), using simple movement")
                self.simple_move_toward_dot(current_pos, closest_dot_pos, walls)
            elif self.algorithm and self.algorithm in ["A*", "GBFS", "UCS"]:
                self.use_search_algorithm(current_pos, closest_dot_pos, walls)
            else:
                self.simple_move_toward_dot(current_pos, closest_dot_pos, walls)
        else:
            # No suitable dots found, try any dot
            for dot in block_list:
                dot_pos = (dot.rect.centerx, dot.rect.centery)
                self.simple_move_toward_dot(current_pos, dot_pos, walls)
                break
    def simple_move_toward_dot(self, current_pos, target_pos, walls):
        """Original simple movement logic (as fallback)"""
        # Calculate direction to dot
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        
        # Reset movement
        self.change_x = 0
        self.change_y = 0
        
        # Try to move in the primary preferred direction
        moved = False
        
        if abs(dx) > abs(dy):
            # Prefer horizontal movement
            if dx > 0:
                if self.can_move_right(walls):
                    self.change_x = 30
                    moved = True
                    self.last_direction = "right"
                    self.last_successful_move = "right"
            else:  # dx < 0
                if self.can_move_left(walls):
                    self.change_x = -30
                    moved = True
                    self.last_direction = "left"
                    self.last_successful_move = "left"
            
            # If horizontal movement failed, try vertical
            if not moved:
                if dy > 0:
                    if self.can_move_down(walls):
                        self.change_y = 30
                        moved = True
                        self.last_direction = "down"
                        self.last_successful_move = "down"
                elif dy < 0:
                    if self.can_move_up(walls):
                        self.change_y = -30
                        moved = True
                        self.last_direction = "up"
                        self.last_successful_move = "up"
        else:
            # Prefer vertical movement
            if dy > 0:
                if self.can_move_down(walls):
                    self.change_y = 30
                    moved = True
                    self.last_direction = "down"
                    self.last_successful_move = "down"
            elif dy < 0:
                if self.can_move_up(walls):
                    self.change_y = -30
                    moved = True
                    self.last_direction = "up"
                    self.last_successful_move = "up"
            
            # If vertical movement failed, try horizontal
            if not moved:
                if dx > 0:
                    if self.can_move_right(walls):
                        self.change_x = 30
                        moved = True
                        self.last_direction = "right"
                        self.last_successful_move = "right"
                elif dx < 0:
                    if self.can_move_left(walls):
                        self.change_x = -30
                        moved = True
                        self.last_direction = "left"
                        self.last_successful_move = "left"
        
        # Debug output
        if moved:
            direction = ""
            if self.change_x > 0:
                direction = "right"
            elif self.change_x < 0:
                direction = "left"
            elif self.change_y > 0:
                direction = "down"
            elif self.change_y < 0:
                direction = "up"
            print(f"Moving {direction} towards dot at {target_pos}")
        else:
            # Can't move in preferred direction, try any direction
            self.try_any_direction(walls)
    
    def move_to_position(self, target_pos):
        """Move toward a specific position (used for path following)"""
        current_pos = (self.rect.left, self.rect.top)
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        
        # Reset movement
        self.change_x = 0
        self.change_y = 0
        
        # Determine which direction to move
        if abs(dx) > abs(dy):
            if dx > 0:
                self.change_x = 30
                self.last_direction = "right"
                self.last_successful_move = "right"
            else:
                self.change_x = -30
                self.last_direction = "left"
                self.last_successful_move = "left"
        else:
            if dy > 0:
                self.change_y = 30
                self.last_direction = "down"
                self.last_successful_move = "down"
            else:
                self.change_y = -30
                self.last_direction = "up"
                self.last_successful_move = "up"
    
    def is_oscillating(self):
        """Check if Pacman is oscillating (moving back and forth)"""
        if self.oscillation_cooldown > 0:
            return False
            
        if len(self.last_positions) < 10:  # Need at least 10 positions to check
            return False
        
        # Check if we're visiting the same positions repeatedly
        unique_positions = set(self.last_positions)
        if len(unique_positions) < 4:  # Only 4 or fewer unique positions = likely oscillation
            self.oscillation_cooldown = 25  # Set cooldown
            return True
        
        # Check for consistent back-and-forth pattern
        positions = self.last_positions[-8:]  # Check last 8 positions
        
        # Check horizontal oscillation
        x_values = [pos[0] for pos in positions]
        if len(set(x_values)) <= 2 and x_values[-1] == x_values[-3] == x_values[-5]:
            if x_values[-2] == x_values[-4] == x_values[-6]:
                if abs(x_values[-1] - x_values[-2]) == 30:  # Moving 30 pixels back and forth
                    self.oscillation_cooldown = 30
                    return True
        
        # Check vertical oscillation
        y_values = [pos[1] for pos in positions]
        if len(set(y_values)) <= 2 and y_values[-1] == y_values[-3] == y_values[-5]:
            if y_values[-2] == y_values[-4] == y_values[-6]:
                if abs(y_values[-1] - y_values[-2]) == 30:  # Moving 30 pixels back and forth
                    self.oscillation_cooldown = 30
                    return True
        
        return False
    
    def avoid_oscillation(self, walls):
        """Try to break out of oscillation pattern"""
        print("Attempting to break oscillation...")
        
        # Get current position
        current_pos = (self.rect.left, self.rect.top)
        
        # Try to find a direction that leads to a new area
        directions = [
            (30, 0, "right"),
            (-30, 0, "left"),
            (0, 30, "down"),
            (0, -30, "up")
        ]
        
        # Score each direction
        scored_directions = []
        
        for dx, dy, direction_name in directions:
            test_x = self.rect.left + dx
            test_y = self.rect.top + dy
            test_sprite = pygame.sprite.Sprite()
            test_sprite.rect = pygame.Rect(test_x, test_y, 30, 30)
            
            if not pygame.sprite.spritecollide(test_sprite, walls, False):
                score = 0
                
                # High score for directions opposite to recent movement
                if direction_name != self.last_direction:
                    score += 3
                
                # Bonus for directions that haven't been tried recently
                new_pos = (test_x, test_y)
                if new_pos not in self.last_positions[-6:]:
                    score += 5
                
                # Bonus for directions different from last successful move
                if direction_name != self.last_successful_move:
                    score += 2
                
                scored_directions.append((score, dx, dy, direction_name))
        
        # Sort by score (highest first)
        scored_directions.sort(reverse=True, key=lambda x: x[0])
        
        # Try the best scoring direction
        if scored_directions:
            _, dx, dy, direction_name = scored_directions[0]
            self.change_x = dx
            self.change_y = dy
            self.last_direction = direction_name
            self.last_successful_move = direction_name
            print(f"Breaking oscillation by moving {direction_name}")
            return
        
        # If no direction found, try any direction
        self.try_any_direction(walls)
    
    def try_any_direction(self, walls):
        """Try any available direction when completely stuck"""
        directions = [
            (30, 0, "right"),
            (-30, 0, "left"),
            (0, 30, "down"),
            (0, -30, "up")
        ]
        
        # Try directions in random order
        random.shuffle(directions)
        
        for dx, dy, direction_name in directions:
            test_x = self.rect.left + dx
            test_y = self.rect.top + dy
            test_sprite = pygame.sprite.Sprite()
            test_sprite.rect = pygame.Rect(test_x, test_y, 30, 30)
            
            if not pygame.sprite.spritecollide(test_sprite, walls, False):
                self.change_x = dx
                self.change_y = dy
                self.last_direction = direction_name
                self.last_successful_move = direction_name
                print(f"Moving {direction_name} as last resort")
                return
        
        # If no direction works, stay still
        self.change_x = 0
        self.change_y = 0
        print("Completely stuck! No direction available")
        self.stuck_counter += 1
        
        # If stuck for too long, clear position history
        if self.stuck_counter > 10:
            self.last_positions = []
            self.stuck_counter = 0
    
    def get_closest_dot(self, block_list, current_pos):
        if not block_list:
            return None
            
        closest_dot = None
        min_distance = float('inf')
        
        for dot in block_list:
            dot_pos = (dot.rect.centerx, dot.rect.centery)
            distance = abs(dot_pos[0] - current_pos[0]) + abs(dot_pos[1] - current_pos[1])
            
            if distance < min_distance:
                min_distance = distance
                closest_dot = dot_pos
                
        return closest_dot

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
   
    Blinky=Ghost(w, b_h, "images/Blinky.png")
    monsta_list.add(Blinky)
    all_sprites_list.add(Blinky)

    Pinky=Ghost(w, m_h, "images/Pinky.png")
    monsta_list.add(Pinky)
    all_sprites_list.add(Pinky)
   
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
                block = Block(red, 4, 4)
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

        returned = Pinky.changespeed(Pinky_directions, False, p_turn, p_steps, pl)
        p_turn = returned[0]
        p_steps = returned[1]
        Pinky.changespeed(Pinky_directions, False, p_turn, p_steps, pl)
        Pinky.update(wall_list, False)

        returned = Blinky.changespeed(Blinky_directions, False, b_turn, b_steps, bl)
        b_turn = returned[0]
        b_steps = returned[1]
        Blinky.changespeed(Blinky_directions, False, b_turn, b_steps, bl)
        Blinky.update(wall_list, False)

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
        
        current_algo = Pacman.algorithm
        if Pacman.current_path:
            algo_status = f"{current_algo} (Pathfinding: {len(Pacman.current_path)-Pacman.path_index} steps)"
        else:
            algo_status = f"{current_algo} (Simple)"
            
        algorithm_text = font.render(f"Algorithm: {algo_status}", True, green)
        screen.blit(algorithm_text, [10, 40])
        
        time_text = font.render(f"Time: {survival_time}s", True, purple)
        screen.blit(time_text, [10, 70])
        
        stats_text = font.render(f"Dots: {dots_collected} Ghosts Avoided: {ghosts_avoided}", True, yellow)
        screen.blit(stats_text, [10, 100])
        
        # Show ghost avoidance status
        if Pacman.is_running_away:
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
            # Count as avoided if Pacman was actively running away
            if Pacman.is_running_away:
                ghosts_avoided += 1
                print(f"Ghost avoided! Total avoided: {ghosts_avoided}")
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