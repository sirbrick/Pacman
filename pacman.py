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
# Fix the SearchAlgorithm class
class SearchAlgorithm:
    def __init__(self, walls, block_size=30):
        self.walls = walls
        self.block_size = block_size
        
    def is_valid_position(self, x, y):
        # Create a test sprite with Pacman's actual size
        test_sprite = pygame.sprite.Sprite()
        test_sprite.rect = pygame.Rect(x, y, 30, 30)  # Pacman's size
        return not pygame.sprite.spritecollide(test_sprite, self.walls, False)
    
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
            # Check bounds - make sure Pacman stays within screen
            if 0 <= new_x <= 576 and 0 <= new_y <= 576:
                if self.is_valid_position(new_x, new_y):
                    neighbors.append((new_x, new_y))
        return neighbors
    
    def manhattan_distance(self, x1, y1, x2, y2):
        return abs(x1 - x2) + abs(y1 - y2)
    
    def a_star_search(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.manhattan_distance(start[0], start[1], goal[0], goal[1])}
        
        while open_set:
            current_f, current = heapq.heappop(open_set)
            
            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path
                
            for neighbor in self.get_neighbors(current[0], current[1]):
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.manhattan_distance(
                        neighbor[0], neighbor[1], goal[0], goal[1])
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
                    
        return []  # No path found
    
    def greedy_best_first_search(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (self.manhattan_distance(start[0], start[1], goal[0], goal[1]), start))
        came_from = {}
        visited = set()
        
        while open_set:
            current_priority, current = heapq.heappop(open_set)
            
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path
                
            if current in visited:
                continue
            visited.add(current)
            
            for neighbor in self.get_neighbors(current[0], current[1]):
                if neighbor not in visited:
                    came_from[neighbor] = current
                    priority = self.manhattan_distance(neighbor[0], neighbor[1], goal[0], goal[1])
                    heapq.heappush(open_set, (priority, neighbor))
                    
        return []
    
    def uniform_cost_search(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        cost_so_far = {start: 0}
        
        while open_set:
            current_cost, current = heapq.heappop(open_set)
            
            if current == goal:
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
                    
        return []

# Fix the AIPacman update method
# Replace the AIPacman class with this fixed version:

class AIPacman(Player):
    def __init__(self, x, y, filename, algorithm= None):
        super().__init__(x, y, filename)
        self.algorithm = algorithm
        self.path = []
        self.current_target = None
        self.ghost_safety_distance = 120
        self.last_path_update = 0
        self.path_update_delay = 500  # ms between path recalculations
        self.stuck_timer = 0
        self.last_position = (x, y)
        
    def set_algorithm(self, algorithm):
        self.algorithm = algorithm
        self.path = []
        
    def update(self, walls, gate, block_list, monsta_list):
        current_time = pygame.time.get_ticks()
        current_pos = (self.rect.left, self.rect.top)    
        
        # Check if Pacman is stuck
        if current_pos == self.last_position:
            self.stuck_timer += 1
            if self.stuck_timer > 30:  # Reduced from 100 to respond faster
                self.path = []
                self.stuck_timer = 0
        else:
            self.stuck_timer = 0
            self.last_position = current_pos
        
        # Only recalculate path occasionally to improve performance
    
        self.find_new_target(block_list, monsta_list, walls)
        self.last_path_update = current_time
        
        # Reset movement before setting new direction
        self.change_x = 0
        self.change_y = 0
        
        # Follow the path
        if self.path and len(self.path) > 0:
            target_x, target_y = self.path[0]
            
            # Check if we're close enough to the target (within tolerance)
            distance_to_target = abs(target_x - self.rect.left) + abs(target_y - self.rect.top)
            
            if distance_to_target < 15:  # Tolerance for "reached" target
                self.path.pop(0)
                if not self.path:
                    return
            
            # Get next target
            if self.path:
                target_x, target_y = self.path[0]
                
                # Calculate exact direction needed
                dx = target_x - self.rect.left
                dy = target_y - self.rect.top
                
                # : Move in only ONE direction at a time, prioritize larger difference
                if abs(dx) > abs(dy):
                    # Horizontal movement
                    if dx > 0:
                        self.change_x = 30
                    else:
                        self.change_x = -30
                else:
                    # Vertical movement
                    if dy > 0:
                        self.change_y = 30
                    else:
                        self.change_y = -30
        
        # Call parent update to handle collisions
        super().update(walls, gate)
        
    def find_new_target(self, block_list, monsta_list, walls):
        current_pos = (self.rect.left, self.rect.top)
        
        # Emergency ghost avoidance
        closest_ghost, ghost_distance = self.get_closest_ghost(monsta_list)
        if ghost_distance < self.ghost_safety_distance :
            safe_pos = self.find_safe_position(monsta_list, walls)
            if safe_pos:
                self.current_target = safe_pos
            else:
                # If no safe position, pick the safest dot
                self.current_target = self.get_safest_dot(block_list, monsta_list, current_pos)
        else:
            # Normal dot collection
            self.current_target = self.get_closest_dot(block_list, current_pos)
        
        
        # Calculate path to target
        if self.current_target:
            searcher = SearchAlgorithm(walls)
            
            if self.algorithm == "A*":
                self.path = searcher.a_star_search(current_pos, self.current_target)
            elif self.algorithm == "GBFS":
                self.path = searcher.greedy_best_first_search(current_pos, self.current_target)
            elif self.algorithm == "UCS":
                self.path = searcher.uniform_cost_search(current_pos, self.current_target)

    
    def get_closest_dot(self, block_list, current_pos):
        if not block_list:
            return None
            
        closest_dot = None
        min_distance = float('inf')
        
        for dot in block_list:
            dot_pos = (dot.rect.left, dot.rect.top)
            distance = self.manhattan_distance(current_pos[0], current_pos[1], dot_pos[0], dot_pos[1])
            
            if distance < min_distance:
                min_distance = distance
                closest_dot = dot_pos
                
        return closest_dot
    
    def get_safest_dot(self, block_list, monsta_list, current_pos):
        if not block_list:
            return None
            
        safest_dot = None
        best_score = -float('inf')
        
        for dot in block_list:
            dot_pos = (dot.rect.left, dot.rect.top)
            
            # Score based on distance to Pacman and distance from ghosts
            dot_to_pacman = self.manhattan_distance(current_pos[0], current_pos[1], dot_pos[0], dot_pos[1])
            min_ghost_distance = float('inf')
            
            for ghost in monsta_list:
                ghost_pos = (ghost.rect.left, ghost.rect.top)
                ghost_dist = self.manhattan_distance(dot_pos[0], dot_pos[1], ghost_pos[0], ghost_pos[1])
                min_ghost_distance = min(min_ghost_distance, ghost_dist)
            
            # Higher score for closer dots and dots farther from ghosts
            score = min_ghost_distance - (dot_to_pacman * 2)
            
            if score > best_score:
                best_score = score
                safest_dot = dot_pos
                
        return safest_dot
    
    def get_closest_ghost(self, monsta_list):
        if not monsta_list:
            return None, float('inf')
            
        closest_ghost = None
        min_distance = float('inf')
        
        for ghost in monsta_list:
            ghost_pos = (ghost.rect.left, ghost.rect.top)
            distance = self.manhattan_distance(
                self.rect.left, self.rect.top, ghost_pos[0], ghost_pos[1])
            
            if distance < min_distance:
                min_distance = distance
                closest_ghost = ghost
                
        return closest_ghost, min_distance
    
    def find_safe_position(self, monsta_list, walls):
        current_pos = (self.rect.left, self.rect.top)
        searcher = SearchAlgorithm(walls)
        neighbors = searcher.get_neighbors(current_pos[0], current_pos[1])
        
        if not neighbors:
            return None
            
        safest_pos = None
        max_min_distance = -1
        
        for neighbor in neighbors:
            min_distance_to_ghost = float('inf')
            
            for ghost in monsta_list:
                ghost_pos = (ghost.rect.left, ghost.rect.top)
                distance = self.manhattan_distance(neighbor[0], neighbor[1], ghost_pos[0], ghost_pos[1])
                min_distance_to_ghost = min(min_distance_to_ghost, distance)
            
            if min_distance_to_ghost > max_min_distance:
                max_min_distance = min_distance_to_ghost
                safest_pos = neighbor
                
        return safest_pos
    
    def manhattan_distance(self, x1, y1, x2, y2):
        return abs(x2 - x1) + abs(y2 - y1)
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
            
        algorithm_text = font.render(f"Algorithm: {current_algo}", True, green)
        screen.blit(algorithm_text, [10, 40])
        
        time_text = font.render(f"Time: {survival_time}s", True, purple)
        screen.blit(time_text, [10, 70])
        
        stats_text = font.render(f"Dots: {dots_collected} Ghosts Avoided: {ghosts_avoided}", True, yellow)
        screen.blit(stats_text, [10, 100])
        
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
            if Pacman.current_target and Pacman.current_target != Pacman.get_closest_dot(block_list, (Pacman.rect.left, Pacman.rect.top)):
                ghosts_avoided += 1
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