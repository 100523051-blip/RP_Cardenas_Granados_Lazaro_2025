#!/usr/bin/python3

import sys
import random
from dataclasses import dataclass
from typing import List, Tuple,Dict
import pygame
import math
import rospy
from std_msgs.msg import String, Int64
from game_pkg.msg import user_msg
from game_pkg.srv import GetUserScore, GetUserScoreResponse, SetGameDifficulty, SetGameDifficultyResponse

WINDOW_TITLE = "Galaga (pygame)"
SCREEN_WIDTH = 480
SCREEN_HEIGHT = 640
FPS = 60

# Colors
COLOR_BACKGROUND = (10, 10, 18)
COLOR_TEXT = (235, 235, 235)
COLOR_PLAYER = (160, 32, 240)  # Purple player requirement (default)
COLOR_ENEMY = (230, 70, 70)
COLOR_ENEMY2 = (70, 200, 230)
COLOR_BULLET = (255, 240, 120)
COLOR_ENEMY_BULLET = (255, 120, 120)
COLOR_ITEM = (120, 255, 170)
MAX_LIVES = 3

# Player color options (1, 2, 3)
PLAYER_COLOR_1 = (160, 32, 240)   # Purple (default)
PLAYER_COLOR_2 = (32, 240, 160)   # Green
PLAYER_COLOR_3 = (240, 160, 32)   # Orange

# Gameplay constants
PLAYER_SPEED = 5
PLAYER_SHOT_COOLDOWN_MS = 500
PLAYER_HIT_INVULN_MS = 1000
PLAYER_LIVES = 3

BULLET_SPEED = -8
ENEMY_BULLET_SPEED = 6

INITIAL_ENEMY_SPEED = 10
INITIAL_WAVE_INTERVAL_MS = 2200
MIN_WAVE_INTERVAL_MS = 700
DIFFICULTY_STEP_EVERY_MS = 10000
DIFFICULTY_SPEED_INCREMENT = 2
DIFFICULTY_WAVE_INTERVAL_DECREMENT = 150

ITEM_DROP_CHANCE = 0.08
ITEM_SCORE_VALUE = 150
ENEMY_SCORE_VALUE = 100

STATE_WELCOME = 0
STATE_PLAYING = 1
STATE_GAME_OVER = 2
STATE_PAUSED = 3

# Player pixel art (P = purple, R = red, space = transparent)
PLAYER_PIXEL_ART = [
    "           PP           ",
    "           PP           ",
    "           PP           ",
    "           PP           ",
    "          PPPP          ",
    "          PPPP          ",
    "          PPPP          ",
    "          PPPP          ",
    "          PPPP          ",
    "     R   PPPPPP   R     ",
    "     R   PPPPPP   R     ",
    "     R   PPPPPP   R     ",
    "     P PPPPPPPPPP P     ",
    "     P PPPPRRPPPP P     ",
    "     PPPPPRRRRPPPPP     ",
    "RR   PPPPPRRRRPPPPP   RR",
    "RR   PPPPPRPPRPPPPP   RR",
    "RR   PPPPPPPPPPPPPP   RR",
    "PP   PPPPPPPPPPPPPP   PP",
    "PP PPPPPPPPPPPPPPPPPP PP",
    "PP PPPPRRPPPPPPRRPPPP PP",
    "PPPPP RRRPPPPPPRRR PPPPP",
    "PPPPP RRRPPPPPPRRR PPPPP",
    "PPP   RRR  PP  RRR  PPP",
    "PP    RRR  PP  RRR    PP",
    "PP         PP         PP",
]

PLAYER_PIXEL_SIZE = 3  # base size of each pixel cell
PLAYER_SCALE = 0.75    # scale relative to base size

def get_player_cell_size() -> int:
    # Use rounded size to keep proportions; ensure at least 1px
    return max(1, int(round(PLAYER_PIXEL_SIZE * PLAYER_SCALE)))

# Enemy pixel art (W = white)
ENEMY_PIXEL_ART = [
    "  W     W  ",
    "   W   W   ",
    "  WWWWWWW  ",
    " WW WWW WW ",
    "WWWWWWWWWWW",
    "W WWWWWWW W",
    "W W     W W",
    "   WW WW   ",
]
ENEMY_PIXEL_SIZE = 2

def get_enemy_cell_size() -> int:
    return max(1, int(ENEMY_PIXEL_SIZE))

# Life icon pixel art (B=border, R=red, W=white)
LIFE_PIXEL_ART = [
    "  BBBB   BBBB  ",
    " BRRRRB BRRRRB ",
    "BRRWWRRBRRRRRRB",
    "BRWWRRRRRRRRRRB",
    "BRWRRRRRRRRRB",
    "BRRRRRRRRRRRRRRB",
    " BRRRRRRRRRRRB ",
    "  BRRRRRRRRRB  ",
    "   BRRRRRRRB   ",
    "    BRRRRRB    ",
    "     BRRRB     ",
    "      BRB      ",
    "       B       ",
]
LIFE_PIXEL_SIZE = 2
LIFE_SCALE = 0.75

def get_life_cell_size():
    return max(1, int(math.floor(LIFE_PIXEL_SIZE * LIFE_SCALE)))

class RobotInfoSubscriber(object):
    def __init__(self):
        self.last_command = None
        self.sub = rospy.Subscriber("keyboard_control", String, self.__callback_function)
        rospy.loginfo("Subscriber listening")
        
    def __callback_function(self, msg):
        self.last_command = msg.data
        rospy.loginfo(f"Received: {self.last_command}")

class UserInfoSubscriber(object):
    def __init__(self):
        self.name = None
        self.username = None
        self.age = None
        self.sub = rospy.Subscriber("user_information", user_msg, self.__callback_function)
        rospy.loginfo("User info subscriber listening")
        
    def __callback_function(self, msg):
        self.name = msg.name
        self.username = msg.username
        self.age = msg.age
        rospy.loginfo(f"Received user info: name={self.name}, username={self.username}, age={self.age}")
    
    def is_complete(self):
        """Check if all user information has been received"""
        return self.name is not None and self.name != "" and \
               self.username is not None and self.username != "" and \
               self.age is not None

class ColorInfoSubscriber(object):
    def __init__(self):
        self.sub = rospy.Subscriber("color_information", user_msg, self.__callback_function)
        rospy.loginfo("Color info subscriber listening")
        
    def __callback_function(self, msg):
        """Callback to update player color based on received message"""
        try:
            # Get color value from message
            # The publisher sends color as 1, 2, or 3
            color_value = None
            
            # Try to get color from different possible fields
            if hasattr(msg, 'color'):
                color_value = msg.color
            elif hasattr(msg, 'change_player_color'):
                color_value = msg.change_player_color
            else:
                # If user_msg doesn't have color field, check if it's a custom extension
                # Try accessing it directly (in case it's dynamically added)
                try:
                    color_value = getattr(msg, 'color', None)
                except:
                    pass
                
                if color_value is None:
                    rospy.logwarn("Color field not found in message. Message type may need color field.")
                    return
            
            # Convert to int if it's a string
            if isinstance(color_value, str):
                try:
                    color_value = int(color_value.strip())
                except ValueError:
                    rospy.logwarn(f"Could not convert color value '{color_value}' to integer")
                    return
            
            # Validate and update the ROS parameter
            if color_value in [1, 2, 3]:
                rospy.set_param("change_player_color", color_value)
                color_names = {1: "Purple", 2: "Green", 3: "Orange"}
                rospy.loginfo(f"Player color updated to: {color_value} ({color_names[color_value]})")
            else:
                rospy.logwarn(f"Invalid color value received: {color_value}. Must be 1, 2, or 3")
        except Exception as e:
            rospy.logerr(f"Error updating player color: {e}")

@dataclass
class RectSprite:
    x: float
    y: float
    w: int
    h: int

    def rect(self) -> pygame.Rect:
        return pygame.Rect(int(self.x), int(self.y), self.w, self.h)


class Player(RectSprite):
    def __init__(self) -> None:
        cell = get_player_cell_size()
        sprite_w = len(PLAYER_PIXEL_ART[0]) * cell
        sprite_h = len(PLAYER_PIXEL_ART) * cell
        super().__init__(x=SCREEN_WIDTH / 2 - sprite_w / 2, y=SCREEN_HEIGHT - sprite_h - 16, w=int(sprite_w), h=int(sprite_h))
        self.last_shot_at_ms = 0
        self.lives = PLAYER_LIVES
        self.invulnerable_until_ms = 0

    def move(self, dx: float, dy: float) -> None:
        self.x = max(8, min(self.x + dx, SCREEN_WIDTH - self.w - 8))
        self.y = max(SCREEN_HEIGHT * 0.55, min(self.y + dy, SCREEN_HEIGHT - self.h - 8))

    def can_shoot(self, now_ms: int) -> bool:
        return now_ms - self.last_shot_at_ms >= PLAYER_SHOT_COOLDOWN_MS

    def on_shot(self, now_ms: int) -> None:
        self.last_shot_at_ms = now_ms

    def is_vulnerable(self, now_ms: int) -> bool:
        return now_ms >= self.invulnerable_until_ms

    def grant_invulnerability(self, now_ms: int) -> None:
        self.invulnerable_until_ms = now_ms + PLAYER_HIT_INVULN_MS


class Bullet(RectSprite):
    def __init__(self, x: float, y: float, vy: float, color: Tuple[int, int, int]) -> None:
        super().__init__(x=x, y=y, w=4, h=10)
        self.vy = vy
        self.color = color

    def update(self) -> None:
        self.y += self.vy


class Enemy(RectSprite):
    def __init__(self, x: float, y: float, vx: float, vy: float, color: Tuple[int, int, int]) -> None:
        cell = get_enemy_cell_size()
        sprite_w = len(ENEMY_PIXEL_ART[0]) * cell
        sprite_h = len(ENEMY_PIXEL_ART) * cell
        super().__init__(x=x, y=y, w=int(sprite_w), h=int(sprite_h))
        self.vx = vx
        self.vy = vy
        self.color = color

    def update(self) -> None:
        self.x += self.vx
        self.y += self.vy
        if self.x <= 8 or self.x + self.w >= SCREEN_WIDTH - 8:
            self.vx *= -1


class Item(RectSprite):
    def __init__(self, x: float, y: float) -> None:
        super().__init__(x=x, y=y, w=10, h=10)
        self.vy = 2.2

    def update(self) -> None:
        self.y += self.vy


def draw_text(surface: pygame.Surface, text: str, pos: Tuple[int, int], font: pygame.font.Font, color: Tuple[int, int, int] = COLOR_TEXT) -> None:
    img = font.render(text, False, color)
    surface.blit(img, pos)


# Draw pixel-art helper
def draw_pixel_art(surface: pygame.Surface, art: List[str], top_left: Tuple[int, int], cell: int, color_map: Dict[str, Tuple[int, int, int]]) -> None:
    ox, oy = top_left
    for r, row in enumerate(art):
        for c, ch in enumerate(row):
            if ch == ' ':
                continue
            color = color_map.get(ch)
            if color is None:
                continue
            surface.fill(color, pygame.Rect(ox + c * cell, oy + r * cell, cell, cell))

# Formation that moves a block of enemies together while members shoot/die individually
@dataclass
class Formation:
    x: float
    y: float
    width: float
    height: float
    horizontal_step_px: int
    down_step_px: int
    step_interval_ms: int
    direction: int = 1
    prev_x: float = 0.0
    prev_y: float = 0.0
    last_step_at_ms: int = 0

    def start(self, now_ms: int) -> None:
        self.prev_x = self.x
        self.prev_y = self.y
        self.last_step_at_ms = now_ms

    def move_step(self, now_ms: int) -> Tuple[float, float]:
        if now_ms - self.last_step_at_ms < self.step_interval_ms:
            return 0.0, 0.0
        self.last_step_at_ms = now_ms
        self.prev_x, self.prev_y = self.x, self.y
        next_x = self.x + self.direction * self.horizontal_step_px
        if next_x <= 8 or next_x + self.width >= SCREEN_WIDTH - 8:
            self.y += self.down_step_px
            self.direction *= -1
        else:
            self.x = next_x
        return self.x - self.prev_x, self.y - self.prev_y


# --- Wave/Pattern Spawning --------------------------------------------------
def spawn_wave(enemies: List[Enemy], difficulty_level: int, enemy_speed: float) -> None:
    # difficulty_level increases every DIFFICULTY_STEP_EVERY_MS
    # choose pattern pool based on difficulty
    patterns = [wave_line, wave_column]
    if difficulty_level >= 1:
        patterns.append(wave_v_shape)
    if difficulty_level >= 2:
        patterns.append(wave_sine_squad)
    pattern = random.choice(patterns)
    pattern(enemies, enemy_speed, difficulty_level)


def wave_line(enemies: List[Enemy], enemy_speed: float, difficulty_level: int) -> None:
    count = 5 + min(3, difficulty_level)
    spacing = (SCREEN_WIDTH - 60) // (count)
    y = -30
    for i in range(count):
        x = 30 + i * spacing
        vx = (enemy_speed if i % 2 == 0 else -enemy_speed)
        vy = enemy_speed * 0.8
        color = COLOR_ENEMY if i % 2 == 0 else COLOR_ENEMY2
        enemies.append(Enemy(x=x, y=y - i * 8, vx=vx, vy=vy, color=color))


def wave_column(enemies: List[Enemy], enemy_speed: float, difficulty_level: int) -> None:
    columns = 1 if difficulty_level == 0 else 2
    for c in range(columns):
        x = random.randint(30, SCREEN_WIDTH - 56)
        for i in range(4 + min(3, difficulty_level)):
            vx = enemy_speed * (1 if (c + i) % 2 == 0 else -1)
            vy = enemy_speed * 1.0
            enemies.append(Enemy(x=x, y=-30 - i * 24, vx=vx, vy=vy, color=COLOR_ENEMY))


def wave_v_shape(enemies: List[Enemy], enemy_speed: float, difficulty_level: int) -> None:
    center_x = SCREEN_WIDTH // 2
    rows = 3 + min(2, difficulty_level)
    for r in range(rows):
        offset = 18 + r * 16
        left_x = center_x - offset
        right_x = center_x + offset
        vy = enemy_speed * 1.1
        enemies.append(Enemy(x=left_x, y=-30 - r * 16, vx=enemy_speed, vy=vy, color=COLOR_ENEMY2))
        enemies.append(Enemy(x=right_x, y=-30 - r * 16, vx=-enemy_speed, vy=vy, color=COLOR_ENEMY))


def wave_sine_squad(enemies: List[Enemy], enemy_speed: float, difficulty_level: int) -> None:
    count = 6 + min(4, difficulty_level)
    y = -30
    for i in range(count):
        x = 20 + i * ((SCREEN_WIDTH - 40) // count)
        vx = enemy_speed * (1 if i % 2 == 0 else -1)
        vy = enemy_speed * 0.9
        e = Enemy(x=x, y=y - i * 6, vx=vx, vy=vy, color=COLOR_ENEMY if i % 3 else COLOR_ENEMY2)
        e.wave_phase = random.uniform(0.0, 6.28)
        enemies.append(e)


def enemy_fire_probability(difficulty_level: int, level: int) -> float:
    # Base chance grows with time difficulty and current level; hard-capped
    base = 0.004 + 0.0015 * difficulty_level + 0.002 * max(0, level - 1)
    return min(0.06, base)


def wave_grid_6x3(enemies: List[Enemy], enemy_speed: float, difficulty_level: int, waves_cleared: int) -> Formation:
    cols = 6
    rows = 3
    spacing = 34  # equal horizontal and vertical spacing (more separation)
    enemy_w, enemy_h = 26, 18
    formation_width = (cols - 1) * spacing + enemy_w
    formation_height = (rows - 1) * spacing + enemy_h
    start_x = (SCREEN_WIDTH - formation_width) / 2
    start_y = 60.0
    # Faster initial step speed (lower interval), still ramps with clears/difficulty
    step_ms = max(90, 360 - 40 * waves_cleared - 15 * difficulty_level)
    formation = Formation(
        x=float(start_x),
        y=float(start_y),
        width=float(formation_width),
        height=float(formation_height),
        horizontal_step_px=10,
        down_step_px=spacing,
        step_interval_ms=step_ms,
    )
    for r in range(rows):
        for c in range(cols):
            local_x = c * spacing
            local_y = r * spacing
            x = formation.x + local_x
            y = formation.y + local_y
            color = COLOR_ENEMY if (r + c) % 2 == 0 else COLOR_ENEMY2
            e = Enemy(x=float(x), y=float(y), vx=0.0, vy=0.0, color=color)
            setattr(e, "local_x", float(local_x))
            setattr(e, "local_y", float(local_y))
            enemies.append(e)
    return formation


def run_game() -> None:
    pygame.init()
    pygame.display.set_caption(WINDOW_TITLE)
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    clock = pygame.time.Clock()

    subscriber = None
    user_info_subscriber = None
    color_info_subscriber = None
    result_publisher = None
    try:
        rospy.init_node("Subscriber")
        subscriber=RobotInfoSubscriber()
        user_info_subscriber = UserInfoSubscriber()
        color_info_subscriber = ColorInfoSubscriber()
        result_publisher = rospy.Publisher("result_information", Int64, queue_size=10)
        rospy.loginfo("Result publisher initialized")
        
        # Initialize ROS parameters with default values
        rospy.set_param("user_name", "")
        rospy.set_param("change_player_color", 1)  # Default to color 1 (purple)
        rospy.set_param("screen_param", "beginning_screen")  # Initial state
        rospy.loginfo("ROS parameters initialized")
    except rospy.ROSInterruptException:
        pass

    # Try to load a pixel-like font from common names; fall back to monospace
    candidate_fonts = [
        "Press Start 2P", "PressStart2P", "VT323", "Pixellari", "Pixelify Sans",
        "Minecraft", "PixelMplus10", "Perfect DOS VGA 437", "Unifont",
        "ProggyClean", "Monaco", "Menlo", "Courier New", "Courier"
    ]
    def match(name_list):
        for name in name_list:
            path = pygame.font.match_font(name)
            if path:
                return path
        return None
    pixel_font_path = match(candidate_fonts)
    if pixel_font_path is None:
        font_small = pygame.font.SysFont("couriernew", 18, bold=False)
        font_medium = pygame.font.SysFont("couriernew", 22, bold=False)
        font_large = pygame.font.SysFont("couriernew", 32, bold=True)
    else:
        font_small = pygame.font.Font(pixel_font_path, 16)
        font_medium = pygame.font.Font(pixel_font_path, 22)
        font_large = pygame.font.Font(pixel_font_path, 32)

    state = STATE_WELCOME
    score = 0
    player = Player()
    player_bullets: List[Bullet] = []
    enemy_bullets: List[Bullet] = []
    enemies: List[Enemy] = []
    items: List[Item] = []

    enemy_speed = INITIAL_ENEMY_SPEED
    wave_interval_ms = INITIAL_WAVE_INTERVAL_MS
    next_wave_at_ms = 0
    current_formation: Formation | None = None
    last_difficulty_step_at_ms = 0
    waves_cleared = 0
    level = 1
    wave_active = False
    score_published = False
    stored_username = None
    stored_score = None
    game_difficulty = 1  # Default difficulty level (1, 2, or 3)
    previous_state = STATE_WELCOME

    # Helper function to update screen_param ROS parameter
    def update_screen_param(current_state):
        """Update screen_param ROS parameter based on current state"""
        try:
            if current_state == STATE_WELCOME:
                rospy.set_param("screen_param", "beginning_screen")
            elif current_state == STATE_PLAYING:
                rospy.set_param("screen_param", "game")
            elif current_state == STATE_GAME_OVER:
                rospy.set_param("screen_param", "game_over")
            elif current_state == STATE_PAUSED:
                rospy.set_param("screen_param", "game")  # Paused is still in game
        except Exception as e:
            rospy.logwarn(f"Failed to update screen_param: {e}")

    # Service callback function for user score
    def handle_get_user_score(req):
        """Service callback to return score for a username"""
        if stored_username is not None and req.username == stored_username:
            return GetUserScoreResponse(score=stored_score)
        else:
            return GetUserScoreResponse(score=-1)
    
    # Service callback function for difficulty
    def handle_get_game_difficulty(req):
        """Service callback to set game difficulty - only allowed in welcome screen"""
        nonlocal game_difficulty, state
        # Only allow difficulty changes when in welcome screen
        if state != STATE_WELCOME:
            rospy.logwarn(f"Cannot change difficulty: game is not in welcome screen (current state: {state})")
            return SetGameDifficultyResponse(success=False)
        
        print(req.change_difficulty)
        if req.change_difficulty in ["easy", "medium", "hard"]:
            if req.change_difficulty=="easy":
                game_difficulty=1
            elif req.change_difficulty=="medium":
                game_difficulty=2
            elif req.change_difficulty=="hard":
                game_difficulty=3
            rospy.loginfo(f"Difficulty set to {game_difficulty}")
            return SetGameDifficultyResponse(success=True)
        else:
            rospy.logwarn(f"Invalid difficulty level: {req.change_difficulty}")
            return SetGameDifficultyResponse(success=False)
    
    # Initialize service servers
    try:
        score_service = rospy.Service("score", GetUserScore, handle_get_user_score)
        rospy.loginfo("Score service server initialized")
        difficulty_service = rospy.Service("difficulty", SetGameDifficulty, handle_get_game_difficulty)
        rospy.loginfo("Difficulty service server initialized")
    except Exception as e:
        rospy.logerr(f"Failed to initialize service: {e}")

    starfield = [(random.randint(0, SCREEN_WIDTH - 1), random.randint(0, SCREEN_HEIGHT - 1), random.randint(1, 3)) for _ in range(80)]

    while True:
        now_ms = pygame.time.get_ticks()
        
        # Update screen_param if state changed
        if state != previous_state:
            update_screen_param(state)
            previous_state = state
        
        # Update user_name parameter when user info is received
        if user_info_subscriber is not None and user_info_subscriber.username is not None:
            try:
                current_user_name = rospy.get_param("user_name", "")
                if current_user_name != user_info_subscriber.username:
                    rospy.set_param("user_name", user_info_subscriber.username)
                    rospy.loginfo(f"Updated user_name parameter: {user_info_subscriber.username}")
            except Exception as e:
                rospy.logwarn(f"Failed to update user_name parameter: {e}")
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit(0)
            if event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    # In paused or welcome/game over, Esc returns to menu; otherwise quit
                    if state in (STATE_WELCOME, STATE_GAME_OVER):
                        pygame.quit()
                        sys.exit(0)
                    elif state == STATE_PAUSED:
                        state = STATE_WELCOME
                    else:
                        state = STATE_PAUSED
                if state == STATE_WELCOME:
                    # Only start game if user info is complete
                    if user_info_subscriber is not None and user_info_subscriber.is_complete():
                        state = STATE_PLAYING
                        # Store username when game starts
                        if user_info_subscriber.username is not None:
                            stored_username = user_info_subscriber.username
                            rospy.loginfo(f"Game started for username: {stored_username}")
                        # Reset game objects
                        score = 0
                        score_published = False
                        player = Player()
                        player_bullets.clear()
                        enemy_bullets.clear()
                        enemies.clear()
                        items.clear()
                        enemy_speed = INITIAL_ENEMY_SPEED
                        wave_interval_ms = INITIAL_WAVE_INTERVAL_MS
                        next_wave_at_ms = now_ms
                        current_formation = None
                        last_difficulty_step_at_ms = now_ms
                        waves_cleared = 0
                        level = 1
                        wave_active = False
                elif state == STATE_GAME_OVER:
                    if event.key in (pygame.K_RETURN, pygame.K_r):
                        state = STATE_WELCOME
                elif state == STATE_PLAYING:
                    if event.key == pygame.K_p:
                        state = STATE_PAUSED
                elif state == STATE_PAUSED:
                    # Keyboard shortcuts in pause menu
                    if event.key in (pygame.K_p, pygame.K_RETURN):
                        state = STATE_PLAYING
                    elif event.key in (pygame.K_m,):
                        state = STATE_WELCOME
                    elif event.key in (pygame.K_r,):
                        # Restart game
                        state = STATE_PLAYING
                        score = 0
                        score_published = False
                        player = Player()
                        player_bullets.clear()
                        enemy_bullets.clear()
                        enemies.clear()
                        items.clear()
                        enemy_speed = INITIAL_ENEMY_SPEED
                        wave_interval_ms = INITIAL_WAVE_INTERVAL_MS
                        next_wave_at_ms = now_ms
                        current_formation = None
                        last_difficulty_step_at_ms = now_ms
                        waves_cleared = 0
                        level = 1
                        wave_active = False
            elif event.type == pygame.MOUSEBUTTONDOWN and state == STATE_WELCOME:
                mx, my = event.pos
                # Match the rendered pixel-art Play button geometry
                PLAY_BUTTON_ART = [
                    "RRRR R     RR  R  R",
                    "R  R R    R  R R  R",
                    "BRRR R    R  R RRRR",
                    "B    B    BRRR    R",
                    "B    B    B  R    R",
                    "B    B    B  B    B",
                    "B    BBBB B  B BBBB",
                ]
                btn_cell = 8
                btn_w = len(PLAY_BUTTON_ART[0]) * btn_cell
                btn_h = len(PLAY_BUTTON_ART) * btn_cell
                btn_x = SCREEN_WIDTH // 2 - btn_w // 2
                btn_y = SCREEN_HEIGHT // 2 + 40
                if pygame.Rect(btn_x, btn_y, btn_w, btn_h).collidepoint(mx, my):
                    # Only start game if user info is complete
                    if user_info_subscriber is not None and user_info_subscriber.is_complete():
                        state = STATE_PLAYING
                        # Store username when game starts
                        if user_info_subscriber.username is not None:
                            stored_username = user_info_subscriber.username
                            rospy.loginfo(f"Game started for username: {stored_username}")
                        score = 0
                        score_published = False
                        player = Player()
                        player_bullets.clear()
                        enemy_bullets.clear()
                        enemies.clear()
                        items.clear()
                        enemy_speed = INITIAL_ENEMY_SPEED
                        wave_interval_ms = INITIAL_WAVE_INTERVAL_MS
                        next_wave_at_ms = now_ms
                        current_formation = None
                        last_difficulty_step_at_ms = now_ms
                        waves_cleared = 0
                        level = 1
                        wave_active = False
            elif event.type == pygame.MOUSEBUTTONDOWN and state == STATE_PLAYING:
                # Clickable pause button in HUD
                pause_w, pause_h = 90, 30
                pause_x = SCREEN_WIDTH - pause_w - 12
                pause_y = 12
                if pygame.Rect(pause_x, pause_y, pause_w, pause_h).collidepoint(event.pos):
                    state = STATE_PAUSED
            elif event.type == pygame.MOUSEBUTTONDOWN and state == STATE_PAUSED:
                mx, my = event.pos
                # Build pause menu button rects
                menu_w, menu_h = 220, 44
                gap = 16
                total_h = menu_h * 3 + gap * 2
                start_x = SCREEN_WIDTH // 2 - menu_w // 2
                start_y = SCREEN_HEIGHT // 2 - total_h // 2
                resume_rect = pygame.Rect(start_x, start_y, menu_w, menu_h)
                restart_rect = pygame.Rect(start_x, start_y + menu_h + gap, menu_w, menu_h)
                to_menu_rect = pygame.Rect(start_x, start_y + (menu_h + gap) * 2, menu_w, menu_h)
                if resume_rect.collidepoint(mx, my):
                    state = STATE_PLAYING
                elif restart_rect.collidepoint(mx, my):
                    state = STATE_PLAYING
                    score = 0
                    score_published = False
                    player = Player()
                    player_bullets.clear()
                    enemy_bullets.clear()
                    enemies.clear()
                    items.clear()
                    enemy_speed = INITIAL_ENEMY_SPEED
                    wave_interval_ms = INITIAL_WAVE_INTERVAL_MS
                    next_wave_at_ms = now_ms
                    current_formation = None
                    last_difficulty_step_at_ms = now_ms
                    waves_cleared = 0
                    level = 1
                    wave_active = False
                elif to_menu_rect.collidepoint(mx, my):
                    state = STATE_WELCOME
            elif event.type == pygame.MOUSEBUTTONDOWN and state == STATE_GAME_OVER:
                mx, my = event.pos
                # Build game over button rects
                button_w, button_h = 200, 50
                gap = 20
                start_x = SCREEN_WIDTH // 2 - button_w // 2
                start_y = SCREEN_HEIGHT // 2 - 10
                
                restart_rect = pygame.Rect(start_x, start_y, button_w, button_h)
                menu_rect = pygame.Rect(start_x, start_y + button_h + gap, button_w, button_h)
                
                if restart_rect.collidepoint(mx, my):
                    # Restart game
                    state = STATE_PLAYING
                    score = 0
                    score_published = False
                    player = Player()
                    player_bullets.clear()
                    enemy_bullets.clear()
                    enemies.clear()
                    items.clear()
                    enemy_speed = INITIAL_ENEMY_SPEED
                    wave_interval_ms = INITIAL_WAVE_INTERVAL_MS
                    next_wave_at_ms = now_ms
                    current_formation = None
                    last_difficulty_step_at_ms = now_ms
                    waves_cleared = 0
                    level = 1
                    wave_active = False
                elif menu_rect.collidepoint(mx, my):
                    # Return to main menu
                    state = STATE_WELCOME

        # Handle ROS subscriber commands for all states
        if state == STATE_WELCOME:
            if subscriber.last_command == "QUIT":
                pygame.quit()
                sys.exit(0)
            # Any command from ROS subscriber starts the game, but only if user info is complete
            elif subscriber.last_command is not None:
                if user_info_subscriber is not None and user_info_subscriber.is_complete():
                    state = STATE_PLAYING
                    # Store username when game starts
                    if user_info_subscriber.username is not None:
                        stored_username = user_info_subscriber.username
                        rospy.loginfo(f"Game started for username: {stored_username}")
                    # Reset game objects
                    score = 0
                    score_published = False
                    player = Player()
                    player_bullets.clear()
                    enemy_bullets.clear()
                    enemies.clear()
                    items.clear()
                    enemy_speed = INITIAL_ENEMY_SPEED
                    wave_interval_ms = INITIAL_WAVE_INTERVAL_MS
                    next_wave_at_ms = now_ms
                    current_formation = None
                    last_difficulty_step_at_ms = now_ms
                    waves_cleared = 0
                    level = 1
                    wave_active = False
                    subscriber.last_command = None
                else:
                    # Reset command if user info is not complete
                    subscriber.last_command = None
        elif state == STATE_PAUSED:
            if subscriber.last_command == "RESUME":
                state = STATE_PLAYING
                subscriber.last_command = None
            elif subscriber.last_command == "RESTART":
                # Restart game
                state = STATE_PLAYING
                score = 0
                score_published = False
                player = Player()
                player_bullets.clear()
                enemy_bullets.clear()
                enemies.clear()
                items.clear()
                enemy_speed = INITIAL_ENEMY_SPEED
                wave_interval_ms = INITIAL_WAVE_INTERVAL_MS
                next_wave_at_ms = now_ms
                current_formation = None
                last_difficulty_step_at_ms = now_ms
                waves_cleared = 0
                level = 1
                wave_active = False
                subscriber.last_command = None
            elif subscriber.last_command == "MENU":
                state = STATE_WELCOME
                subscriber.last_command = None
            elif subscriber.last_command == "QUIT":
                pygame.quit()
                sys.exit(0)
        elif state == STATE_GAME_OVER:
            if subscriber.last_command == "RESTART" or subscriber.last_command == "SHOOT":
                # Restart game
                state = STATE_PLAYING
                score = 0
                score_published = False
                player = Player()
                player_bullets.clear()
                enemy_bullets.clear()
                enemies.clear()
                items.clear()
                enemy_speed = INITIAL_ENEMY_SPEED
                wave_interval_ms = INITIAL_WAVE_INTERVAL_MS
                next_wave_at_ms = now_ms
                current_formation = None
                last_difficulty_step_at_ms = now_ms
                waves_cleared = 0
                level = 1
                wave_active = False
                subscriber.last_command = None
            elif subscriber.last_command == "MENU":
                state = STATE_WELCOME
                subscriber.last_command = None
            elif subscriber.last_command == "QUIT":
                pygame.quit()
                sys.exit(0)

        if state == STATE_PLAYING:
            dx = 0
            dy = 0

            if subscriber.last_command == "LEFT":
                dx = -1
            elif subscriber.last_command == "RIGHT":
                dx = 1
            elif subscriber.last_command == "UP":
                dy = -1
            elif subscriber.last_command == "DOWN":
                dy = 1
            elif subscriber.last_command == "SHOOT":
                if player.can_shoot(now_ms):
                    player.on_shot(now_ms)
                    player_bullets.append(Bullet(
                        x=player.x + player.w / 2 - 2,
                        y=player.y - max(6, get_player_cell_size() + 2),
                        vy=BULLET_SPEED,
                        color=COLOR_BULLET,
                    ))
            elif subscriber.last_command == "PAUSE":
                state = STATE_PAUSED
                subscriber.last_command = None
            elif subscriber.last_command == "QUIT":
                pygame.quit()
                sys.exit(0)
                

            # Reset command (so each message triggers only once)
            subscriber.last_command = None

            # Apply movement
            player.move(dx * PLAYER_SPEED * 2, dy * PLAYER_SPEED)
            
            
            difficulty_level = max(0, (now_ms - last_difficulty_step_at_ms) // DIFFICULTY_STEP_EVERY_MS)
            # Ensure level reflects cleared waves consistently
            level = waves_cleared + 1

            # spawn next 6x3 formation only when all enemies are cleared
            if now_ms >= next_wave_at_ms and len(enemies) == 0 and not wave_active:
                # Apply difficulty multiplier to enemy speed (1.0, 1.5, 2.0 for levels 1, 2, 3)
                difficulty_speed_multiplier = 1.0 + (game_difficulty - 1) * 0.5
                adjusted_enemy_speed = enemy_speed * difficulty_speed_multiplier
                current_formation = wave_grid_6x3(enemies, adjusted_enemy_speed, difficulty_level, waves_cleared)
                current_formation.start(now_ms)
                next_wave_at_ms = now_ms + wave_interval_ms
                wave_active = True

            if now_ms - last_difficulty_step_at_ms >= DIFFICULTY_STEP_EVERY_MS:
                last_difficulty_step_at_ms = now_ms
                enemy_speed += DIFFICULTY_SPEED_INCREMENT
                wave_interval_ms = max(MIN_WAVE_INTERVAL_MS, wave_interval_ms - DIFFICULTY_WAVE_INTERVAL_DECREMENT)

            for s_index, (sx, sy, sp) in enumerate(starfield):
                sy += sp
                if sy >= SCREEN_HEIGHT:
                    sx = random.randint(0, SCREEN_WIDTH - 1)
                    sy = -2
                starfield[s_index] = (sx, sy, sp)

            for b in player_bullets:
                b.update()
            for b in enemy_bullets:
                b.update()
            # Move formation as discrete steps; enemies inherit movement via local offsets
            if current_formation is not None and len(enemies) > 0:
                dx_block, dy_block = current_formation.move_step(now_ms)
                for e in enemies:
                    if hasattr(e, "local_x") and hasattr(e, "local_y"):
                        e.x += dx_block
                        e.y += dy_block
                    else:
                        e.update()
                else:
                    for e in enemies:
                        e.update()
                    # Apply difficulty multiplier to fire probability (1.0, 1.5, 2.0 for levels 1, 2, 3)
                    fire_prob = enemy_fire_probability(difficulty_level, level)
                    difficulty_fire_multiplier = 1.0 + (game_difficulty - 1) * 0.5
                    adjusted_fire_prob = min(0.06, fire_prob * difficulty_fire_multiplier)
                    if random.random() < adjusted_fire_prob:
                        enemy_bullets.append(Bullet(x=e.x + e.w / 2 - 2, y=e.y + e.h, vy=ENEMY_BULLET_SPEED, color=COLOR_ENEMY_BULLET))
            for it in items:
                it.update()

            player_bullets = [b for b in player_bullets if -20 <= b.y <= SCREEN_HEIGHT + 20]
            enemy_bullets = [b for b in enemy_bullets if -20 <= b.y <= SCREEN_HEIGHT + 20]
            enemies = [e for e in enemies if e.y <= SCREEN_HEIGHT + 40]
            items = [it for it in items if it.y <= SCREEN_HEIGHT + 20]
            if len(enemies) == 0 and current_formation is not None:
                current_formation = None
                wave_active = False
                waves_cleared += 1
                # increase speed each time a stock/wave is wiped
                enemy_speed += DIFFICULTY_SPEED_INCREMENT
                # spawn next wave sooner (shortened delay)
                respawn_delay_ms = max(300, wave_interval_ms // 3)
                next_wave_at_ms = now_ms + respawn_delay_ms

            for b in list(player_bullets):
                hit_any = False
                for e in list(enemies):
                    if b.rect().colliderect(e.rect()):
                        hit_any = True
                        enemies.remove(e)
                        # Apply difficulty reduction to item drop chance (divide by difficulty level)
                        adjusted_drop_chance = ITEM_DROP_CHANCE / game_difficulty
                        if random.random() < adjusted_drop_chance:
                            items.append(Item(x=e.x + e.w / 2 - 5, y=e.y + e.h / 2))
                        score += ENEMY_SCORE_VALUE
                        break
                if hit_any:
                    player_bullets.remove(b)

            for it in list(items):
                if player.rect().colliderect(it.rect()):
                    score += ITEM_SCORE_VALUE
                    if player.lives < MAX_LIVES:
                        player.lives += 1
                    items.remove(it)

            if player.is_vulnerable(now_ms):
                was_hit = False
                for e in list(enemies):
                    if player.rect().colliderect(e.rect()):
                        was_hit = True
                        enemies.remove(e)
                for b in list(enemy_bullets):
                    if player.rect().colliderect(b.rect()):
                        was_hit = True
                        enemy_bullets.remove(b)
                if was_hit:
                    player.lives -= 1
                    player.grant_invulnerability(now_ms)
                    if player.lives <= 0:
                        state = STATE_GAME_OVER
                        # Store username and score when game ends
                        if user_info_subscriber is not None and user_info_subscriber.username is not None:
                            stored_username = user_info_subscriber.username
                            stored_score = score
                            rospy.loginfo(f"Stored score {stored_score} for username: {stored_username}")
                        # Publish score when game ends
                        if result_publisher is not None and not score_published:
                            msg = Int64()
                            msg.data = score
                            result_publisher.publish(msg)
                            rospy.loginfo(f"Published score: {score} to result_information topic")
                            score_published = True

        screen.fill(COLOR_BACKGROUND)

        for sx, sy, sp in starfield:
            screen.fill((180, 180, 200), rect=pygame.Rect(sx, sy, 2 if sp > 1 else 1, 2 if sp > 2 else 1))

        if state == STATE_WELCOME:
            # Button geometry (used to place the title closer to it)
            btn_w, btn_h = 200, 48
            btn_x = SCREEN_WIDTH // 2 - btn_w // 2
            btn_y = SCREEN_HEIGHT // 2 - 10

            # Pixel-art game title
            GAME_TITLE_ART = [
                "  RRRR                             ",
                " RRRRRR                          ",
                "RR    RR                         ",
                "RR    RR                         ",
                "RR                              ",
                "RR        RR  R     RR   RR   RR ",
                "RR  RRRR R  R R    R  R R  R R  R",
                "RR  RRRR R  R R    R  R R    R  R",
                "RR    RR RRRR R    RRRR R RR RRRR",
                "RR    RR R  R R    R  R R  R R  R",
                " RRRRRR  R  R RRRR R  R  RR  R  R",
                "  RRRR                           ",
            ]
            title_cell = 12
            title_w = len(GAME_TITLE_ART[0]) * title_cell
            title_h = len(GAME_TITLE_ART) * title_cell
            title_x = SCREEN_WIDTH // 2 - title_w // 2
            # Place title higher and centered vertically
            title_y = SCREEN_HEIGHT // 2 - 200
            draw_pixel_art(screen, GAME_TITLE_ART, (title_x, title_y), title_cell, {'R': (255, 255, 255)})
            
            
            # Pixel-art PLAY button
            PLAY_BUTTON_ART = [
                "RRRR R     RR  R  R",
                "R  R R    R  R R  R",
                "BRRR R    R  R RRRR",
                "B    B    BRRR    R",
                "B    B    B  R    R",
                "B    B    B  B    B",
                "B    BBBB B  B BBBB",
            ]
            btn_cell = 8
            btn_w = len(PLAY_BUTTON_ART[0]) * btn_cell
            btn_h = len(PLAY_BUTTON_ART) * btn_cell
            btn_x = SCREEN_WIDTH // 2 - btn_w // 2
            btn_y = SCREEN_HEIGHT // 2 + 40
            mouse_x, mouse_y = pygame.mouse.get_pos()
            hovered = pygame.Rect(btn_x, btn_y, btn_w, btn_h).collidepoint(mouse_x, mouse_y)
            draw_pixel_art(screen, PLAY_BUTTON_ART, (btn_x, btn_y), btn_cell, {
                'R': (255, 255, 255),
                'B': (255, 255, 255)
            })
            # Center the prompt text to the button (rendered pixel-sharp)
            prompt = "Press any key or click Play"
            prompt_img = font_small.render(prompt, False, COLOR_TEXT)
            prompt_x = btn_x + btn_w // 2 - prompt_img.get_width() // 2
            prompt_y = btn_y - 56
            screen.blit(prompt_img, (prompt_x, prompt_y))
            
            # Show user info status
            if user_info_subscriber is not None and not user_info_subscriber.is_complete():
                status_text = "Waiting for user information..."
                status_img = font_small.render(status_text, False, (255, 200, 100))
                status_x = SCREEN_WIDTH // 2 - status_img.get_width() // 2
                status_y = prompt_y + font_small.get_height() + 10
                screen.blit(status_img, (status_x, status_y))
            
            draw_text(screen, "Move: Arrows / WASD  Shoot: Space", (SCREEN_WIDTH // 2 - 170, btn_y + btn_h + 20), font_small)
        elif state == STATE_PLAYING:
            player_rect = player.rect()
            # Get player color from ROS parameter
            try:
                color_param = rospy.get_param("change_player_color", 1)
                if color_param == 1:
                    player_color = PLAYER_COLOR_1
                elif color_param == 2:
                    player_color = PLAYER_COLOR_2
                elif color_param == 3:
                    player_color = PLAYER_COLOR_3
                else:
                    player_color = PLAYER_COLOR_1  # Default to color 1
            except:
                player_color = PLAYER_COLOR_1  # Default if parameter not available
            
            # draw pixel-art player
            px = int(player.x)
            py = int(player.y)
            cell = get_player_cell_size()
            for r, row in enumerate(PLAYER_PIXEL_ART):
                for c, ch in enumerate(row):
                    if ch == 'P':
                        pygame.draw.rect(screen, player_color, pygame.Rect(px + c * cell, py + r * cell, cell, cell))
                    elif ch == 'R':
                        pygame.draw.rect(screen, (220, 40, 40), pygame.Rect(px + c * cell, py + r * cell, cell, cell))

            # draw pixel-art enemies in white
            ec = get_enemy_cell_size()
            for e in enemies:
                ex, ey = int(e.x), int(e.y)
                for rr, row in enumerate(ENEMY_PIXEL_ART):
                    for cc, ch in enumerate(row):
                        if ch == 'W':
                            pygame.draw.rect(screen, (245, 245, 245), pygame.Rect(ex + cc * ec, ey + rr * ec, ec, ec))

            for b in player_bullets:
                pygame.draw.rect(screen, b.color, b.rect())
            for b in enemy_bullets:
                pygame.draw.rect(screen, b.color, b.rect())
            for it in items:
                pygame.draw.rect(screen, COLOR_ITEM, it.rect(), border_radius=2)

            # Level and HUD
            draw_text(screen, f"Level: {level}", (12, 10), font_small)
            draw_text(screen, f"Score: {score}", (12, 10 + font_small.get_height() + 4), font_small)
            draw_text(screen, f"Lives: {player.lives}", (SCREEN_WIDTH - 100, 10), font_small)

            # Draw remaining lives icons under score
            life_cell = get_life_cell_size()
            life_w = len(LIFE_PIXEL_ART[0]) * life_cell
            life_h = len(LIFE_PIXEL_ART) * life_cell
            start_x = 12
            # Place below both Level and Score lines
            start_y = 10 + font_small.get_height() * 2 + 10
            for i in range(player.lives):
                x = start_x + i * (life_w + 6)
                draw_pixel_art(screen, LIFE_PIXEL_ART, (x, start_y), life_cell, {
                    'B': (60, 60, 90),
                    'R': (220, 40, 40),
                    'W': (245, 245, 245),
                })

            # Pause button (clickable)
            pause_w, pause_h = 90, 30
            pause_x = SCREEN_WIDTH - pause_w - 12
            pause_y = 12
            pygame.draw.rect(screen, (40, 40, 60), pygame.Rect(pause_x, pause_y, pause_w, pause_h), border_radius=6)
            draw_text(screen, "PAUSE", (pause_x + 12, pause_y + 6), font_small)
 
        elif state == STATE_GAME_OVER:
            # Game Over title - centered
            game_over_img = font_large.render("GAME OVER", False, COLOR_TEXT)
            game_over_x = SCREEN_WIDTH // 2 - game_over_img.get_width() // 2
            game_over_y = SCREEN_HEIGHT // 2 - 120
            screen.blit(game_over_img, (game_over_x, game_over_y))
            
            # Final score - centered
            score_text = f"Final Score: {score}"
            score_img = font_medium.render(score_text, False, COLOR_TEXT)
            score_x = SCREEN_WIDTH // 2 - score_img.get_width() // 2
            score_y = SCREEN_HEIGHT // 2 - 60
            screen.blit(score_img, (score_x, score_y))
            
            # Buttons (Restart, Menu)
            button_w, button_h = 200, 50
            gap = 20
            start_x = SCREEN_WIDTH // 2 - button_w // 2
            start_y = SCREEN_HEIGHT // 2 - 10
            
            restart_rect = pygame.Rect(start_x, start_y, button_w, button_h)
            menu_rect = pygame.Rect(start_x, start_y + button_h + gap, button_w, button_h)
            
            # Draw buttons
            for rect, label in ((restart_rect, "Restart"), (menu_rect, "Main Menu")):
                pygame.draw.rect(screen, (40, 40, 60), rect, border_radius=8)
                # Center text in each rectangle
                label_img = font_medium.render(label, False, COLOR_TEXT)
                tx = rect.x + rect.w // 2 - label_img.get_width() // 2
                ty = rect.y + rect.h // 2 - label_img.get_height() // 2
                screen.blit(label_img, (tx, ty))
            
            # Instructions text
            instructions = "Press R to restart or M to exit the menu"
            inst_img = font_small.render(instructions, False, COLOR_TEXT)
            inst_x = SCREEN_WIDTH // 2 - inst_img.get_width() // 2
            inst_y = start_y + button_h * 2 + gap + 20
            screen.blit(inst_img, (inst_x, inst_y))
        elif state == STATE_PAUSED:
            # Dim background
            overlay = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)
            overlay.fill((0, 0, 0, 140))
            screen.blit(overlay, (0, 0))

            # Pause title higher
            draw_text(screen, "PAUSED", (SCREEN_WIDTH // 2 - 60, SCREEN_HEIGHT // 2 - 180), font_large)

            # Buttons (Resume, Restart, Menu)
            menu_w, menu_h = 220, 44
            gap = 16
            total_h = menu_h * 3 + gap * 2
            start_x = SCREEN_WIDTH // 2 - menu_w // 2
            start_y = SCREEN_HEIGHT // 2 - total_h // 2
            resume_rect = pygame.Rect(start_x, start_y, menu_w, menu_h)
            restart_rect = pygame.Rect(start_x, start_y + menu_h + gap, menu_w, menu_h)
            to_menu_rect = pygame.Rect(start_x, start_y + (menu_h + gap) * 2, menu_w, menu_h)
            for rect, label in ((resume_rect, "Resume"), (restart_rect, "Restart"), (to_menu_rect, "Menu")):
                pygame.draw.rect(screen, (40, 40, 60), rect, border_radius=8)
                # Center text in each rectangle
                label_img = font_medium.render(label, False, COLOR_TEXT)
                tx = rect.x + rect.w // 2 - label_img.get_width() // 2
                ty = rect.y + rect.h // 2 - label_img.get_height() // 2
                screen.blit(label_img, (tx, ty))
            
            # Instructions text
            instructions = "ENTER to resume,R to restart, M for main menu"
            lines = instructions.split(',')  # split by comma for separate lines

            start_y_offset = start_y + menu_h * 3 + gap * 2 + 50

            for i, line in enumerate(lines):
                inst_img = font_small.render(line.strip(), True, COLOR_TEXT)
                inst_x = SCREEN_WIDTH // 2 - inst_img.get_width() // 2
                inst_y = start_y_offset + i * (inst_img.get_height() + 5)  # 5 px gap between lines
                screen.blit(inst_img, (inst_x, inst_y))

        pygame.display.flip()
        clock.tick(FPS)


if __name__ == "__main__":
    run_game()