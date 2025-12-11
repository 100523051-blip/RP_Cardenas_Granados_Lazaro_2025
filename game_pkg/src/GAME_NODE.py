#!/usr/bin/python3
import pygame
import random
import math
import sys
import os
from pathlib import Path

# ---------------- ROS Imports ----------------
import rospy
from std_msgs.msg import String

# ---------------- Constants ----------------
TILES = Path(__file__).parent / "../Assets/"
COLOR_WHITE = (255, 255, 255)
COLOR_BLACK = (0, 0, 0)
COLOR_TITLE_RED = (255, 50, 50)
COLOR_TITLE_BLUE = (30, 144, 255)
COLOR_TEXT = (220, 220, 220)
COLOR_GHOST = (255, 255, 255, 128)
COLOR_LIGHT = (255, 255, 255, 128)

WIDTH, HEIGHT = 800, 750
FPS = 60
STAR_COUNT = 50
STAR_SPEED = 1
PLAYER_SPEED = 5
PLAYER_SIZE = 32
ENEMY_SIZE = 24
PLAYER_COOLDOWN = 300
ENEMY_SHOOT_INTERVAL = 600
BULLET_SPEED = -7
ENEMY_BULLET_SPEED = 4
TILE_WIDTH = 32
TILE_HEIGHT = 32
TILES_PER_LINE = 24

# ROS input buffer
ros_input = None  # will store latest command from ROS

# ------------------------------------------------------------
# ROS CALLBACK
# ------------------------------------------------------------
def ros_callback(msg):
    """
    Callback for ROS messages.
    Stores the movement/shoot command in global variable ros_input.
    """
    global ros_input
    ros_input = msg.data


# ------------------------------------------------------------
# Pygame Utility Functions
# ------------------------------------------------------------
def scale_image(image, new_w=None, new_h=None):
    if new_w is None and new_h is None:
        return image
    old_w, old_h = image.get_width(), image.get_height()
    if new_w is not None and new_h is not None:
        return pygame.transform.scale(image, (new_w, new_h))
    elif new_w is not None:
        ratio = new_w / float(old_w)
        return pygame.transform.scale(image, (new_w, int(old_h * ratio)))
    else:
        ratio = new_h / float(old_h)
        return pygame.transform.scale(image, (int(old_w * ratio), new_h))


def load_sprite(which_x_tile):
    x0 = which_x_tile * TILE_WIDTH
    sheet_x = x0 % (TILE_WIDTH * TILES_PER_LINE)
    sheet_y = (x0 // (TILE_WIDTH * TILES_PER_LINE)) * TILE_HEIGHT
    rect = pygame.Rect(sheet_x, sheet_y, TILE_WIDTH, TILE_HEIGHT)
    sprite = SCALE_SHEET.subsurface(rect).copy()
    return sprite


# ------------------------------------------------------------
# Classes
# ------------------------------------------------------------
class Star:
    def __init__(self):
        self.x = random.randrange(0, WIDTH)
        self.y = random.randrange(0, HEIGHT)
        self.speed = STAR_SPEED
    def move(self):
        self.y += self.speed
        if self.y > HEIGHT:
            self.x = random.randrange(0, WIDTH)
            self.y = random.randrange(-20, 0)
    def draw(self, surface):
        pygame.draw.circle(surface, COLOR_WHITE, (int(self.x), int(self.y)), 2)


class Bullet:
    def __init__(self, x, y, vy, color):
        self.x = x
        self.y = y
        self.vy = vy
        self.color = color
        self.r = 5
    def move(self):
        self.y += self.vy
    def offscreen(self):
        return (self.y < -10) or (self.y > HEIGHT + 10)
    def draw(self, surface):
        pygame.draw.circle(surface, self.color, (int(self.x), int(self.y)), self.r)
    def collides(self, other):
        dx = self.x - (other.x + other.w / 2.0)
        dy = self.y - (other.y + other.h / 2.0)
        return dx * dx + dy * dy < (self.r + other.w / 2.0) ** 2


class Player:
    def __init__(self):
        self.x = WIDTH // 2 - PLAYER_SIZE // 2
        self.y = HEIGHT - 100
        self.w = PLAYER_SIZE
        self.h = PLAYER_SIZE
        self.last_shot_time = 0
    def move(self, dx, dy):
        self.x = max(0, min(WIDTH - self.w, self.x + dx))
        self.y = max(0, min(HEIGHT - self.h, self.y + dy))
    def can_shoot(self, now_ms):
        return now_ms - self.last_shot_time >= PLAYER_COOLDOWN
    def on_shot(self, now_ms):
        self.last_shot_time = now_ms
    def draw(self, surf):
        sur = scale_image(SHIP_SPRITE, PLAYER_SIZE, PLAYER_SIZE)
        surf.blit(sur, (self.x, self.y))


class Enemy:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.w = ENEMY_SIZE
        self.h = ENEMY_SIZE
        self.last_shot_time = 0
    def move(self):
        self.y += 1.5
    def try_shoot(self, now_ms):
        if now_ms - self.last_shot_time >= ENEMY_SHOOT_INTERVAL:
            self.last_shot_time = now_ms
            return Bullet(self.x + self.w/2-2, self.y+self.h+5, ENEMY_BULLET_SPEED, (255,0,0))
        return None
    def draw(self, surf):
        sur = scale_image(ENEMY_SPRITE, ENEMY_SIZE, ENEMY_SIZE)
        surf.blit(sur, (self.x, self.y))


# ------------------------------------------------------------
# Main Game Function
# ------------------------------------------------------------
def run_game():
    global ros_input

    pygame.init()
    window = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("ROS Controlled Galaga")
    clock = pygame.time.Clock()

    # Initialize ROS
    rospy.init_node("galaga_ros_controller", anonymous=True)
    rospy.Subscriber("keyboard_control", String, ros_callback)
    rospy.loginfo("Galaga game subscribed to /keyboard_control")

    player = Player()

    enemies = []
    for i in range(10):
        ex = random.randint(0, WIDTH - ENEMY_SIZE)
        ey = random.randint(-300, -50)
        enemies.append(Enemy(ex, ey))

    stars = [Star() for _ in range(STAR_COUNT)]
    player_bullets = []
    enemy_bullets = []
    last_enemy_spawn = 0
    spawn_delay = 1500
    run = True

    while run and not rospy.is_shutdown():
        dt = clock.tick(FPS)
        now_ms = pygame.time.get_ticks()

        # Pygame must process events or the window freezes
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

        # --------------------------------------------------------
        # ROS CONTROL INPUT (Movement + Shooting)
        # --------------------------------------------------------
        dx, dy = 0, 0

        if ros_input == "UP":
            dy = -1
        elif ros_input == "DOWN":
            dy = 1
        elif ros_input == "LEFT":
            dx = -1
        elif ros_input == "RIGHT":
            dx = 1
        elif ros_input == "SHOOT":
            if player.can_shoot(now_ms):
                player.on_shot(now_ms)
                player_bullets.append(
                    Bullet(player.x + player.w/2, player.y - 10, BULLET_SPEED, (0,255,0))
                )
        elif ros_input == "QUIT":
            print("Quit command received via ROS.")
            run = False

        # Reset after processing
        ros_input = None  

        player.move(dx * PLAYER_SPEED, dy * PLAYER_SPEED)

        # --------------------------------------------------------
        # Game Mechanics
        # --------------------------------------------------------
        for s in stars:
            s.move()

        for b in player_bullets[:]:
            b.move()
            if b.offscreen():
                player_bullets.remove(b)

        for b in enemy_bullets[:]:
            b.move()
            if b.offscreen():
                enemy_bullets.remove(b)

        for e in enemies[:]:
            e.move()
            if e.y > HEIGHT:
                enemies.remove(e)
            shot = e.try_shoot(now_ms)
            if shot:
                enemy_bullets.append(shot)

        # Collisions
        for e in enemies[:]:
            for b in player_bullets[:]:
                if b.collides(e):
                    enemies.remove(e)
                    player_bullets.remove(b)
                    break

        for b in enemy_bullets:
            if b.collides(player):
                print("You died!")
                run = False

        # Spawn new enemies
        if now_ms - last_enemy_spawn > spawn_delay:
            last_enemy_spawn = now_ms
            enemies.append(Enemy(random.randint(0, WIDTH - ENEMY_SIZE), random.randint(-300, -50)))

        # --------------------------------------------------------
        # DRAWING
        # --------------------------------------------------------
        window.fill(COLOR_BLACK)

        for s in stars:
            s.draw(window)
        player.draw(window)

        for e in enemies:
            e.draw(window)
        for b in player_bullets:
            b.draw(window)
        for b in enemy_bullets:
            b.draw(window)

        pygame.display.flip()

    pygame.quit()
    sys.exit()


# ------------------------------------------------------------
# Load Sprites
# ------------------------------------------------------------
#pygame.init()
#SPRITE_SHEET = pygame.image.load(str(TILES / "galaga-tiles.png")).convert_alpha()
#SCALE_SHEET = pygame.transform.scale(
#    SPRITE_SHEET,
#    (SPRITE_SHEET.get_width() * (PLAYER_SIZE // TILE_WIDTH),
#     SPRITE_SHEET.get_height() * (PLAYER_SIZE // TILE_HEIGHT))
#)

#SHIP_SPRITE = load_sprite(0)
#SHIP_SPRITE = pygame.transform.scale(SHIP_SPRITE, (PLAYER_SIZE, PLAYER_SIZE))
#ENEMY_SPRITE = load_sprite(8)
#ENEMY_SPRITE = pygame.transform.scale(ENEMY_SPRITE, (ENEMY_SIZE, ENEMY_SIZE))


# ------------------------------------------------------------
# RUN GAME
# ------------------------------------------------------------
if __name__ == "__main__":
    run_game()
