#!/usr/bin/env python
import pygame
import random
import time
import math
import json
import os
import rospy
from std_msgs.msg import String, Bool
import os
os.environ['DISPLAY'] = ':0'
# ROS Node Initialization
rospy.init_node('cartoon_emotive_eyes')

# Initialize Pygame
pygame.init()

# Screen dimensions
screen_width = 800
screen_height = 600

# Create the screen
screen = pygame.display.set_mode((screen_width, screen_height), pygame.NOFRAME)
pygame.display.set_caption('Cartoon Emotive Eyes')

# Utility functions (no changes needed)
def lerp(a, b, t):
    return a + (b - a) * t

def load_emotion_dict_from_ros_param(param_name):
    emotion_json = rospy.get_param(param_name, 'default.json')
    rospy.loginfo(emotion_json)
    with open(emotion_json) as f:
        emotions = json.load(f)
    return emotions

def hex_to_rgb(color_value):
    predefined_colors = {
        'white': (255, 255, 255),
        'black': (0, 0, 0),
        'blue': (0, 0, 255),
        'red': (255, 0, 0),
        'green': (0, 255, 0),
        'yellow': (255, 255, 0),
        'cyan': (0, 255, 255),
        'magenta': (255, 0, 255),
        'iris_color': (0, 0, 255),
    }
    if color_value in predefined_colors:
        return predefined_colors[color_value]

    color_value = color_value.lstrip('#')
    return tuple(int(color_value[i:i+2], 16) for i in (0, 2, 4))

# Eye class and Eyes class (no changes needed)
class Eye:
    def __init__(self, center, radius, pupil_radius, emotion_dict, side):
        self.center = center
        self.default_radius = radius
        self.radius = radius
        self.pupil_radius = pupil_radius
        self.pupil_pos = center
        self.direction = [random.uniform(-1, 1), random.uniform(-1, 1)]
        self.blink_progress = 0  # progress of blink (0 to 1)
        self.emotion = 'neutral'  # current emotion
        self.movement_radius = radius - pupil_radius - 10
        self.emotion_dict = emotion_dict
        self.side = side

    def update_emotion_shape(self):
        if self.emotion in self.emotion_dict:
            self.radius = self.default_radius * self.emotion_dict[self.emotion]['radius_multiplier']

    def draw(self, screen, is_blinking, blink_progress):
        self.update_emotion_shape()
        eye_rect = pygame.Rect(self.center[0] - self.radius, self.center[1] - self.radius, self.radius * 2, self.radius * 2)
        
        if self.emotion in self.emotion_dict:
            eye_params = self.emotion_dict[self.emotion]
            # Draw eye background
            pygame.draw.ellipse(screen, hex_to_rgb(eye_params['eye_color']), eye_rect)
            # Draw iris
            pygame.draw.ellipse(screen, hex_to_rgb(eye_params['iris_color']), eye_rect.inflate(-10, -10))

            # Draw rectangles with symmetry across the Y-axis
            if 'rectangles' in eye_params:
                for rect in eye_params['rectangles']:
                    self.draw_transformed_rect(screen, self.center, self.radius, rect)

        # Draw the pupil (it should never disappear)
        pygame.draw.circle(screen, (0, 0, 0), self.pupil_pos, self.pupil_radius)

        # Draw highlights to make the eyes look cuter
        highlight_pos = [self.pupil_pos[0] - self.pupil_radius // 2, self.pupil_pos[1] - self.pupil_radius // 2]
        pygame.draw.circle(screen, (255, 255, 255), highlight_pos, self.pupil_radius // 4)
        
        if is_blinking:
            lid_height = int(self.radius * blink_progress)
            pygame.draw.rect(screen, (0, 0, 0), (self.center[0] - self.radius, self.center[1] - self.radius, self.radius * 2, lid_height))
            pygame.draw.rect(screen, (0, 0, 0), (self.center[0] - self.radius, self.center[1] + self.radius - lid_height, self.radius * 2, lid_height))

    def draw_transformed_rect(self, screen, center, radius, rect_data):
        width = rect_data.get('width', 1.0) * radius
        height = rect_data.get('height', 1.0) * radius
        angle = rect_data.get('rotation', 0)
        offset_x = rect_data.get('offset_x', 0.0) * radius
        offset_y = rect_data.get('offset_y', 0.0) * radius
        color = hex_to_rgb(rect_data.get('color', '#000000'))

        rect = pygame.Rect(0, 0, width, height)
        rect.center = (center[0] + offset_x, center[1] + offset_y)

        rotated_surface = pygame.Surface((width, height), pygame.SRCALPHA)
        rotated_surface.fill((0, 0, 0, 0))  # Transparent background
        pygame.draw.rect(rotated_surface, color, rotated_surface.get_rect())
        rotated_surface = pygame.transform.rotate(rotated_surface, angle)

        # Draw the rotated rectangle
        if self.side == "Left":
            screen.blit(rotated_surface, rotated_surface.get_rect(center=rect.center))

        # Draw the symmetrical rectangle across the Y-axis
        rect.center = (center[0] - offset_x, center[1] + offset_y)
        rotated_surface = pygame.Surface((width, height), pygame.SRCALPHA)
        rotated_surface.fill((0, 0, 0, 0))
        pygame.draw.rect(rotated_surface, color, rotated_surface.get_rect())
        rotated_surface = pygame.transform.rotate(rotated_surface, -angle)
        if self.side == "Right":
            screen.blit(rotated_surface, rotated_surface.get_rect(center=rect.center))

    def move_pupil(self, direction, t):
        angle = math.atan2(direction[1], direction[0])
        self.pupil_pos = [
            self.center[0] + math.cos(angle) * self.movement_radius * t,
            self.center[1] + math.sin(angle) * self.movement_radius * t
        ]

    def move_to_center(self, t):
        self.pupil_pos = [
            lerp(self.pupil_pos[0], self.center[0], t),
            lerp(self.pupil_pos[1], self.center[1], t)
        ]

class Eyes:
    def __init__(self, left_center, right_center, radius, pupil_radius, blink_interval, blink_duration, move_duration, emotion_dict):
        self.left_eye = Eye(left_center, radius, pupil_radius, emotion_dict, "Left")
        self.right_eye = Eye(right_center, radius, pupil_radius, emotion_dict, "Right")
        self.blink_interval = blink_interval
        self.blink_duration = blink_duration
        self.move_duration = move_duration
        self.last_blink_time = time.time()
        self.last_blink_start = 0
        self.is_blinking = False
        self.move_start_time = time.time()
        self.return_to_center_start = 0
        self.is_returning_to_center = False
        self.random_direction = [random.uniform(-1, 1), random.uniform(-1, 1)]
        self.normalize_direction()

        self.routines = []
        self.current_routine = None
        self.routine_start_time = time.time()

    def normalize_direction(self):
        magnitude = math.sqrt(self.random_direction[0] ** 2 + self.random_direction[1] ** 2)
        self.random_direction = [self.random_direction[0] / magnitude, self.random_direction[1] / magnitude]

    def update(self):
        current_time = time.time()

        # Check for blinking
        if current_time - self.last_blink_time >= self.blink_interval:
            self.is_blinking = True
            self.last_blink_start = current_time
            self.last_blink_time = current_time + random.uniform(0.5, 1.5) * self.blink_interval  # randomize next blink interval
        elif current_time - self.last_blink_start >= self.blink_duration:
            self.is_blinking = False

        # Update blink progress
        if self.is_blinking:
            blink_time = current_time - self.last_blink_start
            if blink_time < self.blink_duration / 2:
                blink_progress = blink_time / (self.blink_duration / 2)
            else:
                blink_progress = 1 - (blink_time - self.blink_duration / 2) / (self.blink_duration / 2)
        else:
            blink_progress = 0

        # Check and run routines
        if self.current_routine:
            routine_elapsed = current_time - self.routine_start_time
            if routine_elapsed >= self.current_routine['duration']:
                self.routine_start_time = current_time
                self.current_routine = None
                if self.routines:
                    self.current_routine = self.routines.pop(0)

        if self.current_routine:
            direction = self.current_routine['direction']
            t = (current_time - self.routine_start_time) / self.current_routine['duration']
        else:
            # Default movement
            if not self.is_returning_to_center and current_time - self.move_start_time >= self.move_duration:
                self.is_returning_to_center = True
                self.return_to_center_start = current_time

            # Calculate lerp factor
            if self.is_returning_to_center:
                t = (current_time - self.return_to_center_start) / self.move_duration
                self.left_eye.move_to_center(t)
                self.right_eye.move_to_center(t)
                if t >= 1:
                    self.is_returning_to_center = False
                    self.move_start_time = current_time + random.uniform(0.5, 1.5)
                    self.random_direction = [random.uniform(-1, 1), random.uniform(-1, 1)]
                    self.normalize_direction()
            else:
                t = (current_time - self.move_start_time) / self.move_duration

        return blink_progress

    def draw(self, screen, blink_progress):
        self.left_eye.draw(screen, self.is_blinking, blink_progress)
        self.right_eye.draw(screen, self.is_blinking, blink_progress)

    def set_emotion(self, emotion):
        self.left_eye.emotion = emotion
        self.right_eye.emotion = emotion
        self.left_eye.update_emotion_shape()
        self.right_eye.update_emotion_shape()

    def add_routine(self, direction, duration):
        self.routines.append({'direction': direction, 'duration': duration})
        if not self.current_routine:
            self.current_routine = self.routines.pop(0)
            self.routine_start_time = time.time()

# Load the emotion dictionary from a ROS parameter
emotion_dict = load_emotion_dict_from_ros_param('/eyes/emotion_dict')

# Initialize Eyes
distance = 150
eyes = Eyes(
    left_center=[screen_width // 2 - distance, screen_height // 2],
    right_center=[screen_width // 2 + distance, screen_height // 2],
    radius=100,
    pupil_radius=40,
    blink_interval=5,
    blink_duration=1,
    move_duration=2,
    emotion_dict=emotion_dict
)

# ROS Publishers
blink_pub = rospy.Publisher('/eyes/blink', Bool, queue_size=10)

# ROS Subscriber Callbacks
def emotion_callback(msg):
    rospy.loginfo("here")
    if msg.data in emotion_dict:
        eyes.set_emotion(msg.data)
        rospy.loginfo(f"changed to new emotion: {msg.data}")
    else:
        rospy.logwarn(f"Received unknown emotion: {msg.data}")

def blink_callback(msg):
    if msg.data:
        eyes.is_blinking = True
    else:
        eyes.is_blinking = False

# ROS Subscribers
rospy.Subscriber('/eyes/emotion', String, emotion_callback)
rospy.Subscriber('/eyes/blink', Bool, blink_callback)

# Main loop with ROS spin
rate = rospy.Rate(60)  # 60Hz

while not rospy.is_shutdown():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            rospy.signal_shutdown('Quit event received')

    # Clear the screen
    screen.fill((0, 0, 0))

    # Update and draw eyes
    blink_progress = eyes.update()
    eyes.draw(screen, blink_progress)

    # Update the display
    pygame.display.flip()

    # ROS spin and rate sleep
    rate.sleep()

pygame.quit()
