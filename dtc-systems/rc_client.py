"""
    Osgar remote control via pygame
"""
import math
from threading import Thread

ROBOT_LENGTH = 0.348 * 2  # just for K2


def angle_angular_speed(angel, speed):
    return math.tan(math.radians(angel)) * speed / ROBOT_LENGTH


class RcClient:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.bus = bus
        self.bus.register("desired_speed")
        self.verbose = False
        self.max_speed = 1.0
        self.max_angular_speed = 2
        import pygame
        global pygame

        pygame.init()
        screen = pygame.display.set_mode((100, 100))


    def start(self):
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def send_speed(self, speed, angular_speed):
        angular_speed = math.copysign(min(self.max_angular_speed, abs(angular_speed)), angular_speed)
        self.bus.publish('desired_speed', [round(speed * 1000), round(math.degrees(angular_speed) * 100)])


    def run_input(self):
        speed = 0
        angle = 0  # degrees
        max_speed = self.max_speed

        while self.bus.is_alive():
            self.bus.sleep(0.1)
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_UP:
                        speed = min(speed + 0.1, max_speed)

                    elif event.key == pygame.K_DOWN:
                        speed = max(speed - 0.1, -max_speed)

                    elif event.key == pygame.K_LEFT:
                        angle = min(angle + 5, 70)

                    elif event.key == pygame.K_RIGHT:
                        angle = max(angle - 5, -70)

                    elif event.key == pygame.K_SPACE:
                        speed = 0
                        angle = 0

                    angular_speed = angle_angular_speed(angle, speed)
                    print(f"{speed:0.1f}, {angular_speed:0.3f}, {angle}")
                    self.send_speed(speed, angular_speed)

    def request_stop(self):
        self.bus.shutdown()
