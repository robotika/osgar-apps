"""
    Osgar remote control via pygame
"""
import math
from threading import Thread


class RcClient:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.bus = bus
        self.bus.register("desired_steering")
        self.verbose = False
        self.max_speed = 0.5
        self.max_angle = 45
        import pygame
        global pygame

        pygame.init()
        screen = pygame.display.set_mode((100, 100))


    def start(self):
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def send_speed(self, speed, angle):
        self.bus.publish('desired_speed', [round(speed * 1000), round(math.degrees(angle) * 100)])


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
                        angle = min(angle + 5, self.max_angle)

                    elif event.key == pygame.K_RIGHT:
                        angle = max(angle - 5, -self.max_angle)

                    elif event.key == pygame.K_SPACE:
                        speed = 0
                        angle = 0

                    print(f"{speed:0.1f}, {angle}")
                    self.send_speed(speed, math.radians(angle))

    def request_stop(self):
        self.bus.shutdown()
