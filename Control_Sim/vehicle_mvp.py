
import pygame
from vehicle import VehicleParameters, VehicleState, handle_input, update_vehicle, draw_vehicle


# Simulation parameters
WIDTH = 1200
HEIGHT = 800
FPS = 60

PIXELS_PER_METER = 15


def main():
    pygame.init()

    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Vehicle Dynamics MVP")

    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 28)

    params = VehicleParameters()
    state = VehicleState()

    running = True

    while running:
        dt = clock.tick(FPS) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()

        throttle, brake = handle_input(keys, state, dt)

        update_vehicle(state, params, throttle, brake, dt)

        draw_vehicle(screen, state, PIXELS_PER_METER, font, WIDTH, HEIGHT)
        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
