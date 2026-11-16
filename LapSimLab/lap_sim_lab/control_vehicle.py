import math
import pygame


class VehicleParameters:
    def __init__(self):
        self.m = 370.0
        self.Iz = 120.0

        self.lf = 0.8
        self.lr = 0.8

        self.Cf = 40000.0
        self.Cr = 40000.0

        self.engine_force = 3000.0
        self.brake_force = 5000.0
        self.max_speed = math.inf
        self.drag_factor = 0.0
        self.rolling_force = 0.0


class VehicleState:
    def __init__(self, initial_speed=5.0):
        self.x = 0.0
        self.y = 0.0

        self.vx = initial_speed
        self.vy = 0.0

        self.yaw = 0.0
        self.yaw_rate = 0.0

        self.steer = 0.0


def handle_input(keys, state, dt):
    throttle = 0.0
    brake = 0.0

    if keys[pygame.K_UP]:
        throttle = 1.0

    if keys[pygame.K_DOWN]:
        brake = 1.0

    steer_rate = math.radians(80)

    if keys[pygame.K_LEFT]:
        state.steer += steer_rate * dt

    elif keys[pygame.K_RIGHT]:
        state.steer -= steer_rate * dt

    else:
        state.steer *= 0.95

    max_steer = math.radians(25)
    state.steer = max(-max_steer, min(max_steer, state.steer))

    return throttle, brake


def update_vehicle(state: VehicleState, params: VehicleParameters, throttle: float, brake: float, dt: float):
    vx = state.vx
    vy = state.vy
    yaw = state.yaw
    yaw_rate = state.yaw_rate
    steer = state.steer

    vx_safe = max(abs(vx), 0.5)

    alpha_f = steer - math.atan2(vy + params.lf * yaw_rate, vx_safe)
    alpha_r = -math.atan2(vy - params.lr * yaw_rate, vx_safe)

    Fyf = params.Cf * alpha_f
    Fyr = params.Cr * alpha_r

    Fx = throttle * params.engine_force - brake * params.brake_force

    dvx = Fx / params.m

    dvy = (Fyf + Fyr) / params.m - yaw_rate * vx

    dyaw_rate = (params.lf * Fyf - params.lr * Fyr) / params.Iz

    vx += dvx * dt
    vy += dvy * dt
    if params.rolling_force > 0:
        speed_loss = params.rolling_force / params.m * dt
        vx = math.copysign(max(abs(vx) - speed_loss, 0.0), vx)
    speed = math.hypot(vx, vy)
    if params.drag_factor > 0 and speed > 0:
        # Integrate quadratic drag along the velocity vector without reversing it.
        drag_scale = 1.0 / (1.0 + params.drag_factor * speed * dt / params.m)
        vx *= drag_scale
        vy *= drag_scale
        speed *= drag_scale
    if speed > params.max_speed:
        scale = params.max_speed / speed
        vx *= scale
        vy *= scale
    yaw_rate += dyaw_rate * dt

    yaw += yaw_rate * dt

    state.vx = vx
    state.vy = vy
    state.yaw_rate = yaw_rate
    state.yaw = yaw

    state.x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
    state.y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt


def draw_vehicle(
    screen, state: VehicleState, pixels_per_meter: float, font, width: int, height: int,
    *, camera=None, params=None,
):
    screen.fill((240, 240, 240))

    cx = width // 2
    cy = height // 2

    camera_x, camera_y = camera if camera is not None else (0.0, 0.0)
    px = cx + (state.x - camera_x) * pixels_per_meter
    py = cy - (state.y - camera_y) * pixels_per_meter

    if camera is not None:
        spacing = round(5 * pixels_per_meter)
        grid_color = (216, 222, 229)
        grid_x = round(cx - (camera_x % 5) * pixels_per_meter) % spacing
        grid_y = round(cy + (camera_y % 5) * pixels_per_meter) % spacing
        for x in range(grid_x, width, spacing):
            pygame.draw.line(screen, grid_color, (x, 0), (x, height))
        for y in range(grid_y, height, spacing):
            pygame.draw.line(screen, grid_color, (0, y), (width, y))

    car_length = 4.0
    car_width = 1.8

    hl = car_length * pixels_per_meter / 2
    hw = car_width * pixels_per_meter / 2

    corners = [
        (+hl, +hw),
        (+hl, -hw),
        (-hl, -hw),
        (-hl, +hw),
    ]

    rotated = []

    for dx, dy in corners:
        rx = dx * math.cos(state.yaw) - dy * math.sin(state.yaw)
        ry = dx * math.sin(state.yaw) + dy * math.cos(state.yaw)

        rotated.append((px + rx, py - ry))

    pygame.draw.polygon(screen, (40, 120, 220), rotated)

    speed = math.hypot(state.vx, state.vy)
    beta_deg = math.degrees(math.atan2(state.vy, max(abs(state.vx), 0.1)))

    info = [
        f"Speed : {speed * 3.6:.1f} km/h",
        f"Steer : {math.degrees(state.steer):.1f} deg",
        f"Yaw Rate : {math.degrees(state.yaw_rate):.1f} deg/s",
        f"Beta : {beta_deg:.2f} deg",
    ]
    if params is not None:
        info.append(f"Aero Drag : {params.drag_factor * speed * speed:.1f} N")
        rolling_force = params.rolling_force if state.vx else 0.0
        info.append(f"Rolling Resistance : {rolling_force:.1f} N")
    info.extend(("", "UP    : throttle", "DOWN  : brake", "LEFT/RIGHT : steer"))

    y_text = 20
    for line in info:
        txt = font.render(line, True, (0, 0, 0))
        screen.blit(txt, (20, y_text))
        y_text += 28

