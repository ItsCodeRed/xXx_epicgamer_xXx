from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.messages.flat.QuickChatSelection import QuickChatSelection
from rlbot.utils.structures.game_data_struct import GameTickPacket, Vector3, Rotator, PlayerInfo
from rlbot.utils.structures.ball_prediction_struct import BallPrediction, Slice
import math

class Vec3:
    def __init__(self, *args):
        if hasattr(args[0],"__getitem__"):
            self.data = list(args[0])
        elif isinstance(args[0], Vector3):
            self.data = [args[0].x, args[0].y, args[0].z]
        elif isinstance(args[0], Rotator):
            self.data = [args[0].pitch, args[0].yaw, args[0].roll]
        elif len(args) == 3:
            self.data = list(args)
        else:
            raise TypeError("Vec3 unable to accept %s"%(args))

    @property
    def x(self): return self.data[0]
    @x.setter
    def x(self,value): self.data[0] = value
    @property
    def y(self): return self.data[1]
    @y.setter
    def y(self,value): self.data[1] = value
    @property
    def z(self): return self.data[2]
    @z.setter
    def z(self,value): self.data[2] = values

    def __getitem__(self, item: int):
        return (self.x, self.y, self.z)[item]
    def __add__(self, other: 'Vec3') -> 'Vec3':
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)
    def __sub__(self, other: 'Vec3') -> 'Vec3':
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)
    def __neg__(self):
        return Vec3(-self.x, -self.y, -self.z)
    def __mul__(self, scale: float) -> 'Vec3':
        return Vec3(self.x * scale, self.y * scale, self.z * scale)
    def __rmul__(self, scale):
        return self * scale
    def __truediv__(self, scale: float) -> 'Vec3':
        scale = 1 / float(scale)
        return self * scale

    def flat(self):
        return Vec3(self.x, self.y, 0)
    def length(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)
    def dist(self, other: 'Vec3') -> float:
        return (self - other).length()
    def dot(self, other: 'Vec3') -> float:
        return self.x*other.x + self.y*other.y + self.z*other.z
    def cross(self, other: 'Vec3') -> 'Vec3':
        return Vec3(self.y * other.z - self.z * other.y, self.z * other.x - self.x * other.z, self.x * other.y - self.y * other.x)
    def ang_to(self, ideal: 'Vec3') -> float:
        return math.acos(self.dot(ideal) / (self.length() * ideal.length()))

class Orientation:
    def __init__(self, rotation):
        cr = math.cos(float(rotation.roll))
        sr = math.sin(float(rotation.roll))
        cp = math.cos(float(rotation.pitch))
        sp = math.sin(float(rotation.pitch))
        cy = math.cos(float(rotation.yaw))
        sy = math.sin(float(rotation.yaw))

        self.forward = Vec3(cp * cy, cp * sy, sp)
        self.right = Vec3(cy*sp*sr-cr*sy, sy*sp*sr+cr*cy, -cp*sr)
        self.up = Vec3(-cr*cy*sp-sr*sy, -cr*sy*sp+sr*cy, cp*cr)

class epicgamerbotletsgooolmaoezpz(BaseAgent):
    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        self.stack = []
        self.ball_prediction = None
    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        self.ball_prediction = self.get_ball_prediction_struct()
        my_car = packet.game_cars[self.index]
        ball_location = Vec3(packet.game_ball.physics.location)
        opponent_goal_location = Vec3(0, 5300 * (-1 if self.team == 1 else 1), 320)
        my_goal_location = (Vec3(0, 5100 * (1 if self.team == 1 else -1), 320) + ball_location) / 2
        spiker = who_has_spiked(packet, ball_location)
        time_until_hit = 9999 if not spiker else Vec3(spiker.physics.location).dist(Vec3(my_car.physics.location)) / 1800 + cap((spiker.physics.location.z - 500) / 650, 0, 1)
        target_location = ball_location if not spiker else Vec3(spiker.physics.location) + Vec3(spiker.physics.velocity) * time_until_hit
        if len(self.stack) > 0 and not self.stack[-1].can_interupt:
            return self.stack[-1].run(my_car, packet, self)
        if spiker and spiker.team != self.team and target_location.z > 600:
            return goto(my_goal_location, 2000, my_car, self, packet)
        if spiker and spiker.name == my_car.name:
            if len(self.stack) > 0 and not isinstance(self.stack[-1], shoot): self.stack.pop()
            if len(self.stack) == 0: self.stack.append(shoot(opponent_goal_location, 2300))
            return self.stack[-1].run(my_car, packet, self)
        if spiker and spiker.team != self.team:
            if len(self.stack) > 0 and isinstance(self.stack[-1], intercept) and self.stack[-1].stealing:
                self.stack[-1].update(target_location, packet.game_info.seconds_elapsed + time_until_hit)
            else:
                self.stack.append(intercept(target_location, packet.game_info.seconds_elapsed + time_until_hit, True))
            return self.stack[-1].run(my_car, packet, self)
        else:
            if len(self.stack) == 0 or (len(self.stack) > 0 and isinstance(self.stack[-1], intercept) and self.stack[-1].stealing):
                shot = find_shot(self.ball_prediction, packet, my_car)
                if shot: self.stack.append(shot)
                else: return goto(ball_location, 1400, my_car, self, packet)
            return self.stack[-1].run(my_car, packet, self)

def relative_location(center: Vec3, ori: Orientation, target: Vec3) -> Vec3:
    return Vec3((target - center).dot(ori.forward), (target - center).dot(ori.right), (target - center).dot(ori.up))

def cap(value: float, minimum: float, maximum: float) -> float:
    return minimum if value < minimum else maximum if value > maximum else value

def steer_toward_target(car: PlayerInfo, target: Vec3, rate: float) -> float:
    relative = relative_location(Vec3(car.physics.location), Orientation(car.physics.rotation), target)
    return cap(((35*(math.atan2(relative.y, relative.x)+rate))**3)/10, -1.0, 1.0)

def who_has_spiked(packet, ball_location):
    closest_candidate = None
    closest_distance = 999999
    for i in range(packet.num_cars):
        car = packet.game_cars[i]
        distance = Vec3(car.physics.location).dist(ball_location)
        if 200 > distance < closest_distance:
            closest_candidate = car
            closest_distance = distance
    return closest_candidate

def find_shot(ball_prediction, packet, my_car):
    for coarse_index in range(20, ball_prediction.num_slices, 20):
        if test_shot(my_car, Vec3(ball_prediction.slices[coarse_index].physics.location), ball_prediction.slices[coarse_index].game_seconds - packet.game_info.seconds_elapsed):
            for index in range(max(20, coarse_index - 20), coarse_index):
                if test_shot(my_car, Vec3(ball_prediction.slices[index].physics.location), ball_prediction.slices[index].game_seconds - packet.game_info.seconds_elapsed):
                    return intercept(Vec3(ball_prediction.slices[index].physics.location), ball_prediction.slices[index].game_seconds)
    return None

def shot_valid(agent, intercept_time, ball_location):
    approx_index = int((intercept_time - agent.ball_prediction.slices[0].game_seconds) * 60)
    if 0 <= approx_index < agent.ball_prediction.num_slices: return Vec3(agent.ball_prediction.slices[approx_index].physics.location).dist(ball_location) < 50
    return False

def test_shot(my_car, ball_location, time_remaining):
    car_location = Vec3(my_car.physics.location)
    angle = Orientation(my_car.physics.rotation).forward.ang_to(ball_location - car_location)
    return car_location.dist(ball_location) / (1410 + 900 * (cap(my_car.boost, 0, 30) / 30)) + angle * 0.314 < time_remaining and ball_location.z < 450

def goto(target_location, target_speed, my_car, agent, packet):
    car_speed = Vec3(my_car.physics.velocity).length()
    distance = Vec3(my_car.physics.location).flat().dist(target_location.flat())
    angle = Orientation(my_car.physics.rotation).forward.ang_to(target_location - Vec3(my_car.physics.location))
    controls = SimpleControllerState()
    controls.steer = steer_toward_target(my_car, target_location, 0)
    controls.yaw = steer_toward_target(my_car, target_location, -my_car.physics.angular_velocity.z/6)
    controls.throttle = cap(target_speed - car_speed, -1, 1)
    controls.boost = (target_speed > 1410 and abs(target_speed - car_speed) > 20 and angle < 0.3)
    controls.handbrake = angle > 2.3
    controls.jump = (1 if my_car.physics.location.y >= 0 else -1) == (1 if agent.team == 1 else -1) and abs(my_car.physics.location.y) > 5000 and my_car.physics.location.z > 200
    controls.use_item = Vec3(my_car.physics.location).dist(Vec3(packet.game_ball.physics.location)) < 200 and relative_location(Vec3(my_car.physics.location), Orientation(my_car.physics.rotation), Vec3(packet.game_ball.physics.location)).z < 75
    if (abs(target_speed - car_speed) > 20 and angle < 0.3 and distance > 600) and ((target_speed > 1410 and my_car.boost == 0) or 700 < car_speed < 800): agent.stack.append(wavedash(target_location))
    return controls

class wavedash:
    def __init__(self, target):
        self.target = target
        self.can_interupt = False
        self.tick = 0
    def run(self, my_car, packet, agent):
        car_location = Vec3(my_car.physics.location)
        vertical_vel = my_car.physics.velocity.z
        controls = SimpleControllerState()
        controls.yaw = steer_toward_target(my_car, self.target, -my_car.physics.angular_velocity.z/6)
        controls.boost = not (vertical_vel < 0 and car_location.z > 40 and self.tick < 20)
        controls.use_item = car_location.dist(Vec3(packet.game_ball.physics.location)) < 200 and relative_location(car_location, Orientation(my_car.physics.rotation), Vec3(packet.game_ball.physics.location)).z < 75
        if self.tick == 0:
            controls.jump = True
            self.tick = 1
        else:
            if self.tick <= 10: self.tick += 1
            elif my_car.has_wheel_contact: agent.stack.pop()
            if vertical_vel < 0 and car_location.z > 40 and self.tick < 20:
                self.tick += 1
                controls.pitch = 1
            if vertical_vel < 0 and car_location.z < 40:
                controls.jump = True
                controls.pitch = -1
                controls.yaw = 0
        return controls

class intercept:
    def __init__(self, ball_location, intercept_time, stealing=False):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        self.stealing = stealing
        self.time_of_jump = None
        self.can_interupt = True
        self.has_quickchatted = False
    def update(self, ball_location, intercept_time):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
    def run(self, my_car, packet, agent):
        time_remaining = self.intercept_time - packet.game_info.seconds_elapsed
        perdicted_location = Vec3(my_car.physics.location).flat() + Vec3(my_car.physics.velocity).flat() * time_remaining
        controls = goto(self.ball_location, Vec3(my_car.physics.location).flat().dist(self.ball_location.flat()) / time_remaining, my_car, agent, packet)
        if not self.has_quickchatted: agent.send_quick_chat(team_only=False, quick_chat=QuickChatSelection.Information_IGotIt)
        self.has_quickchatted = True

        if not self.time_of_jump:
            if time_remaining < self.ball_location.z / 500 and perdicted_location.dist(self.ball_location.flat()) < 95 and self.ball_location.z > 140: self.time_of_jump = packet.game_info.seconds_elapsed
        else:
            if isinstance(agent.stack[-1], wavedash): agent.stack.pop()
            elapsed = packet.game_info.seconds_elapsed - self.time_of_jump
            if 0.2 < elapsed < 0.25 and self.ball_location.z > 300: controls.jump = False
            else: controls.jump = True

        if time_remaining < 0 or (not shot_valid(agent, self.intercept_time, self.ball_location) and not self.stealing) or Vec3(my_car.physics.location).flat().dist(self.ball_location.flat()) / time_remaining > 2400: agent.stack.pop()

        return controls

class shoot:
    def __init__(self, target_location, target_speed):
        self.target_location = target_location
        self.target_speed = target_speed
        self.time_of_jump = None
        self.can_interupt = True
    def run(self, my_car, packet, agent):
        distance = Vec3(my_car.physics.location).dist(Vec3(packet.game_ball.physics.location))
        angle = Orientation(my_car.physics.rotation).forward.ang_to(self.target_location - Vec3(my_car.physics.location))
        controls = goto(self.target_location, self.target_speed, my_car, agent, packet)

        if not self.time_of_jump:
            for i in range(packet.num_cars):
                car = packet.game_cars[i]
                if Vec3(car.physics.location).dist(Vec3(0, 5300 * (-1 if agent.team == 1 else 1), 320)) < Vec3(my_car.physics.location).dist(Vec3(0, 5300 * (-1 if agent.team == 1 else 1), 320)) and \
                    Vec3(car.physics.location).dist(Vec3(my_car.physics.location)) / Vec3(car.physics.velocity).length() < 1.5 and car.team != my_car.team and my_car.has_wheel_contact and angle < 0.3:
                    self.time_of_jump = packet.game_info.seconds_elapsed
                    if isinstance(agent.stack[-1], wavedash): agent.stack.pop()
        else:
            if isinstance(agent.stack[-1], wavedash): agent.stack.pop()
            if my_car.has_wheel_contact: self.time_of_jump = packet.game_info.seconds_elapsed
            elapsed = packet.game_info.seconds_elapsed - self.time_of_jump
            controls.jump = (0.2 > elapsed and my_car.has_wheel_contact) or elapsed > 0.5
            controls.pitch = 1 if 0.1 < elapsed < 0.5 else -1 if elapsed > 0.5 else 0
            controls.use_item = elapsed > 0.6
            if elapsed > 1: self.time_of_jump = None

        if distance > 200: agent.stack.pop()

        return controls