import pybullet as p
import os


class Wall:
    def __init__(self, client, base_position=(0, 0, 0)):
        self.client = client
        f_name = os.path.join(os.path.dirname(__file__), 'wall.urdf')
        self.body = p.loadURDF(fileName=f_name,
                               basePosition=[base_position[0], base_position[1], base_position[2]],
                               physicsClientId=client)

    def get_id(self):
        return self.body

    def check_and_apply_recoil(self, car_id, recoil_coeff=0.5):
        contacts = p.getContactPoints(bodyA=self.body, bodyB=car_id, physicsClientId=self.client)
        if not contacts:
            return False
        linear_vel, angular_vel = p.getBaseVelocity(car_id, physicsClientId=self.client)
        rel_speed = (linear_vel[0] ** 2 + linear_vel[1] ** 2 + linear_vel[2] ** 2) ** 0.5
        if rel_speed <= 0:
            return True
        impulse = recoil_coeff * rel_speed
        vx, vy, vz = linear_vel
        rx, ry = -vx * impulse, -vy * impulse
        p.resetBaseVelocity(car_id, linearVelocity=[vx + rx, vy + ry, vz], physicsClientId=self.client)
        return True
