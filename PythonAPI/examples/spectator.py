import pygame
import carla

class Spectator:
    
    def __init__(self, controller, world):
        self.controller = controller
        self.player = world.get_spectator()
        self.transform = self.player.get_transform()
        self.move_speed = 5
    
    def tick(self, dt):
        if self.controller.forward:
            self.player.set_location(self.player.get_location() + self.transform.get_forward_vector() * self.move_speed * dt)
        elif self.controller.back:
            self.player.set_location(self.player.get_location() + self.transform.get_forward_vector() * -self.move_speed * dt)
        rotation = self.transform.rotation 
        mouse_dt = pygame.mouse.get_rel() 
        print(mouse_dt)
        newRot = carla.Rotation(rotation.pitch ,
            rotation.yaw + mouse_dt[0] * dt, 
            rotation.roll + mouse_dt[1] * dt
        )
        self.transform.rotation = newRot
        self.player.set_transform(self.transform)
    def __str__(self):
        return "hey i am spectator"