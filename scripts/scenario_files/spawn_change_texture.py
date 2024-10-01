#!/usr/bin/env python

# Copyright (c) 2024 IISLab at the Technical University of Košice.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import random
import time
from PIL import Image

class TextureChange():
    def __init__(self, world, ego_vehicle, trigger_points=None):
        self.triggered = False
        if trigger_points is not None:
            self.trigger_points = trigger_points
        else:
            self.trigger_points = [(-95.30, 10.30),
                                   (-98.80, 15.10)]
        self.trigger_area = self.trigger_points
        self.actor_list = []
        blueprint_library = world.get_blueprint_library()
        bp_pedestrian = random.choice(blueprint_library.filter('walker.pedestrian.' + str(random.randint(5, 8)).zfill(4)))

        if bp_pedestrian.has_attribute('is_invincible'):
            bp_pedestrian.set_attribute('is_invincible', 'false')
        
        pedestrian_transform = carla.Transform(carla.Location(x=-112.00, y=17.20, z=2.13), carla.Rotation(pitch=0, yaw=40, roll=0))

        self.pedestrian_actor = world.spawn_actor(bp_pedestrian, pedestrian_transform)
        self.actor_list.append(self.pedestrian_actor)
        self.pedestrian_actor.set_simulate_physics(True)

    def tick(self, ego_vehicle, traffic_manager=None):
        if self.triggered:
            ego_vehicle_location = ego_vehicle.get_transform().location
            pedestrian_actor_location = self.pedestrian_actor.get_transform().location
            if pedestrian_actor_location.x == 0.0 and pedestrian_actor_location.y == 0.0:
                print('Pedestrian has been killed, ego vehicle failed to complete the scenario!')
                return 1
            elif (ego_vehicle_location.x < -120.70 and ego_vehicle_location.y > 37.50) or (pedestrian_actor_location.x > -98.00 and pedestrian_actor_location.y > 31.00):
                print('The pedestrian has been successfully avoided, scenario passed!')
                return 1
            return 0
        elif is_inside_bounding_box(ego_vehicle.get_transform().location, self.trigger_area[0], self.trigger_area[1]):
            self.trigger()
        else:
            self.trigger_area = update_trigger_points(self.trigger_points, get_vehicle_speed(ego_vehicle), add_speed=True)
        return 0

    def trigger(self):
        self.triggered = True
        self.pedestrian_actor.apply_control(carla.WalkerControl(direction=carla.Vector3D(x=1.0, y=1.0, z=0.1), speed=2.7, jump=False))
        print('Scenario -Girl In A Park- has been triggered!')

    def destroy(self, client):
        print('\nScenario -Girl In A Park- finished!')
        print('Scenario is destroying %d actors' % len(self.actor_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in self.actor_list])

def main():
    path = 'C:/carla/PythonAPI/examples/images/'
    texture_path = 'textures/'
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(5.0)
    seed = int(time.time())
    random.seed(seed)
    textures = ['T_Cartel_Add_05_Opt_d_1.tga',
                'T_Cartel_Add_05_Opt_d_2.tga',
                'T_Cartel_Add_05_Opt_d_3.tga',
                'T_Cartel_Add_05_Opt_d_4.tga']
    world = client.get_world()
    
    #print(world.get_names_of_all_objects())
    #print(list(filter(lambda k: 'Add' in k, world.get_names_of_all_objects())))

    # Load the modified texture
    image = Image.open(texture_path + random.choice(textures))
    height = image.size[1]
    width = image.size[0]

    # Instantiate a carla.TextureColor object and populate
    # the pixels with data from the modified image
    texture = carla.TextureColor(width ,height)
    for x in range(0,width):
        for y in range(0,height):
            color = image.getpixel((x,y))
            r = int(color[0])
            g = int(color[1])
            b = int(color[2])
            a = 255
            texture.set(x, y, carla.Color(r,g,b,a))

    # Now apply the texture to the building asset
    world.apply_color_texture_to_object('BP_AddPole2_2', carla.MaterialParameter.Diffuse, texture)
    
    print('done.')


if __name__ == '__main__':

    main()
