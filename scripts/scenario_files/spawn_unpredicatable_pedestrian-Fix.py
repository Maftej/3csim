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
from checks import is_inside_bounding_box, get_vehicle_speed, update_trigger_points

class EmergencyPlane():
    def __init__(self, world, ego_vehicle):
        self.triggered = False
        self.trigger_points = [(-42.50, 201.00),
                               (-43.50, 209.50)]
        self.trigger_area = self.trigger_points
        self.timer = 100
        self.actor_list = []
        self.checkpoint = False
        blueprint_library = world.get_blueprint_library()
        bp_cirrus_body = blueprint_library.find('static.prop.cirrus_body')
        bp_cirrus_prop = blueprint_library.find('static.prop.cirrus_prop')

        cirrus_body_transform = carla.Transform(carla.Location(x=209.50, y=205.30, z=25.00), carla.Rotation(pitch=0, yaw=180, roll=0))
        cirrus_prop_transform = carla.Transform(carla.Location(x=4.11, y=0.00, z=0.29), carla.Rotation(pitch=0, yaw=0, roll=10))

        self.cirrus_body_actor = world.spawn_actor(bp_cirrus_body, cirrus_body_transform)
        self.actor_list.append(self.cirrus_body_actor)
        self.cirrus_prop_actor = world.spawn_actor(bp_cirrus_prop, cirrus_prop_transform, attach_to=self.cirrus_body_actor)
        self.actor_list.append(self.cirrus_prop_actor)
        self.cirrus_body_actor.set_simulate_physics(False)
        self.cirrus_prop_actor.set_simulate_physics(False)
        
    def tick(self, ego_vehicle, traffic_manager=None):
        if self.triggered:
            current_rotation = self.cirrus_prop_actor.get_transform().rotation.roll
            # Carla issue #5718 (a part of #857) from 09.22: Actor.set_location() is relative, but Actor.set_transform() is absolute for actors attached to another actor
            self.cirrus_prop_actor.set_transform(carla.Transform(carla.Location(x=4.11, y=0.00, z=0.29), carla.Rotation(pitch=0, yaw=0, roll=current_rotation + 18)))
            current_location = self.cirrus_body_actor.get_transform().location
            if self.cirrus_body_actor.get_transform().location.z > 2.80:
                self.cirrus_body_actor.set_transform(carla.Transform(carla.Location(x=current_location.x - 1.00, y=current_location.y, z=current_location.z - 0.30), carla.Rotation(pitch=0, yaw=180, roll=0)))
                return 0
            elif self.timer > 0:
                self.timer -= 1
                self.cirrus_body_actor.set_transform(carla.Transform(carla.Location(x=current_location.x - (self.timer / 100), y=current_location.y, z=current_location.z), carla.Rotation(pitch=0, yaw=180, roll=0)))
                return 0
            else:
                print('The plane has landed safely, scenario passed!')
                return 1
        elif is_inside_bounding_box(ego_vehicle.get_transform().location, self.trigger_points[0], self.trigger_points[1]):
            self.trigger()
        else:
            self.trigger_area = update_trigger_points(self.trigger_points, get_vehicle_speed(ego_vehicle), add_speed=False)
        return 0

    def trigger(self):
        self.triggered = True
        print('Scenario -Emergency Plane Landing- has been triggered!')

    def destroy(self, client):
        print('\nScenario -Emergency Plane Landing- finished!')
        print('Scenario is destroying %d actors' % len(self.actor_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in self.actor_list])

def main():
    path = 'C:/carla/PythonAPI/examples/images/'
    scenario_timer = 600
    vehicles_list = []
    walkers_list = []
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(5.0)
    seed = int(time.time())
    random.seed(seed)

    try:
        world = client.get_world()

        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        traffic_manager.set_random_device_seed(seed)
        traffic_manager.set_synchronous_mode(True)

        settings = world.get_settings()
        traffic_manager.set_synchronous_mode(True)
        if not settings.synchronous_mode:
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        # Create ego vehicle
        blueprint_library = world.get_blueprint_library()
        bp_ego_vehicle = blueprint_library.find('vehicle.dodge.charger_2020')
        bp_ego_vehicle.set_attribute('role_name', 'ego')
        # Spawn ego vehicle
        ego_vehicle_actor = world.spawn_actor(bp_ego_vehicle, carla.Transform(carla.Location(x=-88.60, y=149.85, z=0.3), carla.Rotation(pitch=0, yaw=90, roll=0)))
        vehicles_list.append(ego_vehicle_actor)
        
        # Tick world to update the simulation
        world.tick()

        # Set ego vehicle autopilot, if you have custom ego vehicle controller,
        # you should disable autopilot and use your controller to control the ego vehicle here
        ego_vehicle_actor.set_autopilot(True)
        
        # Set ego vehicle lights on, add camera sensors if selected
        traffic_manager.update_vehicle_lights(ego_vehicle_actor, True)
        # Create RGB camera for ego vehicle
        bp_camera = blueprint_library.find('sensor.camera.rgb')
        bp_camera.set_attribute("image_size_x",str(1280))
        bp_camera.set_attribute("image_size_y",str(720))
        bp_camera.set_attribute("fov",str(90))
        bp_camera.set_attribute("sensor_tick",str(0.2))
        rgb_camera_transform = carla.Transform(carla.Location(2,0,1), carla.Rotation(0,0,0))
        rgb_camera = world.spawn_actor(bp_camera, rgb_camera_transform, attach_to=ego_vehicle_actor, attachment_type=carla.AttachmentType.Rigid)
        vehicles_list.insert(len(vehicles_list)-1,rgb_camera)
        rgb_camera.listen(lambda image: image.save_to_disk(path + 'rgb/%.6d.png' % image.frame))
        # Create Semantic camera for ego vehicle
        bp_semantic_camera = blueprint_library.find('sensor.camera.semantic_segmentation')
        bp_semantic_camera.set_attribute("image_size_x",str(1280))
        bp_semantic_camera.set_attribute("image_size_y",str(720))
        bp_semantic_camera.set_attribute("fov",str(90))
        bp_semantic_camera.set_attribute("sensor_tick",str(0.2))
        semantic_camera_transform = carla.Transform(carla.Location(2,0,1), carla.Rotation(0,0,0))
        semantic_camera = world.spawn_actor(bp_semantic_camera, semantic_camera_transform, attach_to=ego_vehicle_actor, attachment_type=carla.AttachmentType.Rigid)
        vehicles_list.insert(len(vehicles_list)-1,semantic_camera)
        semantic_camera.listen(lambda image: image.save_to_disk(path + 'semantic/%.6d.png' % image.frame,carla.ColorConverter.CityScapesPalette))
        
        # Tick world to update the simulation
        Scenario = None
        world.tick()

        # Set route for ego vehicle
        vehicle_path = ['Straight', 'Straight']
        traffic_manager.set_route(ego_vehicle_actor, vehicle_path)
        traffic_manager.ignore_lights_percentage(ego_vehicle_actor, 100)

        # Main loop
        try:
            while True:
                # Artificial delay for images to be saved
                time.sleep(0.2)
                # Get the location of ego_vehicle_actor
                #ego_vehicle_location = ego_vehicle_actor.get_transform().location
                # Check if a scenario is triggered
                if Scenario is None:
                    Scenario = EmergencyPlane(world, ego_vehicle_actor)
                else:
                    if scenario_timer > 0:
                        sucess = Scenario.tick(ego_vehicle_actor, traffic_manager=traffic_manager)
                        if sucess == 1:
                            scenario_timer = 0
                        else:
                            scenario_timer -= 1
                    else:
                        Scenario.destroy(client)
                        Scenario = None
                        scenario_timer = 600
                        break
                        
                world.tick()
        except KeyboardInterrupt:
            pass

    finally:
        time.sleep(2.0)
        # Stop camera sensors to avoid errors
        rgb_camera.stop()
        semantic_camera.stop()
        world.tick()
        
        # Disable synchronous mode
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.no_rendering_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        # Destroy scenario
        if Scenario is not None:
            Scenario.destroy(client)
        # Destroy all actors
        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
        print('\ndestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in walkers_list])

        time.sleep(0.5)
        print('done.')


if __name__ == '__main__':

    main()
