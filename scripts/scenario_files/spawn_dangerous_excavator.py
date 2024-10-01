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

class DangerousExcavator():
    """
    Represents a scenario where an excavator moves his arm into the road once the ego vehicle enters a specified trigger area.

    This scenario spawns several actors, including a construction cone, street barriers, an excavator, and a warning sign.
    The objective of the scenario is to test the ego vehicle's behavior when encountering a dangerous object moving towards it.

    Attributes:
        triggered (bool): Indicates whether the scenario has been triggered.
        trigger_points (list): List of trigger points defining the trigger area.
        trigger_area (list): List of trigger points defining the trigger area.
        timer (int): The countdown timer for the scenario.
        actor_list (list): List of actors spawned for the scenario.

    Methods:
        __init__(self, world, ego_vehicle, trigger_points=None): Initializes the scenario with the specified world, ego vehicle, and trigger points.
        tick(self, ego_vehicle, traffic_manager=None): Updates the scenario state based on the ego vehicle's position.
        trigger(self): Triggers the scenario.
        destroy(self, client): Destroys all actors spawned for the scenario.
    """
    def __init__(self, world, ego_vehicle=None, trigger_points=None):
        """
        Initializes the scenario with the specified world, ego vehicle, and trigger points.

        Args:
            world: The Carla world object.
            ego_vehicle: The ego vehicle object. Not needed for this scenario.
            trigger_points (list, optional): List of trigger points defining the trigger area. 
                If not provided, default trigger points will be used.
        """
        self.triggered = False
        if trigger_points is not None:
            self.trigger_points = trigger_points
        else:
            self.trigger_points = [(-125.50, 131.30),
                                   (-127.50, 134.50)]
        self.trigger_area = self.trigger_points
        self.timer = 100
        self.actor_list = []
        self.checkpoint = False
        blueprint_library = world.get_blueprint_library()
        bp_construction_cone = blueprint_library.find('static.prop.constructioncone')
        bp_barrier = blueprint_library.find('static.prop.streetbarrier')
        bp_excavator_bottom = blueprint_library.find('static.prop.excavator_bottom')
        bp_excavator_top = blueprint_library.find('static.prop.excavator_top')
        bp_warning_sign = blueprint_library.find('static.prop.warningconstruction')

        construction_cone_transform = carla.Transform(carla.Location(x=-130.80, y=129.20, z=0.0), carla.Rotation(pitch=0, yaw=30, roll=0))
        excavator_bottom_transform = carla.Transform(carla.Location(x=-130.60, y=126.40, z=0.10), carla.Rotation(pitch=0, yaw=300, roll=0))
        excavator_top_transform = carla.Transform(carla.Location(x=-130.60, y=126.40, z=1.0), carla.Rotation(pitch=0, yaw=135, roll=0))
        barrier1_transform = carla.Transform(carla.Location(x=-128.20, y=129.50, z=0.0), carla.Rotation(pitch=0, yaw=300, roll=0))
        barrier2_transform = carla.Transform(carla.Location(x=-127.10, y=127.50, z=0.0), carla.Rotation(pitch=0, yaw=295, roll=0))
        barrier3_transform = carla.Transform(carla.Location(x=-126.10, y=125.50, z=0.2), carla.Rotation(pitch=0, yaw=290, roll=0))
        warning_sign_transform = carla.Transform(carla.Location(x=-122.70, y=127.50, z=0.0), carla.Rotation(pitch=0, yaw=270, roll=0))

        self.construction_cone_actor = world.spawn_actor(bp_construction_cone, construction_cone_transform)
        self.actor_list.append(self.construction_cone_actor)
        self.excavator_bottom_actor = world.spawn_actor(bp_excavator_bottom, excavator_bottom_transform)
        self.actor_list.append(self.excavator_bottom_actor)
        self.excavator_top_actor = world.spawn_actor(bp_excavator_top, excavator_top_transform)
        self.actor_list.append(self.excavator_top_actor)
        self.barrier1_actor = world.spawn_actor(bp_barrier, barrier1_transform)
        self.actor_list.append(self.barrier1_actor)
        self.barrier2_actor = world.spawn_actor(bp_barrier, barrier2_transform)
        self.actor_list.append(self.barrier2_actor)
        self.barrier3_actor = world.spawn_actor(bp_barrier, barrier3_transform)
        self.actor_list.append(self.barrier3_actor)
        self.warning_sign_actor = world.spawn_actor(bp_warning_sign, warning_sign_transform)
        self.actor_list.append(self.warning_sign_actor)
        
    def tick(self, ego_vehicle, traffic_manager=None):
        """
        Updates the scenario state based on the ego vehicle's position.

        The scenario is triggered once the ego vehicle is inside the trigger area,
        the scenario moves the arm of the excavator into his lane by turning the top of the excavator.
        The vehicle must avoid (either by turning into opposing lane or reversing) the arm of the excavator to complete the scenario.

        Args:
            ego_vehicle: The ego vehicle object used to update the scenario state and trigger points based on vehicle speed.
            traffic_manager (optional): The traffic manager object.

        Returns:
            int: 1 if the scenario is completed (either passed or failed), 0 otherwise.
        """
        if self.triggered:
            current_rotation = self.excavator_top_actor.get_transform().rotation
            if self.checkpoint == False:
                if current_rotation.yaw >= 50:
                    self.excavator_top_actor.set_transform(carla.Transform(self.excavator_top_actor.get_transform().location, carla.Rotation(pitch=current_rotation.pitch, yaw=current_rotation.yaw - 2, roll=current_rotation.roll)))
                else:
                    self.checkpoint = True
            elif self.checkpoint == True and current_rotation.yaw < 130:
                self.excavator_top_actor.set_transform(carla.Transform(self.excavator_top_actor.get_transform().location, carla.Rotation(pitch=current_rotation.pitch, yaw=current_rotation.yaw + 2.5, roll=current_rotation.roll)))
            else:                
                if self.timer > 0:
                    self.timer -= 1
                    return 0
                else:
                    return 1
            return 0
        elif is_inside_bounding_box(ego_vehicle.get_transform().location, self.trigger_area[0], self.trigger_area[1]):
            self.trigger()
        else:
            self.trigger_area = update_trigger_points(self.trigger_points, get_vehicle_speed(ego_vehicle), add_speed=True, add_to_x=True)
        return 0

    def trigger(self):
        """
        Triggers the scenario, used by the tick method.

        Sets the `triggered` attribute to True and prints a message indicating that the scenario has been triggered.
        Can be used to trigger the scenario at a specific point in the simulation.
        """
        self.triggered = True
        print('Scenario -Dangerous Excavator- has been triggered!')

    def destroy(self, client):
        """
        Destroys all actors spawned for the scenario.

        Args:
            client: The Carla client object.
        """
        print('\nScenario -Dangerous Excavator- finished!')
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
        ego_vehicle_actor = world.spawn_actor(bp_ego_vehicle, carla.Transform(carla.Location(x=-98.70, y=132.80, z=0.3), carla.Rotation(pitch=0, yaw=180, roll=0)))
        vehicles_list.append(ego_vehicle_actor)
        
        # Tick world to update the simulation
        world.tick()

        # Set ego vehicle autopilot, if you have custom ego vehicle controller,
        # you should disable autopilot and use your controller to control the ego vehicle here
        ego_vehicle_actor.set_autopilot(True)
        
        # Set ego vehicle lights on, add camera sensors if selected
        traffic_manager.update_vehicle_lights(ego_vehicle_actor, True)
        # Create collision sensor for ego vehicle
        collision_sensor = world.spawn_actor(blueprint_library.find('sensor.other.collision'), carla.Transform(), attach_to=ego_vehicle_actor)
        vehicles_list.insert(len(vehicles_list)-1,collision_sensor)
        collision_sensor.listen(lambda event: print('Ego vehicle colided with: ', event.other_actor.type_id))
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
        #vehicle_path = ['Right', 'Right','Right']
        #traffic_manager.set_route(ego_vehicle_actor, vehicle_path)
        traffic_manager.ignore_lights_percentage(ego_vehicle_actor, 100)

        # Main loop
        try:
            while True:
                # Artificial delay for images to be saved
                time.sleep(0.1)
                # Check if a scenario is triggered
                if Scenario is None:
                    Scenario = DangerousExcavator(world, ego_vehicle_actor)
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
        collision_sensor.stop()
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
