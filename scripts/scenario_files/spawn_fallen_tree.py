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
from checks import is_inside_bounding_box

class FallenTree():
    """
    Represents a scenario where a tree has fallen on the road and the ego vehicle must avoid it by making a U-turn.

    This scenario spawns a tree, a pedestrian, a cop, a cop car, and three barriers on the road.

    Attributes:
        triggered (bool): Indicates whether the scenario has been triggered.
        trigger_points (list): List of trigger points defining the trigger area.
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
            self.trigger_points = [(-95.80, -4.90),
                                   (-137.20, 2.40)]
        self.timer = 200
        self.actor_list = []
        blueprint_library = world.get_blueprint_library()
        bp_tree = blueprint_library.find('static.prop.beech_tree')
        bp_pedestrian = random.choice(blueprint_library.filter('walker.pedestrian.' + str(random.randint(3, 8)).zfill(4)))
        bp_cop = blueprint_library.find('walker.pedestrian.0030')
        bp_cop_car = blueprint_library.find('vehicle.dodge.charger_police_2020')
        bp_barrier = blueprint_library.find('static.prop.streetbarrier')

        for pedestrian in [bp_pedestrian, bp_cop]:
            if pedestrian.has_attribute('is_invincible'):
                pedestrian.set_attribute('is_invincible', 'false')

        tree_transform = carla.Transform(carla.Location(x=-108.40, y=6.50, z=0.20), carla.Rotation(pitch=0, yaw=0, roll=-90))
        pedestrian_transform = carla.Transform(carla.Location(x=-101.00, y=-3.10, z=0.93), carla.Rotation(pitch=0, yaw=270, roll=0))
        cop_transform = carla.Transform(carla.Location(x=-101.00, y=-4.60, z=0.93), carla.Rotation(pitch=0, yaw=90, roll=0))
        car_transform = carla.Transform(carla.Location(x=-101.40, y=0.90, z=0.1), carla.Rotation(pitch=0, yaw=90, roll=0))
        barrier1_transform = carla.Transform(carla.Location(x=-99.40, y=2.50, z=0.0), carla.Rotation(pitch=0, yaw=90, roll=0))
        barrier2_transform = carla.Transform(carla.Location(x=-99.40, y=1.00, z=0.0), carla.Rotation(pitch=0, yaw=90, roll=0))
        barrier3_transform = carla.Transform(carla.Location(x=-99.40, y=-0.50, z=0.0), carla.Rotation(pitch=0, yaw=90, roll=0))

        self.tree_actor = world.spawn_actor(bp_tree, tree_transform)
        self.actor_list.append(self.tree_actor)
        self.pedestrian_actor = world.spawn_actor(bp_pedestrian, pedestrian_transform)
        self.actor_list.append(self.pedestrian_actor)
        self.cop_actor = world.spawn_actor(bp_cop, cop_transform)
        self.actor_list.append(self.cop_actor)
        self.cop_car_actor = world.spawn_actor(bp_cop_car, car_transform)
        self.actor_list.append(self.cop_car_actor)
        self.barrier1_actor = world.spawn_actor(bp_barrier, barrier1_transform)
        self.actor_list.append(self.barrier1_actor)
        self.barrier2_actor = world.spawn_actor(bp_barrier, barrier2_transform)
        self.actor_list.append(self.barrier2_actor)
        self.barrier3_actor = world.spawn_actor(bp_barrier, barrier3_transform)
        self.actor_list.append(self.barrier3_actor)
        
        for pedestrian in [self.pedestrian_actor, self.cop_actor]:
            pedestrian.set_simulate_physics(True)
        
        self.cop_car_actor.set_light_state(carla.VehicleLightState(carla.VehicleLightState.Special1 | carla.VehicleLightState.LowBeam | carla.VehicleLightState.Fog | carla.VehicleLightState.Interior))

    def tick(self, ego_vehicle, traffic_manager=None):
        """
        Updates the scenario state based on the ego vehicle's position.

        If the scenario is triggered and the ego vehicle is inside the trigger area,
        the scenario will check if the vehicle has exited the trigger area in the specified time.

        Args:
            ego_vehicle: The ego vehicle object.
            traffic_manager: The traffic manager object.

        Returns:
            int: 1 if the scenario is completed (either passed or failed), 0 otherwise.
        """
        if self.triggered:
            pedestrian_actor_location = self.pedestrian_actor.get_transform().location
            cop_actor_location = self.cop_actor.get_transform().location
            if (pedestrian_actor_location.x == 0.0 and pedestrian_actor_location.y == 0.0) or (cop_actor_location.x == 0.0 and cop_actor_location.y == 0.0):
                print('A pedestrian has been killed, ego vehicle failed to complete the scenario!')
                return 1
            if self.timer > 0: 
                if is_inside_bounding_box(ego_vehicle.get_transform().location, self.trigger_points[0], self.trigger_points[1]):
                    self.timer -= 1     
                else:
                    print('The obstacle has been avoided, scenario passed!')
                    return 1
            else:
                print('The obstacle has NOT been avoided, ego vehicle failed the scenario!')
                return 1
            return 0
        elif is_inside_bounding_box(ego_vehicle.get_transform().location, self.trigger_points[0], self.trigger_points[1]):
            self.trigger()
        return 0
    
    def trigger(self):
        """
        Triggers the scenario, used by the trigger method.

        Sets the `triggered` attribute to True and prints a message indicating that the scenario has been triggered.
        Can be used to trigger the scenario at a specific point in the simulation.
        """
        self.triggered = True
        print('Scenario -Fallen Tree on the Road- has been triggered!')

    def destroy(self, client):
        """
        Destroys all actors spawned for the scenario.

        Args:
            client: The Carla client object.
        """
        print('\nScenario -Fallen Tree on the Road- finished!')
        print('Scenario is destroying %d actors' % len(self.actor_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in self.actor_list])

def main():
    path = 'E:/CARLA/images/'
    scenario_timer = 1200
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
        ego_vehicle_actor = world.spawn_actor(bp_ego_vehicle, carla.Transform(carla.Location(x=-88.40, y=-25.10, z=0.0), carla.Rotation(pitch=0, yaw=90, roll=0)))
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
        vehicle_path = ['Right', 'Right', 'Right']
        traffic_manager.set_route(ego_vehicle_actor, vehicle_path)
        traffic_manager.ignore_lights_percentage(ego_vehicle_actor, 100)
        traffic_manager.ignore_signs_percentage(ego_vehicle_actor, 100)
        traffic_manager.vehicle_percentage_speed_difference(ego_vehicle_actor, -60.0)

        # Main loop
        try:
            while True:
                # Artificial delay for images to be saved
                time.sleep(0.1)
                # Check if a scenario is triggered
                if Scenario is None:
                    Scenario = FallenTree(world, ego_vehicle_actor)
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
