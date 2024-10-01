#!/usr/bin/env python

# Copyright (c) 2024 IISLab at the Technical University of Košice.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Original scenario file edited to deterministically spawn a vehicle with a briefcase item on its roof

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

import random
import carla
import time
from checks import is_inside_bounding_box, get_vehicle_speed, update_trigger_points

# List of items to spawn and their transforms
def setup_transforms():
    # Select randomly from -8 to 8 inclusive
    item_random_alignment = random.randint(-8, 8)
    
    return {
        'static.prop.briefcase': carla.Transform(carla.Location(x=-1.2, y=-0.5, z=1.72), carla.Rotation(pitch=0, yaw=0, roll=-92)),
        'static.prop.guitarcase': carla.Transform(carla.Location(x=-0.6, y=(item_random_alignment/100), z=1.75), carla.Rotation(pitch=0, yaw=(item_random_alignment/100), roll=-88)),
        'static.prop.travelcase': carla.Transform(carla.Location(x=0.0, y=(item_random_alignment/100), z=1.71), carla.Rotation(pitch=0, yaw=(90+item_random_alignment), roll=-90))
    }

# A scenario where a random item drops from the roof of a car.
class CarWithItem():
    """
    Represents a scenario where a parked car starts and merges into ego vehicle's lane. After a while it drops an item and continues driving.

    This scenario spawns two actors and is triggered when the ego vehicle enters a specified trigger area.
    The objective of the scenario is to test ego vehicle's ability to predict dangerous situations.

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
            self.trigger_points = [(-2.90, -56.10),
                                   (10.00, -58.60)]
        self.trigger_area = self.trigger_points
        self.checkpoint = [False, # Full throttle to gain speed, waypoint
                           False, # Autopilot, waypoint
                           False] # Drop the item, waypoint
        self.world = world
        self.autopilot_frames = 0
        self.actor_list = []
        blueprint_library = world.get_blueprint_library()
        bp_vehicle = blueprint_library.find('vehicle.mini.cooper_s_2021')

        vehicle_transform = carla.Transform(carla.Location(x=12.00, y=-88.00, z=0.3), carla.Rotation(pitch=0, yaw=270, roll=0))
        
        blueprint_transforms = setup_transforms()
        # Randomly select a key from the dictionary of blueprints and transforms to spawn
        random_key = 'static.prop.briefcase'
        self.bp_item = blueprint_library.find(random_key)
        item_transform = blueprint_transforms[random_key]

        self.vehicle_actor = world.spawn_actor(bp_vehicle, vehicle_transform)
        self.actor_list.append(self.vehicle_actor)
        self.item = world.spawn_actor(self.bp_item, item_transform, attach_to=self.vehicle_actor)
        self.actor_list.append(self.item)
        self.vehicle_actor.set_light_state(carla.VehicleLightState(carla.VehicleLightState.LeftBlinker | carla.VehicleLightState.LowBeam | carla.VehicleLightState.Fog | carla.VehicleLightState.Interior | carla.VehicleLightState.Position))

    def tick(self, ego_vehicle, traffic_manager):
        """
        Updates the scenario state based on the ego vehicle's position.

        If the scenario is triggered and the ego vehicle is inside the trigger area,
        the scenario moves the vehicle into the lane of the ego vehicle (in front of).
        When ego vehicle straightens out, it drops the item in it's roof.

        Args:
            ego_vehicle: The ego vehicle object.
            traffic_manager: The traffic manager object.

        Returns:
            int: 1 if the scenario is completed (either passed or failed), 0 otherwise.
        """
        if self.triggered:
            if not self.checkpoint[0]:
                if (get_vehicle_speed(self.vehicle_actor) < 1.7):
                    self.vehicle_actor.apply_control(carla.VehicleControl(throttle=0.4, steer=-0.9))
                else:
                    self.vehicle_actor.apply_control(carla.VehicleControl(throttle=0.8, steer=0.0))
                    if abs(self.vehicle_actor.get_transform().location.x - ego_vehicle.get_transform().location.x) < 2:
                        self.checkpoint[0] = True
            elif not self.checkpoint[1]:
                self.vehicle_actor.set_autopilot(True)
                traffic_manager.update_vehicle_lights(self.vehicle_actor, True)
                vehicle_path = ['Straight', 'Right']
                traffic_manager.set_route(self.vehicle_actor, vehicle_path)
                self.checkpoint[1] = True
            elif not self.checkpoint[2]:
                if (get_vehicle_speed(self.vehicle_actor) > 6):
                    if (random.randint(0, 100) > 98):
                        item_transform = self.item.get_transform()
                        # Destroy the item and spawn a new one at the vehicle's location and apply physics
                        self.actor_list[1].destroy()    
                        self.item = self.world.spawn_actor(self.bp_item, item_transform)
                        self.item.set_simulate_physics(True)
                        self.actor_list.pop(1)
                        self.actor_list.append(self.item)
                        self.checkpoint[2] = True
                else:
                    if self.vehicle_actor.is_at_traffic_light():
                        traffic_light = self.vehicle_actor.get_traffic_light()
                        if traffic_light.get_state() == carla.TrafficLightState.Red:
                            traffic_light.set_state(carla.TrafficLightState.Green)
            else:
                self.autopilot_frames += 1
                if self.autopilot_frames >= 200:
                    self.vehicle_actor.set_autopilot(False)
                    return 1
            return 0
        elif is_inside_bounding_box(ego_vehicle.get_transform().location, self.trigger_area[0], self.trigger_area[1]):
            self.trigger()
        else:
            self.trigger_area = update_trigger_points(self.trigger_points, get_vehicle_speed(ego_vehicle), add_speed=True, add_to_x=False)
        return 0
    
    def trigger(self):
        """
        Triggers the scenario, used by the tick method.

        Sets the `triggered` attribute to True and prints a message indicating that the scenario has been triggered.
        Can be used to trigger the scenario at a specific point in the simulation.
        """
        self.triggered = True
        print('Scenario -Car Dropping an Item Briefcase- has been triggered!')

    def destroy(self, client):
        """
        Destroys all actors spawned for the scenario.

        Args:
            client: The Carla client object.
        """
        print('\nScenario -Car Dropping an Item Briefcase- finished!')
        print('Scenario is destroying %d actors' % len(self.actor_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in self.actor_list])


def main():
    path = 'E:/CARLA/images/'
    scenario_timer = 1000
    vehicles_list = []
    walkers_list = []
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(5.0)
    seed = 321321
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
        ego_vehicle_actor = world.spawn_actor(bp_ego_vehicle, carla.Transform(carla.Location(x=4.20, y=-30.30, z=0.3), carla.Rotation(pitch=0, yaw=270, roll=0)))
        vehicles_list.append(ego_vehicle_actor)

        # Tick world to update the simulation
        world.tick()

        # Set ego vehicle autopilot, if you have custom ego vehicle controller,
        # you should disable autopilot and use your controller to control the ego vehicle here
        ego_vehicle_actor.set_autopilot(True)
        vehicle_path = ['Straight', 'Right']
        traffic_manager.set_route(ego_vehicle_actor, vehicle_path)
        
        # Set ego vehicle lights on, add camera sensors if selected
        traffic_manager.update_vehicle_lights(ego_vehicle_actor, True)
        # Add collision sensor to ego vehicle
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
        vehicle_path = ['Straight', 'Left']
        traffic_manager.set_route(ego_vehicle_actor, vehicle_path)
        traffic_manager.ignore_lights_percentage(ego_vehicle_actor, 100)
        traffic_manager.ignore_walkers_percentage(ego_vehicle_actor, 100)
        #traffic_manager.ignore_signs_percentage(ego_vehicle_actor, 100)
        # If ego vehicle with traffic manager is on the outer lane, force lane change to inner lane, otherwise the car would stuck
        #traffic_manager.force_lane_change(ego_vehicle_actor, False)
        traffic_manager.auto_lane_change(ego_vehicle_actor, False)
        #traffic_manager.ignore_vehicles_percentage(ego_vehicle_actor, 100)

        # Main loop
        try:
            while True:
                # Artificial delay for images to be saved
                time.sleep(0.1)
                # Check if a scenario is triggered
                if Scenario is None:
                    Scenario = CarWithItem(world, ego_vehicle_actor)
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
