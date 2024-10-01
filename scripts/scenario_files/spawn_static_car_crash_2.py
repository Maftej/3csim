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

class StaticCarCrash_2():
    """
    Represents a scenario where a car crash occured after entering an intersection.

    This scenario spawns multiple actors, including police cars, civilian cars, and pedestrians.
    The scenario is triggered when the ego vehicle enters a specified trigger area in the affected lane.
    The objective of the scenario is to test ego vehicle behavior in case of a blocked lane within an intersection in a specified time.

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
            self.trigger_points = [(-104.00, 134.40),
                                   (-90.50, 141.50)]
        self.trigger_area = self.trigger_points
        self.checkpoint = False
        self.timer = 200
        self.actor_list = []
        blueprint_library = world.get_blueprint_library()
        bp_police = blueprint_library.find('vehicle.dodge.charger_police_2020')
        bp_taxi = blueprint_library.find('vehicle.ford.crown')
        bp_van = blueprint_library.find('vehicle.mercedes.sprinter')
        bp_woman = blueprint_library.find('walker.pedestrian.00' + random.choice(['06', '07', '08', '15', '19', '37', '42', '43', '31']))
        bp_man = blueprint_library.find('walker.pedestrian.00' + random.choice(['04', '29', '16', '17', '46', '47', '28']))
        bp_police_officer = blueprint_library.find('walker.pedestrian.00' + random.choice(['30', '32']))

        for pedestrian in [bp_man, bp_woman, bp_police_officer]:
            if pedestrian.has_attribute('is_invincible'):
                pedestrian.set_attribute('is_invincible', 'false')

        police_transform = carla.Transform(carla.Location(x=-100.50, y=140.20, z=0.01), carla.Rotation(roll=0, pitch=0, yaw=20))
        taxi_transform = carla.Transform(carla.Location(x=-91.90, y=136.90, z=0.03), carla.Rotation(roll=0, pitch=0, yaw=110))
        van_transform = carla.Transform(carla.Location(x=-93.10, y=139.40, z=1.275), carla.Rotation(roll=93, pitch=1, yaw=40))
        woman_transform = carla.Transform(carla.Location(x=-95.40, y=145.10, z=1.08), carla.Rotation(roll=0, pitch=0, yaw=260))
        man_transform = carla.Transform(carla.Location(x=-95.70, y=143.90, z=1.13), carla.Rotation(roll=0, pitch=0, yaw=80))
        police_officer_transform = carla.Transform(carla.Location(x=-96.60, y=144.60, z=1.08), carla.Rotation(roll=0, pitch=0, yaw=350))

        self.police_car_actor = world.spawn_actor(bp_police, police_transform)
        self.actor_list.append(self.police_car_actor)
        self.taxi_actor = world.spawn_actor(bp_taxi, taxi_transform)
        self.actor_list.append(self.taxi_actor)
        self.van_actor = world.spawn_actor(bp_van, van_transform)
        self.van_actor.set_simulate_physics(False)
        self.actor_list.append(self.van_actor)
        self.woman_actor = world.spawn_actor(bp_woman, woman_transform)
        self.actor_list.append(self.woman_actor)
        self.man_actor = world.spawn_actor(bp_man, man_transform)
        self.actor_list.append(self.man_actor)
        self.police_officer_actor = world.spawn_actor(bp_police_officer, police_officer_transform)
        self.actor_list.append(self.police_officer_actor)
        self.police_car_actor.set_light_state(carla.VehicleLightState(carla.VehicleLightState.RightBlinker | carla.VehicleLightState.LeftBlinker | carla.VehicleLightState.Fog | carla.VehicleLightState.Interior | carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam | carla.VehicleLightState.Special1))
        self.van_actor.set_light_state(carla.VehicleLightState(carla.VehicleLightState.RightBlinker | carla.VehicleLightState.LeftBlinker | carla.VehicleLightState.Fog | carla.VehicleLightState.Interior | carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam))
        self.taxi_actor.set_light_state(carla.VehicleLightState(carla.VehicleLightState.RightBlinker | carla.VehicleLightState.LeftBlinker | carla.VehicleLightState.Fog | carla.VehicleLightState.Interior | carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam))
        for car in [self.police_car_actor, self.taxi_actor, self.van_actor]:
            car.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0, hand_brake=True, reverse=False))

    def tick(self, ego_vehicle, traffic_manager=None):
        """
        Updates the scenario state based on the ego vehicle's position.

        If the scenario is triggered and the ego vehicle is inside the trigger area,
        the scenario checks if the timer has expired. If the timer has expired, the scenario fails.
        If the ego vehicle is outside the trigger area, the scenario passes.

        Args:
            ego_vehicle: The ego vehicle object.
            traffic_manager (optional): The traffic manager object.

        Returns:
            int: 1 if the scenario is completed (either passed or failed), 0 otherwise.
        """
        if self.triggered:
            if self.checkpoint == False:
                if ego_vehicle.is_at_traffic_light():
                    traffic_light = ego_vehicle.get_traffic_light()
                    if traffic_light.get_state() == carla.TrafficLightState.Red or traffic_light.get_state() == carla.TrafficLightState.Yellow:
                        traffic_light.set_state(carla.TrafficLightState.Green)
                        traffic_light.set_green_time(200.0)
                        self.checkpoint = True
            else:
                if is_inside_bounding_box(ego_vehicle.get_transform().location, self.trigger_area[0], self.trigger_area[1]):
                    if self.timer > 0:
                        self.timer -= 1
                    else:
                        print('Ego vehicle failed to complete the scenario!')
                        return 1
                else:
                    print('Ego vehicle successfully avoided the obstacle, scenario passed!')
                    return 1
            return 0
        elif is_inside_bounding_box(ego_vehicle.get_transform().location, self.trigger_area[0], self.trigger_area[1]):
            self.trigger()
        return 0

    def trigger(self):
        """
        Triggers the scenario, used by the tick method.

        Sets the `triggered` attribute to True and prints a message indicating that the scenario has been triggered.
        Can be used to trigger the scenario at a specific point in the simulation.
        """
        self.triggered = True
        print('Scenario -Static Car Crash at intersection - has been triggered!')

    def destroy(self, client):
        """
        Destroys all actors spawned for the scenario.

        Args:
            client: The Carla client object.
        """
        print('\nScenario -Static Car Crash at intersection- finished!')
        print('Scenario is destroying %d actors' % len(self.actor_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in self.actor_list])

def main():
    path = 'E:/CARLA/images/'
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
        ego_vehicle_actor = world.spawn_actor(bp_ego_vehicle, carla.Transform(carla.Location(x=-121.90, y=136.80, z=0.3), carla.Rotation(pitch=0, roll=0, yaw=0)))
        vehicles_list.append(ego_vehicle_actor)
        
        # Tick world to update the simulation
        world.tick()

        # Set ego vehicle autopilot, if you have custom ego vehicle controller,
        # you should disable autopilot and use your controller to control the ego vehicle here
        ego_vehicle_actor.set_autopilot(True)
        
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
        vehicle_path = ['Straight', 'Right']
        traffic_manager.set_route(ego_vehicle_actor, vehicle_path)
        traffic_manager.ignore_lights_percentage(ego_vehicle_actor, 100)
        traffic_manager.ignore_signs_percentage(ego_vehicle_actor, 100)
        # If ego vehicle with traffic manager is on the outer lane, force lane change to inner lane, otherwise the car would stuck
        traffic_manager.force_lane_change(ego_vehicle_actor, False)
        traffic_manager.vehicle_percentage_speed_difference(ego_vehicle_actor, -60.0)

        # Main loop
        try:
            while True:
                # Artificial delay for images to be saved
                time.sleep(0.1)
                # Check if a scenario is triggered
                if Scenario is None:
                    Scenario = StaticCarCrash_2(world, ego_vehicle_actor)
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
