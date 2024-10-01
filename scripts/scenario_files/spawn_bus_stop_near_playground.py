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
from checks import is_inside_bounding_box, get_vehicle_speed, update_beginning_area, adjust_pedestrian_velocity

class BusStopNearPlayground():
    """
    Represents a scenario where the bus stops at bus stop and unloads kids, these kids run across the street.

    This scenario spawns three actors and is triggered when the ego vehicle enters a specified trigger area up the hill.
    The objective of the scenario is to test ego vehicle behavior in behind the bus driving.

    Attributes:
        triggered (bool): Indicates whether the scenario has been triggered.
        trigger_points (list): List of trigger points defining the trigger area.
        trigger_area (list): List of trigger points defining the trigger area.
        timer (int): The countdown timer for the scenario.
        actor_list (list): List of actors spawned for the scenario.

    Methods:
        __init__(self, world, ego_vehicle, trigger_points=None): Initializes the scenario with the specified world, ego vehicle, and trigger points.
        sensor_callback(self, event): Callback function for the collision sensor.
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
            self.trigger_points = [(150.90, -42.90),
                                   (158.20, -101.00)]
        self.trigger_area = self.trigger_points
        self.checkpoint = [False, # checkpoint for right turn
                           False, # checkpoint for left turn
                           False, # checkpoint for stopping
                           False] # checkpoint for kids crossing
        self.timer = 200
        self.actor_list = []
        self.collision_detected = False
        blueprint_library = world.get_blueprint_library()
        bp_bus = blueprint_library.find('vehicle.mitsubishi.fusorosa')
        bp_first_kid = blueprint_library.find('walker.pedestrian.00' + random.choice(['09', '10', '11', '12', '13', '14', '48', '49']))
        bp_second_kid = blueprint_library.find('walker.pedestrian.00' + random.choice(['09', '10', '11', '12', '13', '14', '48', '49']))

        for bp in [bp_first_kid, bp_second_kid]:
            if bp.has_attribute('is_invincible'):
                bp.set_attribute('is_invincible', 'false')
        
        bus_transform = carla.Transform(carla.Location(x=153.30, y=-80.40, z=8.04), carla.Rotation(roll=0, pitch=0, yaw=270))
        first_kid_transform = carla.Transform(carla.Location(x=118.70, y=-60.00, z=8.95), carla.Rotation(roll=0, pitch=0, yaw=200))
        second_kid_transform = carla.Transform(carla.Location(x=111.60, y=-60.00, z=8.95), carla.Rotation(roll=0, pitch=0, yaw=330))

        self.bus_actor = world.spawn_actor(bp_bus, bus_transform)
        self.actor_list.append(self.bus_actor)
        self.first_kid_actor = world.spawn_actor(bp_first_kid, first_kid_transform)
        self.actor_list.append(self.first_kid_actor)
        self.second_kid_actor = world.spawn_actor(bp_second_kid, second_kid_transform)
        self.actor_list.append(self.second_kid_actor)
        self.bus_actor.set_light_state(carla.VehicleLightState(carla.VehicleLightState.Interior | carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam | carla.VehicleLightState.Fog))
        self.bus_actor.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0, hand_brake=False, reverse=False))
        self.collision_sensor = world.spawn_actor(blueprint_library.find('sensor.other.collision'), carla.Transform(), attach_to=self.bus_actor)
        self.actor_list.insert(len(self.actor_list)-1, self.collision_sensor)
        self.collision_sensor.listen(lambda event: self.sensor_callback(event))

    def sensor_callback(self, event):
        """
        Callback function for the collision sensor.

        Args:
            event: The collision event.
        """
        print('Scenario vehicle collided with: ', event.other_actor.type_id)
        self.collision_detected = True

    def tick(self, ego_vehicle, traffic_manager=None):
        """
        Updates the scenario state based on the ego vehicle's position.

        If the scenario is triggered and the ego vehicle is inside the trigger area,
        the scenario moves the bus into a place for pedestrian dropoff point.
        When ego vehicle arrives, kids start running towards the playground.

        Args:
            ego_vehicle: The ego vehicle object.
            traffic_manager (optional): The traffic manager object.

        Returns:
            int: 1 if the scenario is completed (either passed or failed), 0 otherwise.
        """
        if self.triggered:
            if self.collision_detected:
                print('Collision with the car detected, scenario failed!')
                return 1
            bus_transform = self.bus_actor.get_transform()
            if self.checkpoint[0] == False:
                if bus_transform.rotation.yaw > -65.00:
                    self.bus_actor.apply_control(carla.VehicleControl(throttle=0.4, steer=-0.5, brake=0.0, hand_brake=False, reverse=False))
                    self.checkpoint[0] = True
            elif self.checkpoint[1] == False:
                if bus_transform.rotation.yaw < -84.00:
                    self.bus_actor.apply_control(carla.VehicleControl(throttle=0.3, steer=0.0, brake=0.0, hand_brake=False, reverse=False))
                    self.checkpoint[1] = True
            elif self.checkpoint[2] == False:
                if bus_transform.location.y < -97.00:
                    self.bus_actor.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.6, hand_brake=False, reverse=False))
                    self.bus_actor.set_light_state(carla.VehicleLightState(carla.VehicleLightState.RightBlinker | carla.VehicleLightState.Brake | carla.VehicleLightState.Fog | carla.VehicleLightState.Interior | carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam))
                    self.first_kid_actor.set_transform(carla.Transform(carla.Location(x=160.20, y=-103.60, z=8.85), carla.Rotation(roll=0, pitch=0, yaw=180)))
                    self.second_kid_actor.set_transform(carla.Transform(carla.Location(x=160.80, y=-104.00, z=8.85), carla.Rotation(roll=0, pitch=0, yaw=180)))
                    self.first_kid_actor.apply_control(carla.WalkerControl(direction=carla.Vector3D(x=-1.0, y=0.0, z=0.1), speed=0.1, jump=False))
                    self.second_kid_actor.apply_control(carla.WalkerControl(direction=carla.Vector3D(x=-1.0, y=0.0, z=0.1), speed=0.1, jump=False))
                    self.checkpoint[2] = True
            elif self.checkpoint[3] == False and (ego_vehicle.get_transform().location.y < -95.00+get_vehicle_speed(ego_vehicle)):
                if(is_inside_bounding_box(ego_vehicle.get_transform().location, self.trigger_area[0], self.trigger_area[1]) and (get_vehicle_speed(ego_vehicle) > 0.5)):
                    adjust_pedestrian_velocity(ego_vehicle, self.first_kid_actor)
                    adjust_pedestrian_velocity(ego_vehicle, self.second_kid_actor)
                else:
                    self.first_kid_actor.apply_control(carla.WalkerControl(direction=carla.Vector3D(x=-1.0, y=0.0, z=0.1), speed=3.0, jump=False))
                    self.second_kid_actor.apply_control(carla.WalkerControl(direction=carla.Vector3D(x=-1.0, y=0.0, z=0.1), speed=3.4, jump=False))
                    self.checkpoint[3] = True
            else:
                if self.timer > 0:
                    self.timer -= 1
                else:
                    return 1
            return 0
        elif is_inside_bounding_box(ego_vehicle.get_transform().location, self.trigger_area[0], self.trigger_area[1]):
            self.trigger()
        else:
            self.trigger_area[0] = update_beginning_area(self.trigger_points[0], get_vehicle_speed(ego_vehicle), add_speed=True, add_to_x=False)
        return 0

    def trigger(self):
        """
        Triggers the scenario, used by the tick method.

        Sets the `triggered` attribute to True, moves the bus to the right lane, applies right blinker,
        and prints a message indicating that the scenario has been triggered.
        Can be used to trigger the scenario at a specific point in the simulation.
        """
        self.bus_actor.apply_control(carla.VehicleControl(throttle=1.0, steer=0.5, brake=0.0, hand_brake=False, reverse=False))
        self.bus_actor.set_light_state(carla.VehicleLightState(carla.VehicleLightState.RightBlinker | carla.VehicleLightState.Fog | carla.VehicleLightState.Interior | carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam))
        self.triggered = True
        print('Scenario -Bus stopping near playground- has been triggered!')

    def destroy(self, client):
        """
        Stops all sensors and destroys all actors spawned for the scenario.

        Args:
            client: The Carla client object.
        """
        self.collision_sensor.stop()
        print('\nScenario -Bus stopping near playground- finished!')
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
        ego_vehicle_actor = world.spawn_actor(bp_ego_vehicle, carla.Transform(carla.Location(x=152.10, y=-17.50, z=0.6), carla.Rotation(pitch=0, roll=0, yaw=270)))
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
        vehicle_path = ['Straight', 'Left']
        traffic_manager.set_route(ego_vehicle_actor, vehicle_path)
        traffic_manager.ignore_lights_percentage(ego_vehicle_actor, 100)
        #traffic_manager.ignore_walkers_percentage(ego_vehicle_actor, 100)
        #traffic_manager.ignore_signs_percentage(ego_vehicle_actor, 100)
        #traffic_manager.ignore_vehicles_percentage(ego_vehicle_actor, 100)
        # If ego vehicle with traffic manager is on the outer lane, force lane change to inner lane
        #traffic_manager.force_lane_change(ego_vehicle_actor, False)
        #traffic_manager.auto_lane_change(ego_vehicle_actor, False)

        # Main loop
        try:
            while True:
                # Artificial delay for images to be saved
                time.sleep(0.1)
                # Check if a scenario is triggered
                if Scenario is None:
                    Scenario = BusStopNearPlayground(world, ego_vehicle_actor)
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
