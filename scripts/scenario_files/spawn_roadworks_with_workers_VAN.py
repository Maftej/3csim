#!/usr/bin/env python

# Copyright (c) 2024 IISLab at the Technical University of Košice.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Original scenario file edited for deterministic use of assets (scenario using a van vehicle to block the vision)

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
from checks import is_inside_bounding_box, get_vehicle_speed, adjust_pedestrian_velocity

class RoadworksWithWorkers():
    """
    Represents a scenario where either a van is parked and blocking the view of the ego vehicle on a pedestrian crossing
    or a pedestrian is crossing the street from the worksite to the sidewalk.

    This scenario spawns nine actors and is triggered when the ego vehicle enters a specified trigger area after the roundabout.
    The objective of the scenario is to test ego vehicle behavior in the presence of predicatable workers crossing the street.

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
        self.spawn_with_van = True
        if trigger_points is not None:
            self.trigger_points = trigger_points
        else:
            if self.spawn_with_van:
                self.trigger_points = [(-11.80, 45.20),
                                       (-4.70, 70.30)]
            else:
                self.trigger_points = [(-11.80, 55.20),
                                       (-4.70, 70.30)]
        self.trigger_area = self.trigger_points
        self.checkpoint = [False, # checkpoint for left turn
                           False] # checkpoint for running
        self.timer = 400
        self.actor_list = []
        blueprint_library = world.get_blueprint_library()

        bp_pedestrian = blueprint_library.find('walker.pedestrian.0004')            
        bp_barrel = blueprint_library.find('static.prop.barrel')
        bp_ironplank = blueprint_library.find('static.prop.ironplank')
        bp_constructioncone = blueprint_library.find('static.prop.constructioncone')
        bp_plantpot = blueprint_library.find('static.prop.plantpot06')
        bp_box = blueprint_library.find('static.prop.box03')

        if bp_pedestrian.has_attribute('is_invincible'):
            bp_pedestrian.set_attribute('is_invincible', 'false')

        if self.spawn_with_van:
            bp_van = blueprint_library.find('vehicle.mercedes.sprinter')
            van_transform = carla.Transform(carla.Location(x=-13.20, y=70.20, z=0.1), carla.Rotation(roll=0, pitch=0, yaw=90))
            pedestrian_acting_transform = carla.Transform(carla.Location(x=-15.00, y=69.60, z=1.13), carla.Rotation(roll=0, pitch=0, yaw=90))
        else:
            pedestrian_acting_transform = carla.Transform(carla.Location(x=-6.10, y=74.53, z=0.1), carla.Rotation(roll=0, pitch=0, yaw=0))

        box_transform = carla.Transform(carla.Location(x=-4.90, y=72.70, z=0.0), carla.Rotation(roll=0, pitch=0, yaw=random.randint(0, 360)))
        barrel_transform = carla.Transform(carla.Location(x=-5.20, y=75.10, z=0.0), carla.Rotation(roll=0, pitch=0, yaw=-110))
        plank_transform = carla.Transform(carla.Location(x=-6.50, y=74.00, z=0.0), carla.Rotation(roll=0, pitch=0, yaw=0))
        plantpot_transform = carla.Transform(carla.Location(x=-6.20, y=73.00, z=1.0), carla.Rotation(roll=0, pitch=180, yaw=0))
        pedestrian_static_transform = carla.Transform(carla.Location(x=-5.50, y=73.80, z=0.93), carla.Rotation(roll=0, pitch=0, yaw=-140))
        construction_cone_locations = [carla.Location(x=-5.40, y=71.80, z=0.0),
                                       carla.Location(x=-7.50, y=72.10, z=0.0),
                                       carla.Location(x=-7.50, y=75.30, z=0.0),
                                       carla.Location(x=-5.40, y=75.90, z=0.0)]

        # Spawn the actors
        if self.spawn_with_van:
            self.van_actor = world.spawn_actor(bp_van, van_transform)
            self.actor_list.append(self.van_actor)
            self.van_actor.set_light_state(carla.VehicleLightState(carla.VehicleLightState.RightBlinker | carla.VehicleLightState.LeftBlinker | carla.VehicleLightState.Interior | carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam | carla.VehicleLightState.Fog))
        
        self.box_actor = world.spawn_actor(bp_box, box_transform)
        self.actor_list.append(self.box_actor)
        self.barrel_actor = world.spawn_actor(bp_barrel, barrel_transform)
        self.actor_list.append(self.barrel_actor)
        self.ironplank_actor = world.spawn_actor(bp_ironplank, plank_transform)
        self.actor_list.append(self.ironplank_actor)
        self.plantpot_actor = world.spawn_actor(bp_plantpot, plantpot_transform)
        self.actor_list.append(self.plantpot_actor)
        self.pedestrian_static_actor = world.spawn_actor(bp_pedestrian, pedestrian_static_transform)
        self.actor_list.append(self.pedestrian_static_actor)
        self.pedestrian_acting_actor = world.spawn_actor(bp_pedestrian, pedestrian_acting_transform)
        self.actor_list.append(self.pedestrian_acting_actor)
        for construction_cone_location in construction_cone_locations:
            construction_cone = world.spawn_actor(bp_constructioncone, carla.Transform(construction_cone_location, carla.Rotation(roll=0, pitch=0, yaw=random.randint(0, 360))))
            self.actor_list.append(construction_cone)
            construction_cone.set_simulate_physics(True)

    def tick(self, ego_vehicle, traffic_manager=None):
        """
        Updates the scenario state based on the ego vehicle's position.

        If the scenario is triggered and the ego vehicle is inside the trigger area,
        the scenario moves the worker pedestrian into the street and adjusts its velocity
        based on the ego vehicle's speed.

        Args:
            ego_vehicle: The ego vehicle object.
            traffic_manager (optional): The traffic manager object.

        Returns:
            int: 1 if the scenario is completed (either passed or failed), 0 otherwise.
        """
        if self.triggered:
            pedestrian_acting_location = self.pedestrian_acting_actor.get_transform().location
            if self.spawn_with_van:
                if self.checkpoint[0] == False:
                    if pedestrian_acting_location.y > 74.2:
                        self.pedestrian_acting_actor.apply_control(carla.WalkerControl(direction=carla.Vector3D(x=1.0, y=0.0, z=0.0), speed=3.8, jump=False))
                        self.checkpoint[0] = True
                elif self.checkpoint[1] == False:
                    if(is_inside_bounding_box(ego_vehicle.get_transform().location, self.trigger_area[0], self.trigger_area[1]) and (get_vehicle_speed(ego_vehicle) > 0.5)):
                        adjust_pedestrian_velocity(ego_vehicle, self.pedestrian_acting_actor)
                    else:
                        self.pedestrian_acting_actor.apply_control(carla.WalkerControl(direction=carla.Vector3D(x=1.0, y=0.0, z=0.0), speed=2.5, jump=False))
                    if pedestrian_acting_location.x > -6.0:
                        self.pedestrian_acting_actor.apply_control(carla.WalkerControl(direction=carla.Vector3D(x=0.0, y=0.0, z=0.0), speed=0.0, jump=False))
                        self.checkpoint[1] = True
                        return 0
            else:
                if self.checkpoint[0] == False:
                    if(is_inside_bounding_box(ego_vehicle.get_transform().location, self.trigger_area[0], self.trigger_area[1]) and (get_vehicle_speed(ego_vehicle) > 0.5)):
                        adjust_pedestrian_velocity(ego_vehicle, self.pedestrian_acting_actor)
                    else:
                        self.pedestrian_acting_actor.apply_control(carla.WalkerControl(direction=carla.Vector3D(x=-1.0, y=0.0, z=0.0), speed=3.2, jump=False))
                    if pedestrian_acting_location.x < -14.40:
                        self.pedestrian_acting_actor.apply_control(carla.WalkerControl(direction=carla.Vector3D(x=0.0, y=0.0, z=0.0), speed=0.0, jump=False))
                        self.checkpoint[0] = True
                        return 0
            if self.timer > 0:
                self.timer -= 1
            else:
                return 1
            return 0
        elif is_inside_bounding_box(ego_vehicle.get_transform().location, self.trigger_area[0], self.trigger_area[1]):
            self.trigger()
        return 0

    def trigger(self):
        """
        Triggers the scenario, used by the tick method.

        Sets the `triggered` attribute to True, sets the worker pedestrian's pose, applies a control
        to the pedestrian actor, and prints a message indicating that the scenario has been triggered.
        Can be used to trigger the scenario at a specific point in the simulation.
        """
        all_bones = self.pedestrian_static_actor.get_bones() 
        new_bones = []
        for bone in all_bones.bone_transforms:
            if bone.name == 'crl_spine__C':
                rotation = carla.Rotation(roll=40, pitch=0, yaw=0)
                location = [bone.relative.location.x, bone.relative.location.y, bone.relative.location.z]
                new_bones.append((bone.name,carla.Transform(rotation=rotation,location=carla.Location(x=location[0],y=location[1],z=location[2]))))
            if bone.name == 'crl_foreArm__L' or bone.name == 'crl_foreArm__R':
                rotation = carla.Rotation(roll=0, pitch=-50, yaw=0)
                location = [bone.relative.location.x, bone.relative.location.y, bone.relative.location.z]
                new_bones.append((bone.name,carla.Transform(rotation=rotation,location=carla.Location(x=location[0],y=location[1],z=location[2]))))
            if bone.name == 'crl_Head__C':
                rotation = carla.Rotation(roll=-10, pitch=0, yaw=0)
                location = [bone.relative.location.x, bone.relative.location.y, bone.relative.location.z]
                new_bones.append((bone.name,carla.Transform(rotation=rotation,location=carla.Location(x=location[0],y=location[1],z=location[2]))))
            if bone.name == 'crl_shoulder__L':
                rotation = carla.Rotation(roll=-50, pitch=0, yaw=0)
                location = [bone.relative.location.x, bone.relative.location.y, bone.relative.location.z]
                new_bones.append((bone.name,carla.Transform(rotation=rotation,location=carla.Location(x=location[0],y=location[1],z=location[2]))))
            if bone.name == 'crl_shoulder__R':
                rotation = carla.Rotation(roll=50, pitch=0, yaw=180)
                location = [bone.relative.location.x, bone.relative.location.y, bone.relative.location.z]
                new_bones.append((bone.name,carla.Transform(rotation=rotation,location=carla.Location(x=location[0],y=location[1],z=location[2]))))
        self.pedestrian_static_actor.set_bones(carla.WalkerBoneControlIn(new_bones))
        self.pedestrian_static_actor.show_pose()
        if self.spawn_with_van:
            self.pedestrian_acting_actor.apply_control(carla.WalkerControl(direction=carla.Vector3D(x=0.0, y=1.0, z=0.0), speed=3.8, jump=False))
        else:
            self.pedestrian_acting_actor.apply_control(carla.WalkerControl(direction=carla.Vector3D(x=-1.0, y=0.0, z=0.0), speed=3.2, jump=False))
        self.triggered = True
        print('Scenario -Roadworks With Workers- has been triggered!')

    def destroy(self, client):
        """
        Destroys all actors spawned for the scenario.

        Args:
            client: The Carla client object.
        """
        print('\nScenario -Roadworks With Workers- finished!')
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
        ego_vehicle_actor = world.spawn_actor(bp_ego_vehicle, carla.Transform(carla.Location(x=-11.40, y=27.90, z=0.3), carla.Rotation(pitch=0, roll=0, yaw=80)))
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
        traffic_manager.ignore_walkers_percentage(ego_vehicle_actor, 100)

        # Set route for ego vehicle
        vehicle_path = ['Straight', 'Right']
        traffic_manager.set_route(ego_vehicle_actor, vehicle_path)
        traffic_manager.ignore_lights_percentage(ego_vehicle_actor, 100)
        # If ego vehicle with traffic manager is on the outer lane, force lane change to inner lane, otherwise the car would stuck
        #traffic_manager.force_lane_change(ego_vehicle_actor, False)

        # Main loop
        try:
            while True:
                # Artificial delay for images to be saved
                time.sleep(0.1)
                # Check if a scenario is triggered
                if Scenario is None:
                    Scenario = RoadworksWithWorkers(world, ego_vehicle_actor)
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
