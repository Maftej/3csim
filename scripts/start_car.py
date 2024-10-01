#!/usr/bin/env python

# Copyright (c) 2024 IISLab at the Technical University of Košice.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
sys.path.append(r"C:\carla\PythonAPI\examples\custom_synchronous")
import argparse
import time
import random

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
# Static scenes
from scenario_files import SuddenSignAheadAndRight, StaticEMSObstacle, StaticCarCrash_1
# Dynamic scenes
from scenario_files import BoyFootball, FootballHighway, EMSOutgoing, CarWithItem, DangerousExcavator, PedestrianCallingDog, EmergencyPlane, RunnerInAPark

bounding_box_kid = [(41.00, 133.00), (42.00, 136.00)]
bounding_box_ball = [(172.00, 191.50), (175.00, 198.10)]
bounding_box_EMS_outgoing = [(-69.00, -1.20), (-70.00, 2.20)]
bounding_box_car_with_item = [(-69.00, 2.20), (-70.00, 5.20)]

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='Port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '-s', '--seed',
        metavar='S',
        default=None,
        type=int,
        help='Set random device seed and deterministic mode for Traffic Manager')
    argparser.add_argument(
        '-t', '--timer',
        metavar='T',
        default=800,
        type=int,
        choices=range(200, 2000),
        help='Set the maximum time for the scenario to run in range 200-2000 seconds (default: 800)')
    argparser.add_argument(
        '-rgb', '--rgbcamera',
        metavar='RGB',
        default=True,
        type=bool,
        help='Create RGB camera for ego vehicle (default: False)')
    argparser.add_argument(
        '-semantic', '--semanticcamera',
        metavar='SEMANTIC',
        default=True,
        type=bool,
        help='Create Semantic camera for ego vehicle (default: False)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='Camera image resolution (default: 1280x720)')

    args = argparser.parse_args()
    args.camera_width, args.camera_height = [int(x) for x in args.res.split('x')]
    args.camera_fov = 90

    path = 'E:/CARLA/images'
    vehicles_list = []
    walkers_list = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(5.0)
    random.seed(args.seed if args.seed is not None else int(time.time()))

    try:
        world = client.get_world()
        settings = world.get_settings()

        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        if args.seed is not None:
            traffic_manager.set_random_device_seed(args.seed)
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
        spawn_points = world.get_map().get_spawn_points()
        ego_vehicle_actor = world.spawn_actor(bp_ego_vehicle, spawn_points[112])
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
        # Add camera sensors if selected
        if args.rgbcamera:
            # Create RGB camera for ego vehicle
            bp_camera = blueprint_library.find('sensor.camera.rgb')
            bp_camera.set_attribute("image_size_x",str(args.camera_width))
            bp_camera.set_attribute("image_size_y",str(args.camera_height))
            bp_camera.set_attribute("fov",str(args.camera_fov))
            bp_camera.set_attribute("sensor_tick",str(0.2))
            rgb_camera_transform = carla.Transform(carla.Location(2,0,1), carla.Rotation(0,0,0))
            rgb_camera = world.spawn_actor(bp_camera, rgb_camera_transform, attach_to=ego_vehicle_actor, attachment_type=carla.AttachmentType.Rigid)
            vehicles_list.insert(len(vehicles_list)-1,rgb_camera)
            rgb_camera.listen(lambda image: image.save_to_disk(path + '/rgb/%.6d.png' % image.frame))
        if args.semanticcamera:
            # Create Semantic camera for ego vehicle
            bp_semantic_camera = blueprint_library.find('sensor.camera.semantic_segmentation')
            bp_semantic_camera.set_attribute("image_size_x",str(args.camera_width))
            bp_semantic_camera.set_attribute("image_size_y",str(args.camera_height))
            bp_semantic_camera.set_attribute("fov",str(args.camera_fov))
            bp_semantic_camera.set_attribute("sensor_tick",str(0.2))
            semantic_camera_transform = carla.Transform(carla.Location(2,0,1), carla.Rotation(0,0,0))
            semantic_camera = world.spawn_actor(bp_semantic_camera, semantic_camera_transform, attach_to=ego_vehicle_actor, attachment_type=carla.AttachmentType.Rigid)
            vehicles_list.insert(len(vehicles_list)-1,semantic_camera)
            semantic_camera.listen(lambda image: image.save_to_disk(path + '/semantic/%.6d.png' % image.frame,carla.ColorConverter.CityScapesPalette))
        
        # Tick world to update the simulation
        sucess = 0
        world.tick()
        scenario_timers = []
        # List of possible scenarios
        scenarios = [BoyFootball(world, ego_vehicle_actor),
                     FootballHighway(world, ego_vehicle_actor),
                     EMSOutgoing(world, ego_vehicle_actor),
                     CarWithItem(world, ego_vehicle_actor),
                     EmergencyPlane(world, ego_vehicle_actor),
                     SuddenSignAheadAndRight(world, ego_vehicle_actor),
                     PedestrianCallingDog(world, ego_vehicle_actor),
                     DangerousExcavator(world, ego_vehicle_actor),
                     StaticEMSObstacle(world, ego_vehicle_actor),
                     RunnerInAPark(world, ego_vehicle_actor),
                     StaticCarCrash_1(world, ego_vehicle_actor)]
        # List of scenario timers
        for _ in scenarios:
            scenario_timers.append(args.timer)

        # Set route for ego vehicle
        vehicle_path = ['Right', 'Right', 'Straight', 'Straight', 'Right', 'Left', 'Left', 'Straight', 'Straight', 'Right', 'Straight', 'Straight', 'Straight', 'Straight', 'Straight', 'Left']
        traffic_manager.set_route(ego_vehicle_actor, vehicle_path)
        traffic_manager.ignore_lights_percentage(ego_vehicle_actor, 100)

        # Main loop
        try:
            while True:
                time.sleep(0.1)
                # Here insert your custom controller for ego vehicle
                #throttle, steer = 0.5, 0.0 # YourControl()
                #ego_vehicle_actor.apply_control(carla.VehicleControl(throttle=throttle, steer=steer))
                # Check if a scenario is active
                if scenarios is None:
                    break
                else:
                    for index, scenario in enumerate(scenarios):
                        if scenario_timers[index] > 0:
                            sucess = scenario.tick(ego_vehicle_actor, traffic_manager=traffic_manager)
                            if scenario.triggered == 1:
                                if sucess == 1:
                                    scenario_timers[index] = 0
                                else:
                                    scenario_timers[index] -= 1
                        else:
                            scenario.destroy(client)
                            scenarios.pop(index)
                            scenario_timers.pop(index)
                        
                world.tick()
        except KeyboardInterrupt:
            pass

    finally:
        if collision_sensor is not None:
            collision_sensor.stop()
        # Stop camera sensors to avoid errors
        if rgb_camera is not None:
            rgb_camera.stop()
        if semantic_camera is not None:
            semantic_camera.stop()
        world.tick()
        
        # Disable synchronous mode
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.no_rendering_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        # Destroy scenario
        for scenario in scenarios:
            scenario.destroy(client)
        # Destroy all actors
        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
        print('\ndestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in walkers_list])

        time.sleep(0.5)
        print('done.')


if __name__ == '__main__':

    main()
