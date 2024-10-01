#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
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
import time
import random

def main():
    actor_list = []

    try:
        # First of all, we need to create the client that will send the requests
        # to the simulator. Here we'll assume the simulator is accepting
        # requests in the localhost at port 2000.
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Once we have a client we can retrieve the world that is currently
        # running.
        world = client.get_world()

        # The world contains the list blueprints that we can use for adding new
        # actors into the simulation.
        blueprint_library = world.get_blueprint_library()

        # Now let's filter all the blueprints of type 'vehicle' and choose football
        bp_barrel = blueprint_library.find('static.prop.barrel')
        bp_pedestrian = random.choice(blueprint_library.filter('walker.pedestrian.' + str(random.randint(1, 8)).zfill(4)))
        bp_truck = blueprint_library.find('vehicle.mercedes.sprinter')
        bp_handtruck = blueprint_library.find('static.prop.handtruck')

        # Set the walker to a non invincible mode
        if bp_pedestrian.has_attribute('is_invincible'):
            bp_pedestrian.set_attribute('is_invincible', 'false')

        # Now we need to give an initial transform to the vehicle.
        barrel_transform = carla.Transform(carla.Location(x=205.90, y=56.25, z=0.07), carla.Rotation(pitch=0, yaw=-10, roll=0))
        pedestrian_transform = carla.Transform(carla.Location(x=200.10, y=52.50, z=1.20), carla.Rotation(pitch=0, yaw=0, roll=0))
        barrel_prop1_transform = carla.Transform(carla.Location(x=204.70, y=56.30, z=0.0), carla.Rotation(pitch=0, yaw=130, roll=0))
        barrel_prop2_transform = carla.Transform(carla.Location(x=205.30, y=53.60, z=0.0), carla.Rotation(pitch=0, yaw=70, roll=0))
        truck_prop_transform = carla.Transform(carla.Location(x=202.10, y=54.80, z=0.2), carla.Rotation(pitch=0, yaw=180, roll=0))
        handtruck_prop_transform = carla.Transform(carla.Location(x=205.80, y=56.00, z=0.1), carla.Rotation(pitch=0, yaw=0, roll=0))

        # Tell the world to spawn the vehicle.
        barrel_actor = world.spawn_actor(bp_barrel, barrel_transform)
        actor_list.append(barrel_actor)
        pedestrian_actor = world.spawn_actor(bp_pedestrian, pedestrian_transform)
        actor_list.append(pedestrian_actor)
        barrel_prop1_actor = world.spawn_actor(bp_barrel, barrel_prop1_transform)
        actor_list.append(barrel_prop1_actor)
        barrel_prop2_actor = world.spawn_actor(bp_barrel, barrel_prop2_transform)
        actor_list.append(barrel_prop2_actor)
        truck_prop_actor = world.spawn_actor(bp_truck, truck_prop_transform)
        actor_list.append(truck_prop_actor)
        handtruck_prop_actor = world.spawn_actor(bp_handtruck, handtruck_prop_transform)
        actor_list.append(handtruck_prop_actor)
        
        # Set the pedestrian to simulate physics and make it walk
        pedestrian_actor.set_simulate_physics(True)
        pedestrian_actor.apply_control(carla.WalkerControl(direction=carla.Vector3D(x=1.0, y=0.0, z=0.0), speed=1.0, jump=False))
        
        # Wait for 8 seconds
        time.sleep(6.1)
        print(pedestrian_actor.get_transform().location)
        
        # Apply control to the pedestrian_actor to make it walk towards the barrel_actor
        pedestrian_actor.apply_control(carla.WalkerControl(direction=carla.Vector3D(x=0.0, y=1.0, z=0.0), speed=0.8, jump=False))
        time.sleep(4)
        pedestrian_actor.apply_control(carla.WalkerControl(direction=carla.Vector3D(x=1.0, y=0.0, z=0.0), speed=0, jump=False))
        print(pedestrian_actor.get_transform().location)

        # Set the vehicle to simulate physics
        barrel_actor.set_simulate_physics(True)
        barrel_actor.set_enable_gravity(True)

        # Add impulse to the ball
        barrel_actor.add_impulse(carla.Vector3D(x=-10.0, y=40.0, z=-1.0))
        
        # Wait for 20 seconds then kill the object
        time.sleep(20)

        # It is important to note that the actors we create won't be destroyed
        # unless we call their "destroy" function. If we fail to call "destroy"
        # they will stay in the simulation even after we quit the Python script.
        # For that reason, we are storing all the actors we create so we can
        # destroy them afterwards.

    finally:
        print('destroying actors')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        print('done.')


if __name__ == '__main__':

    main()
