#!/usr/bin/env python

# Copyright (c) 2024 IISLab at the Technical University of Košice.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Function to check if the vehicle is inside the bounding box
def is_inside_bounding_box(vehicle_location, bbox_min, bbox_max):
    """
    Check if a given vehicle location is inside a bounding box defined by the minimum and maximum coordinates.

    Parameters:
    - vehicle_location: The location of the vehicle (carla.Location object).
    - bbox_min: The minimum coordinates of the bounding box (tuple of x and y values).
    - bbox_max: The maximum coordinates of the bounding box (tuple of x and y values).

    Returns:
    - True if the vehicle location is inside the bounding box, False otherwise.
    """
    return (min(bbox_min[0], bbox_max[0]) <= vehicle_location.x <= max(bbox_min[0], bbox_max[0]) and
            min(bbox_min[1], bbox_max[1]) <= vehicle_location.y <= max(bbox_min[1], bbox_max[1]))

# Function to get the vehicle speed
def get_vehicle_speed(vehicle):
    """
    Calculate the speed of a vehicle based on its velocity.

    Parameters:
    - vehicle: The vehicle object for which to calculate the speed.

    Returns:
    - The speed of the vehicle in meters per second.
    """
    return (((vehicle.get_velocity().x ** 2) + (vehicle.get_velocity().y ** 2) + (vehicle.get_velocity().z ** 2)) ** 0.5)

# Function to update trigger points based on the vehicle speed
def update_trigger_points(trigger_points, actor_speed, add_speed=True, add_to_x=True):
    """
    Update the trigger points based on the actor's speed and direction.

    Args:
        trigger_points (list): A list of two points representing the trigger points.
        actor_speed (float): The speed of the actor.
        add_speed (bool, optional): Flag indicating whether to add or subtract the actor's speed. Defaults to True.
        add_to_x (bool, optional): Flag indicating whether to add or subtract the speed to the x-coordinate of the trigger points. Defaults to True.

    Returns:
        list: A list of two updated trigger points.

    """
    if add_speed:
        if add_to_x:
            return [(trigger_points[0][0] + actor_speed, trigger_points[0][1]),
                    (trigger_points[1][0] + actor_speed, trigger_points[1][1])]
        else:
            return [(trigger_points[0][0], trigger_points[0][1] + actor_speed),
                    (trigger_points[1][0], trigger_points[1][1] + actor_speed)]
    else:
        if add_to_x:
            return [(trigger_points[0][0] - actor_speed, trigger_points[0][1]),
                    (trigger_points[1][0] - actor_speed, trigger_points[1][1])]
        else:
            return [(trigger_points[0][0], trigger_points[0][1] - actor_speed),
                    (trigger_points[1][0], trigger_points[1][1] - actor_speed)]

# Update only the beginning area of the trigger points
def update_beginning_area(array, actor_speed, add_speed=True, add_to_x=True):
    """
    Modify the array by adding or subtracting the actor's speed to either the x-coordinate or y-coordinate.

    Parameters:
    - array: The array to be modified.
    - actor_speed: The speed of the actor.
    - add_speed: Flag indicating whether to add or subtract the actor's speed. Defaults to True (add).
    - add_to_X: Flag indicating whether to add or subtract the speed to the x-coordinate. Defaults to True (x-axis).

    Returns:
    - The modified array.
    """
    if add_speed:
        if add_to_x:
            return [array[0] + actor_speed, array[1]]
        else:
            return [array[0], array[1] + actor_speed]
    else:
        if add_to_x:
            return [array[0] - actor_speed, array[1]]
        else:
            return [array[0], array[1] - actor_speed]
        
# Function to adjust the car's location based on its forward vector and bumper offset
def adjust_car_location(car_location, car_transform, bumper_offset=2.0):
    """
    Adjusts the car's location based on the car's transform and a bumper offset.

    Parameters:
    car_location (carla.Location): The current location of the car.
    car_transform (carla.Transform): The transform of the car.
    bumper_offset (float, optional): The distance by which to adjust the car's location. Defaults to 2.0.

    Returns:
    carla.Location: The adjusted location of the car.
    """
    forward_vector = car_transform.get_forward_vector()
    adjusted_location = car_location
    adjusted_location.x += forward_vector.x * bumper_offset
    adjusted_location.y += forward_vector.y * bumper_offset
    adjusted_location.z += forward_vector.z * bumper_offset
    return adjusted_location

# Function to calculate the required pedestrian speed for intersection with ego vehicle
def calculate_required_speed(car_location, car_velocity, pedestrian_location, pedestrian_direction):
    """
    Calculates the required speed for a pedestrian to intersect the path of a moving car.

    Parameters:
    - car_location (Vector3D): The current location of the car.
    - car_velocity (Vector3D): The velocity vector of the car.
    - pedestrian_location (Vector3D): The current location of the pedestrian.
    - pedestrian_direction (Vector3D): The direction of movement of the pedestrian.

    Returns:
    - required_speed (float): The speed at which the pedestrian should move to intersect the car's path.

    The function calculates the car's speed and determines the pedestrian's movement direction.
    It then calculates the distance and time to intersect between the car and the pedestrian.
    Finally, it calculates the required speed for the pedestrian to intersect the car's path.

    Note: The function assumes that the car's velocity and pedestrian's direction are in the same coordinate system.
    """
    # Calculate the car's speed (magnitude of the velocity vector)
    car_speed = ((car_velocity.x ** 2 + car_velocity.y ** 2 + car_velocity.z ** 2) ** 0.5) # Speed in m/s
    
    if car_speed < 0.1:
        return 0  # If the car is not moving, return 0 speed for pedestrian
    
    # Determine the pedestrian movement direction
    if pedestrian_direction.x != 0:
        # Pedestrian is moving in the x-direction
        distance_to_pedestrian_path = abs(car_location.y - pedestrian_location.y)
        time_to_intersect = distance_to_pedestrian_path / car_speed
        distance_to_intersect = abs(car_location.x - pedestrian_location.x)
    else:
        # Pedestrian is moving in the y-direction
        distance_to_pedestrian_path = abs(car_location.x - pedestrian_location.x)
        time_to_intersect = distance_to_pedestrian_path / car_speed
        distance_to_intersect = abs(car_location.y - pedestrian_location.y)
    
    # Calculate the required speed for the pedestrian
    required_speed = distance_to_intersect / time_to_intersect
    return required_speed

# Adjust the pedestrian velocity based on the ego vehicle speed
def adjust_pedestrian_velocity(ego_vehicle, pedestrian):
    """
    Adjusts the velocity of a pedestrian based on the position and velocity of an ego vehicle.

    Args:
        ego_vehicle (carla.Vehicle): The ego vehicle.
        pedestrian (carla.Walker): The pedestrian.

    Returns:
        None
    """
    car_transform = ego_vehicle.get_transform()
    car_location = ego_vehicle.get_transform().location
    car_velocity = ego_vehicle.get_velocity()
    
    pedestrian_location = pedestrian.get_transform().location
    walker_control = pedestrian.get_control()  # Get the current WalkerControl
    pedestrian_direction = walker_control.direction  # Get the current direction 
    # Adjust the car's location based on its bumper offset
    adjusted_car_location = adjust_car_location(car_location, car_transform)
    # Calculate the required speed for the pedestrian to intersect the car's path   
    required_speed = calculate_required_speed(adjusted_car_location, car_velocity, pedestrian_location, pedestrian_direction)
    # Set a minimum speed for the pedestrian if required speed is too low
    if required_speed < 0.1:
        required_speed = 0.1  # Set a minimum speed for the pedestrian
    walker_control.speed = required_speed  # Update the speed, the direction remains unchanged
    # Apply WalkerControl to the pedestrian
    pedestrian.apply_control(walker_control)
