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
import time

# List of possible sun states in the form of (altitude, azimuth)
sun_states = [(45.0, 240.0), # Noon
              (2.0, 268.0), # Sunset
              (-88.0, 320.0), # Night
              (1.05, 28.28), # Sunrise
              (0.01, 110.0) # Almost night, no streetlights
              ]

# List of possible storm states in the form of (clouds, rain, puddles, wetness, fog, wind)
storm_states = [(0.0, 0.0, 0.0, 0.0, 0.0, 5.0), # Clear - optimal conditions
                (90.0, 80.0, 85.0, 100.0, 30.0, 90.0), # Storm
                (80.0, 10.0, 5.0, 50.0, 60.0, 10.0), # Foggy
                (100.0, 100.0, 100.0, 100.0, 100.0, 100.0), # worst conditions possible
                ]

class Weather(object):
    def __init__(self, weather):
        self.weather = weather

    def set_weather_sun(self, altitude_angle=0.0, azimuth_angle=0.0):
        self.weather.sun_azimuth_angle = azimuth_angle
        self.weather.sun_altitude_angle = altitude_angle

    def set_weather_storm(self, clouds=0.0, rain=0.0, puddles=0.0, wetness=0.0, fog=0.0, wind=0.0):
        self.weather.cloudiness = clouds
        self.weather.precipitation = rain
        self.weather.precipitation_deposits = puddles
        self.weather.wind_intensity = wind
        self.weather.fog_density = fog
        self.weather.wetness = wetness

    def get_weather_sun(self):
        return round(self.weather.sun_altitude_angle, 2), round(self.weather.sun_azimuth_angle, 2)
    
    def get_weather_storm(self):
        return self.weather.cloudiness, self.weather.precipitation, self.weather.precipitation_deposits, self.weather.wetness,  self.weather.fog_density, self.weather.wind_intensity

    def __str__(self):
        return 'Sun(alt: %.2f, azm: %.2f) Storm(clouds=%d%%, rain=%d%%, wetness = %d%%, puddles = %d%%, fog=%d%%, wind=%d%%)' % (self.weather.sun_altitude_angle, self.weather.sun_azimuth_angle, self.weather.cloudiness, self.weather.precipitation, self.weather.wetness, self.weather.precipitation_deposits, self.weather.fog_density, self.weather.wind_intensity)

# Get the current sun and weather state, compare it to the lists and set it to the next weather state from the list,
# for each sun state, circle all storm states; if the current weather is not from the list, set it to the first one
def world_change_weather(world, sun_states=sun_states, storm_states=storm_states):
    weather = Weather(world.get_weather())

    current_sun = weather.get_weather_sun()
    current_storm = weather.get_weather_storm()

    # If the current weather state is not in the lists, set it to the first one
    if (current_sun not in sun_states) or (current_storm not in storm_states):
        weather.set_weather_sun(altitude_angle=sun_states[0][0], azimuth_angle=sun_states[0][1])
        weather.set_weather_storm(clouds=storm_states[0][0], rain=storm_states[0][1], puddles=storm_states[0][2], wetness=storm_states[0][3], fog=storm_states[0][4], wind=storm_states[0][5])
        world.set_weather(weather.weather)
        print(weather)
        return

    for index_sun, sun in enumerate(sun_states):
        if current_sun == sun:
            for index_storm, storm in enumerate(storm_states):
                if current_storm == storm:
                    if index_storm == len(storm_states) - 1:
                        next_storm = storm_states[0]
                        if index_sun == len(sun_states) - 1:
                            next_sun = sun_states[0]
                        else:
                            next_sun = sun_states[index_sun + 1]
                        weather.set_weather_sun(altitude_angle=next_sun[0], azimuth_angle=next_sun[1])
                    else:
                        next_storm = storm_states[index_storm + 1]
                    weather.set_weather_storm(clouds=next_storm[0], rain=next_storm[1], puddles=next_storm[2], wetness=next_storm[3], fog=next_storm[4], wind=next_storm[5])
                    world.set_weather(weather.weather)
                    print(weather)
                    return

# Demonstrate the use of the Weather states
def main():
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(5.0)
    world = client.get_world()

    # Cycle through all the possible weather states
    #for i in range(len(sun_states) * len(storm_states)):
    #    world_change_weather(world)
    #    time.sleep(5)
    
    # Reset the weather to the first state
    world_change_weather(world)
    #print(weather)

if __name__ == '__main__':

    main()
