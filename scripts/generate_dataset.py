import os
import shutil
import subprocess

# List of scenarios
# Scenario name, Python script name
scenarios = [
    ('PedestrianAdvesarialPlayground', 'spawn_pedestrian_adversarial_playground.py'), # Done, can be failed by complying with the sign on the shirt - used for LLM reasoning evaluation
    
    ('FootballHighway', 'spawn_ball_highway.py'), # Done
    ('RoadworksWithExcavator', 'spawn_dangerous_excavator.py'), # Done, barriers on the ground, no excavator arm movement
    ('ParkingHGV', 'spawn_parking_HGV.py'), # Done, by default the ego vehicle will pass the HGV
    ('ShoppingcartRollingDown', 'spawn_shoppingcart_highway,py') # Done, by default the ego vehicle will near miss the shopping cart

    ('SquirrelRunningAcrossTheRoad', 'spawn_squirrel_running_across.py'), # Done, by default the ego vehicle will pass above the squirrel
    # Adversarial scenarios for visual neural network evaluation
    ('PedestrianAdvesarial_front_ENDALL', 'spawn_pedestrian_advesarial_front_ENDALL.py') # Done, overweight pedestrian with a end all restriction sign on his shirt
    ('PedestrianAdvesarial_front_LEFTRIGHT', 'spawn_pedestrian_advesarial_front_LEFTRIGHT.py') # Done, sport pedestrian with a left/right turn only sign on her shirt
    ('PedestrianAdvesarial_front_STOP', 'spawn_pedestrian_advesarial_front_STOP.py') # Done, pedestrian with a STOP sign on his shirt
    ('PedestrianAdvesarial_back_ENDALL', 'spawn_pedestrian_advesarial_back_ENDALL.py') # Done, overweight pedestrian with a end all restriction sign on his shirt
    ('PedestrianAdvesarial_back_LEFTRIGHT', 'spawn_pedestrian_advesarial_back_LEFTRIGHT.py') # Done, sport pedestrian with a left/right turn only sign on her shirt
    ('PedestrianAdvesarial_back_STOP', 'spawn_pedestrian_advesarial_back_STOP.py') # Done, sport pedestrian with a left/right turn only sign on her shirt
    ('CarForwardParkingAndReverseNoHarm', 'spawn_car_forward_parking_noharm.py'), # Done, by default the ego vehicle will pass the car
    ('CowWithPedestrianCrossing','spawn_cow_and_pedestrian_crossing.py') # Done, by default the ego vehicle will let the cow pass
    ('LooseZooAnimals', 'spawn_loose_zoo_animals.py'), # Done, by default the ego vehicle will run over both animals
    # The following scenarios are not suitable for the current setup (traffic manager autopilot)
    ('CarReverseParkingWithDoors', 'spawn_car_reverse_parking_doors.py'), # Done, by default the ego vehicle will hit the car doors
    ('CarForwardParkingWithDoors', 'spawn_car_forward_parking_doors.py'), # Done, by default the ego vehicle will hit the car doors
    ('CarForwardStoppingDropoff_MISS', 'spawn_car_forward_dropoff_MISS.py'), # Done, by default the ego vehicle will stop before hitting the car
    ('CarForwardStoppingDropoff', 'spawn_car_forward_dropoff.py'), # Done, by default the ego vehicle will hit the car
    ('DangerousExcavator', 'spawn_roadworks_with_excavator.py'), # Done, excavator moves arm into the road
    ('EmergencyPlane', 'spawn_emergency_plane.py'), # Done, by default the ego vehicle will crash into the plane
    ('SuddenSignAheadAndRight', 'spawn_static_sign_aheadandright.py'), # Done, can be failed by turning left
    ('StaticEMSObstacle', 'spawn_static_EMS_obstacle.py'), # Done, can be failed by not passing the obstacle in time (by not moving into left lane)
    ('StaticCarCrash_1', 'spawn_static_car_crash_1.py'), # Done, by default the ego vehicle will pass the obstacle
    ('StaticCarCrash_2', 'spawn_static_car_crash_2.py'), # Done, by default the ego vehicle will NOT pass the obstacle
    ('StaticCarCrash_3', 'spawn_static_car_crash_3.py'), # Done, by default the ego vehicle will pass the cars
    ('FallenTree', 'spawn_fallen_tree.py'), # Done, will be failed always
    ('PedestrianCallingDog', 'spawn_pedestrian_calling_dog.py'), # Done, will be failed always
    ('CarInOppositeDirection_HIT', 'spawn_car_in_opposite_direction_HIT.py'), # Done, will be failed always
    ('CarInOppositeDirection_MISS', 'spawn_car_in_opposite_direction_MISS.py'), # Done, will be passed always
    ('CarInOppositeDirection_MISTAKE', 'spawn_car_in_opposite_direction_MISTAKE.py'), # Done, will be passed always
    ('BusObscuringCar_MISS', 'spawn_bus_obscuring_car_MISS.py'), # Done, will be passed always
    ('BusObscuringCar_HIT', 'spawn_bus_obscuring_car.py'), # Done, will be failed always
    # Can be failed by ignore_vehicles or ignore_walkers - set the flag to True in the script
    ('RoadworksWithWorkers_NOVAN', 'spawn_roadworks_with_workers_NOVAN.py'), # Done, can be failed by ignore_walkers
    ('RoadworksWithWorkers_VAN', 'spawn_roadworks_with_workers_VAN.py'), # Done, can be failed by ignore_walkers
    ('BoyFootball', 'spawn_ball_boy.py'), # Done, can be failed by ignore_walkers
    ('BusStopNearPlayground', 'spawn_bus_stop_near_playground.py'), # Done, can be failed by ignore_walkers
    ('RunnerInAPark', 'spawn_runner_park.py'), # Done, can be failed by ignore_walkers
    ('EMSOutgoing', 'spawn_EMS_outgoing.py'), # Done, can be failed by ignore_vehicles, has two possible turns (left or right)
    ('CarWithItem_GUITARCASE', 'spawn_car_with_item_GUITARCASE.py'), # Done, can be failed by ignore_vehicles, one of three possible items
    ('CarWithItem_BRIEFCASE', 'spawn_car_with_item_BRIEFCASE.py'), # Done, can be failed by ignore_vehicles, one of three possible items
    ('CarWithItem_TRAVELCASE', 'spawn_car_with_item_TRAVELCASE.py'), # Done, can be failed by ignore_vehicles, one of three possible items
    ]

# List of folder names (daytime_weather conditions)
folders = [
    'day_clear', 'day_rainy', 'day_foggy', 'day_worst',
    'sunset_clear', 'sunset_rainy', 'sunset_foggy', 'sunset_worst',
    'night_clear', 'night_rainy', 'night_foggy', 'night_worst',
    'sunrise_clear', 'sunrise_rainy', 'sunrise_foggy', 'sunrise_worst',
    'corner_clear', 'corner_rainy', 'corner_foggy', 'corner_worst'
]

# Define paths to python scripts
main_script_path = 'C:/carla/PythonAPI/examples/custom_synchronous/'
python_script_1 = os.path.join(main_script_path, 'weather_control.py')
python_script_2 = None
python_script_3 = 'E:/CARLA/images/img_to_video.py'

# Path to the main folder containing all the daytime_weather folders
main_folder_path = 'E:/CARLA/corner cases carla/successful/'

# Path to the E_folder where the 'rgb' and 'semantic' folders, and the 'video.mp4' file are located
E_folder_path = 'E:/CARLA/images/'

# Iterate through each scenario
for scenario_name, scenario_path in scenarios:
    python_script_2 = os.path.join(main_script_path, scenario_path)
    # Iterate through each daytime_weather folder
    for folder_name in folders:
        # Define the path to the current daytime_weather folder
        folder_path = os.path.join(main_folder_path, folder_name)
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
        
        if os.path.exists(folder_path):
            # Create the 'scenario_name' folder inside the current daytime_weather folder
            scenario_folder_path = os.path.join(folder_path, scenario_name)
            os.makedirs(scenario_folder_path, exist_ok=True)
            
            # Run the first Python script
            subprocess.run(['python', python_script_1], check=True)
            
            # Run the second Python script
            subprocess.run(['python', python_script_2], check=True)

            # Run the third Python script
            subprocess.run(['python', python_script_3], check=True)
            
            # Move the 'rgb' and 'semantic' folders, and the 'video.mp4' file to the 'scenario_name' folder
            for item in ['rgb', 'semantic', 'video.mp4']:
                source_path = os.path.join(E_folder_path, item)
                destination_path = os.path.join(scenario_folder_path, item)
                
                if os.path.exists(source_path):
                    if os.path.isdir(source_path):
                        shutil.move(source_path, destination_path)
                    elif os.path.isfile(source_path):
                        shutil.move(source_path, scenario_folder_path)

            print(f'Successfully processed and moved items to {scenario_folder_path}')
        else:
            print(f'Folder {folder_name} does not exist in {main_folder_path}')

print("All tasks completed successfully.")
