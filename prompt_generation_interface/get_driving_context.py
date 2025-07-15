import requests
import xml.etree.ElementTree as ET
import rospy
import base64
import numpy as np
import os
import time
from std_msgs.msg import String
# API Keys
# OPENWEATHER_API_KEY = ''
# TOMTOM_API_KEY = ''
try:
    rospy.init_node('get_driving_context_node')
    ros_available = True
except Exception as e:
    print(f"ROS initialization failed: {e}")
    ros_available = False

IMAGE_PATH = "/home/yupeng/Documents/VLM_test_platform/perception_aggregator/camera_lf_image.jpg"
# Global variables to store the latest data
latest_vehicle_state = None
latest_scenario = None
latest_possible_behaviors = None
latest_passenger_command = None

# ===============================
# CALLBACK FUNCTIONS
# ===============================
def vehicle_state_callback(msg):
    global latest_vehicle_state
    latest_vehicle_state = msg.data if msg.data else None

def scenario_callback(msg):
    global latest_scenario
    latest_scenario = msg.data if msg.data else None

def behavior_callback(msg):
    global latest_possible_behaviors
    latest_possible_behaviors = msg.data if msg.data else None



rospy.Subscriber("/vehicle_state", String, vehicle_state_callback)
rospy.Subscriber("/scenario", String, scenario_callback)
rospy.Subscriber("/possible_behaviors", String, behavior_callback)



# weather information through Openweather API
def get_weather_info(position):
    # Define the API endpoint and the API key
    
    url = "https://api.openweathermap.org/data/2.5/weather"
    parameters = {
        'lat': position[0],
        'lon': position[1],
        'appid': OPENWEATHER_API_KEY
    }

    # Make the HTTP request to the OpenWeatherMap API
    response = requests.get(url, params=parameters)

    # Check if the request was successful
    if response.status_code == 200:
        data = response.json()
        # Extract the necessary information
        main = data['weather'][0]['main']
        description = data['weather'][0]['description']
        city = data['name']
        country = data['sys']['country']
        
        # Format the extracted information into a readable string
        result = f"We are currently in {city}, {country}. The weather is {description} ({main})."
        return result
    else:
        # Return an error message if something goes wrong
        return "The current weather information is not available."


# map information (such as road type and speed limits) through OpenStreetMap API
def get_speed_limit(position):
    # use the current position to get the speed limit
    import overpy
    open_street_map_api = overpy.Overpass()
    result = open_street_map_api.query(f"way(around:100,{position[0]},{position[1]})[maxspeed];out;")

    # parse the result and extract the speed limit information
    speed_limits = []
    street_names = []
    road_types = []
    for way in result.ways:
        speed_limit =  way.tags.get("maxspeed")
        street_name = way.tags.get("name")
        road_type = way.tags.get("highway")
        if speed_limit is not None:
            speed_limits.append(speed_limit)
            street_names.append(street_name)
            road_types.append(road_type)
    
    # format the extracted information into a readable string.
    # check if the speed limit is available, if not return an error message
    if len(speed_limits) == 0:
        return "The speed limit information is currently not available."
    else:
        # get the speed limit with the most occurrences and the corresponding street name and road type
        speed_limit = max(set(speed_limits), key = speed_limits.count)
        street_name = street_names[speed_limits.index(speed_limit)]
        road_type = road_types[speed_limits.index(speed_limit)]
        speed_limit_kmh = int(speed_limit.split()[0]) * 1.60934
        speed_limit = f"{speed_limit} ({speed_limit_kmh:.2f} km/h)"
        result = f"We are currently on street {street_name}, where the speed limit is {speed_limit} and the road type is {road_type}."
        return result


# traffic information through TomTom API
def get_traffic_info(position):
    # Define the API endpoint
    url = "https://api.tomtom.com/traffic/services/4/flowSegmentData/absolute/10/xml"
    parameters = {
        'key': TOMTOM_API_KEY,
        'point': f"{position[0]},{position[1]}",
        'unit': 'mph'
    }

    # Make the HTTP request to the TomTom API
    response = requests.get(url, params=parameters)

    # Check if the request was successful
    if response.status_code == 200:
        # Parse the XML response
        root = ET.fromstring(response.content)
        
        # Extract the current speed and free flow speed from the XML data
        current_speed = root.find('currentSpeed').text
        free_flow_speed = root.find('freeFlowSpeed').text
        
        # Format the extracted information into a readable string
        result = f"Current traffic flow speed is {current_speed} mph, and the free flow speed is {free_flow_speed} mph."
        return result
    else:
        # Return an error message if something goes wrong
        return "Failed to retrieve the traffic information."
    


def wait_for_data(timeout=10):
    start_time = time.perf_counter()
    while time.perf_counter() - start_time < timeout:
        if all([latest_vehicle_state, latest_scenario, latest_possible_behaviors, latest_passenger_command]):
            return True
        rospy.logwarn("Waiting for all sensor data...")
        time.sleep(0.5)
    return False

# ===============================
# LOAD REFINED PROMPT FROM FILE
# ===============================
def load_refined_prompt(filepath="/home/yupeng/Documents/VLM_test_platform/templates/system.txt"):
    try:
        with open(filepath, "r") as file:
            return file.read()
    except Exception as e:
        rospy.logerr(f"Error loading prompt file: {e}")
        return ""

# ===============================
# LOAD IMAGE FROM FILE AND CONVERT TO BASE64
# ===============================
def load_image(image_path):
    try:
        with open(image_path, "rb") as image_file:
            image_data = base64.b64encode(image_file.read()).decode("utf-8")
        return image_data
    except Exception as e:
        rospy.logerr(f"Error loading image from {image_path}: {e}")
        return None



def get_driving_context(command):
    global latest_passenger_command
    latest_passenger_command = command
    # Get the current position
    from localization_adapter.get_location import get_current_position
    position, locating_method = get_current_position()
    if locating_method == 'GNSS':
        locating_method_info = "The current position was obtained using GPS, so the information is accurate."
    else:
        locating_method_info = "The current position was obtained using IP geolocation, so the street name, speed and traffic information may not be accurate."
    # Get the weather information
    weather_info = get_weather_info(position)
    # Get the speed limit information
    speed_limit_info = get_speed_limit(position)
    # Get the traffic information
    traffic_info = get_traffic_info(position)
    
    # Combine the information into a single string
    other_driving_context = f"{locating_method_info}\n{weather_info}\n{speed_limit_info}\n{traffic_info}"
    start_time = time.perf_counter()  # Start high-precision timer
    latest_passenger_command = command

    query = f"""
    The current vehicle state is {latest_vehicle_state}.
    The driving scenario is [{latest_scenario}].
    Possible behaviors are {latest_possible_behaviors}.
    The passenger command is {latest_passenger_command}.
    The front-view camera image is provided.

    Please analyze the situation and provide a thought process before making a decision.
    Then, provide ONLY the final driving action, PID control parameters (kp, ki, kd), and MPC parameters.

    Output format:
    Thought: [Reasoning]
    Action: [Behavior]
    PID: [kp, ki, kd]
    MPC: [param1, param2, param3]
    """

    prompt_time = time.perf_counter() - start_time  # Measure time
    # driving_context = f"{query}\n"
    driving_context = f"{query}\n"
    refined_prompt = load_refined_prompt()
    image_data = load_image(IMAGE_PATH)
    return driving_context, refined_prompt, prompt_time,image_data,other_driving_context


def main():
    global latest_vehicle_state, latest_scenario, latest_possible_behaviors, latest_passenger_command
    if ros_available:
        if not wait_for_data(timeout=3):
            rospy.logwarn("Sensor data not received within timeout. Using default values.")
    else:
        print("ROS is not available. Using default values.")
        # latest_vehicle_state = latest_vehicle_state
        # latest_scenario = latest_scenario
        # latest_possible_behaviors = latest_possible_behaviors
        # latest_passenger_command = latest_passenger_command
    image_data = load_image(IMAGE_PATH)
    driving_context, refined_prompt, prompt_time, image_data, other_driving_context = get_driving_context("default command")
    print("Driving Context:\n", driving_context)
    print("Refined Prompt:\n", refined_prompt)
    print("Prompt Generation Time:", prompt_time)

if __name__ == '__main__':
    main()

