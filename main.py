import os
import re
import subprocess
import argparse
import speech_recognition as sr
from langchain_openai import ChatOpenAI
from openai import OpenAI
import google.generativeai as palm

from argparse import Namespace
from model.dbl_llm import DBL
# from model.SMDM.smdm import SMDM
from utils.get_latency import ping
from utils.device_list import get_device_index
from utils.get_key_press import get_key_press
from utils.log_gen import record_timestamp, output_to_file
from model.dbl_v_llm import get_driving_context, query_gpt4o, run


# Uncomment to automatically set up the API key in the environment
os.environ['OPENAI_API_KEY'] = None

# Parse arguments
parser = argparse.ArgumentParser()
parser.add_argument('-m', '--microphone_index', type=int, default=0, help='Index of the microphone device. Use utils/device_list.py to get the index.')
parser.add_argument('--microphone_name', type=str, default='USB Device', help='Microphone name if index is not provided.')
parser.add_argument('--model_name', type=str, default='gpt-4', help='Model name for the LLM. See dbl_llm/dbl_llm.py for supported models.')
parser.add_argument('--template_name', type=str, default='parking', help='Template name for the LLM. Stored in templates/ directory.')
parser.add_argument('-c','--enable_driving_context', type=bool, default=True, help='Enable driving context in the input.')
parser.add_argument('-t1','--text_input_1', type=str, default=None, help='Preload up to 2 text inputs, the first input.')
parser.add_argument('-t2','--text_input_2', type=str, default=None, help='Preload up to 2 text inputs, the second input.')
args: Namespace = parser.parse_args()


def process_command(sr_recognizer, source):
    # Process text input
    if args.text_input_1 or args.text_input_2:
        mic_ready_time = record_timestamp("Microphone ready")
        recording_done_time = record_timestamp("Done recording command")
        if args.text_input_1:
            command = args.text_input_1
            args.text_input_1 = None
        else:
            command = args.text_input_2
            args.text_input_2 = None
        command_detected_time = record_timestamp("Done detecting command", recording_done_time)      
    else:    
        # Record voice input
        sr_recognizer.adjust_for_ambient_noise(source)
        mic_ready_time = record_timestamp("Microphone ready")
        audio = sr_recognizer.listen(source)
        recording_done_time = record_timestamp("Done recording command", mic_ready_time)
        # Audio to text
        try:
            command = sr_recognizer.recognize_google(audio)
        except sr.RequestError as e:
            print("Could not request results from Google API")
            return
        command_detected_time = record_timestamp("Done detecting command", recording_done_time)
    
    print(f"\"{command}\"")
    
    # Add Driving Context
    if args.enable_driving_context:
        driving_context, refined_prompt, prompt_time, image_data,other_driving_context = get_driving_context(command)
        print(f"Driving Context:\n{driving_context}")
        llm_input = other_driving_context+'\n'+command
    else:
        driving_context, refined_prompt, prompt_time, image_data,other_driving_context = get_driving_context(command)
        print(f"Driving Context:\n{driving_context}")
        llm_input = command

    if args.model_name == "VLM":
        print("Generating action using VLM functions...")
        openai_api_key = os.environ['OPENAI_API_KEY']
        thought_response, action_response, pid_values, mpc_values, reasoning_time, action_time = query_gpt4o(
            driving_context, image_data, refined_prompt, openai_api_key
        )
        print("\n=== GPT-4o Thought Process ===\n")
        print(thought_response)
        print("\n=== Selected Driving Action ===")
        print(f"\n**\n{action_response}\nPID {pid_values}\nMPC {mpc_values}\n**")
        # Execute the action using the run function
        run(action_response, pid_values, mpc_values)
        action = action_response
    else:    
        action = DBL(
                llm_model_name=args.model_name,
                template_name=args.template_name,
                memory_enable= False,
                memory_path=None
            ).run(llm_input)

        
    # clean up the action
    print("action")
    print(action)
    match = re.search(r'Action:\s*(.*)', action, flags=re.DOTALL)
    if match:
        action = match.group(1)
    action_output_time = record_timestamp("Done generating action", command_detected_time)
    print(f"\"{action}\"")
    
    # Execute action code
    subprocess.Popen(action,
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE)
    action_executed_time = record_timestamp("Action executed successfully", action_output_time)
    
    # Output to log file
    output_to_file(detected_command=command,
                output_command=action,
                timestamps=[('Microphone Ready', mic_ready_time),
                            ('Done Recording Command', recording_done_time),
                            ('Done Detecting Command', command_detected_time),
                            ('Done Generating Action', action_output_time),
                            ('Action Executed Successfully', action_executed_time)])
    return command, action


if __name__ == '__main__':
    # Test internet latency
    ping_s = ping('https://api.openai.com') * 1000
    print(f"Current ping to api.openai.com: {ping_s:.2f} ms")

    # Get microphone index if not provided
    if args.microphone_index is None:
        args.microphone_index = get_device_index(args.microphone_name)

    command = ''
    action = ''
    while True:
        # Initialize microphone
        sr_recognizer = sr.Recognizer()
        with sr.Microphone(device_index=args.microphone_index) as source:
            # Wait for key press
            print("Listening for key words...")
            key = get_key_press().lower()
            
            # Press 'c' or 'Enter' to command, 'e' to evaluate, 'q' to quit
            if key == 'c' or key == '\r' or key == '\n':  # '\r' or '\n' for Enter key
                print("Recording command...")
                command, action = process_command(sr_recognizer=sr_recognizer, source=source)
            elif key == 'q':
                print("Quitting script...")
                break
