import os
import argparse
import speech_recognition as sr
from utils.device_list import get_device_index
from utils.get_key_press import get_key_press

# Set your OpenAI API key if needed for Whisper API
# os.environ['OPENAI_API_KEY'] = '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'

parser = argparse.ArgumentParser()
parser.add_argument('-m', '--microphone_index', type=int, default=0, help='Index of the microphone device. Use utils/device_list.py to get the index.')
parser.add_argument('--microphone_name', type=str, default='USB Device', help='Microphone name if index is not provided.')
args = parser.parse_args()

if args.microphone_index is None:
    args.microphone_index = get_device_index(args.microphone_name)

while True:
    print("Listening for key words...")
    key = get_key_press().lower()
    if key == 'c' or key == '\r' or key == '\n':
        print("Recording command...")
        sr_recognizer = sr.Recognizer()
        with sr.Microphone(device_index=args.microphone_index) as source:
            sr_recognizer.adjust_for_ambient_noise(source)
            audio = sr_recognizer.listen(source)
            try:
                # command = sr_recognizer.recognize_whisper_api(audio, api_key=os.environ['OPENAI_API_KEY'])
                command = sr_recognizer.recognize_google(audio)
                print(f"Recognized speech: {command}")
            except sr.RequestError as e:
                print("Could not request results from Whisper API")
            except Exception as e:
                print(f"Error recognizing speech: {e}")
    elif key == 'q':
        print("Quitting script...")
        break 