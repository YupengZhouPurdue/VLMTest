import base64
import openai
import time
import subprocess
from std_msgs.msg import String
from prompt_generation_interface.get_driving_context import get_driving_context

# ===============================
# FUNCTION: Send Data to GPT-4o (With Thought, Action, PID, MPC)
# ===============================
def query_gpt4o(query, image_data, refined_prompt, openai_api_key):
    openai.api_key = openai_api_key  # Set API key from argument
    headers = {
        "Authorization": f"Bearer {openai_api_key}",
        "Content-Type": "application/json"
    }

    payload = {
        "model": "gpt-4o",
        "messages": [
            {"role": "system", "content": refined_prompt},
            {"role": "user", "content": query}
        ],
        "max_tokens": 1000
    }

    if image_data:
        payload["messages"].append(
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": query},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_data}"}}
                ]
            }
        )

    start_reasoning_time = time.perf_counter()

    try:
        print("Sending request to GPT-4o... Please wait.")
        response = openai.ChatCompletion.create(**payload)
        
        reasoning_time = time.perf_counter() - start_reasoning_time
        
        action_response = response["choices"][0]["message"]["content"]

        # Extract Thought, Action, PID, and MPC values
        thought_output = "No Thought Generated"
        action_output = "Unknown Action"
        pid_values = "N/A"
        mpc_values = "N/A"

        if "Thought:" in action_response and "Action:" in action_response:
            lines = action_response.split("\n")
            for line in lines:
                if "Thought:" in line:
                    thought_output = line.split("Thought:")[-1].strip()
                elif "Action:" in line:
                    action_output = line.split("Action:")[-1].strip()
                elif "PID:" in line:
                    pid_values = line.split("PID:")[-1].strip()
                elif "MPC:" in line:
                    mpc_values = line.split("MPC:")[-1].strip()

        total_action_time = time.perf_counter() - start_reasoning_time
        return thought_output, action_output, pid_values, mpc_values, reasoning_time, total_action_time

    except Exception as e:
        print(f"Error querying GPT-4o: {e}")
        return "Error processing request.", "N/A", "N/A", "N/A", 0, 0

def run(action, pid_values, mpc_values):
    """
    Execute the corresponding shell scripts based on the action and control parameters.
    """
    script_dir = "/catkin_ws/Project/Talk2Drive/scripts"
    action_script_map = {
        "overtake": "overtake.sh",
        "lane change left": "lane_change_left.sh",
        # Add more actions and their corresponding scripts here
    }
    action_key = action.lower().strip()
    if action_key in action_script_map:
        subprocess.run([f"{script_dir}/{action_script_map[action_key]}"])
    else:
        print(f"No script mapped for action: {action}")

    # Execute longitudinal_speed.sh with PID values
    if pid_values != "N/A":
        pid_list = pid_values.replace('[', '').replace(']', '').split()
        subprocess.run([f"{script_dir}/longitudinal_speed.sh"] + pid_list)

    # Execute lateral_speed.sh with MPC values
    if mpc_values != "N/A":
        mpc_list = mpc_values.replace('[', '').replace(']', '').split()
        subprocess.run([f"{script_dir}/lateral_speed.sh"] + mpc_list)

# ===============================
# MAIN EXECUTION (Runs Once)
# ===============================
if __name__ == "__main__":
    # rospy.loginfo("VLMTestingDrive)")

    # You can still use your API key here for standalone runs
    OPENAI_API_KEY = "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"
    command = "" #Type your command
    driving_context, refined_prompt, prompt_time, image_data = get_driving_context(command)

    if image_data is None:
        print("Failed to load image. Exiting...")
        exit(1)

    start_total_time = time.perf_counter()

    thought_response, action_response, pid_values, mpc_values, reasoning_time, action_time = query_gpt4o(
        driving_context, image_data, refined_prompt, OPENAI_API_KEY
    )

    run(action_response, pid_values, mpc_values)

    total_time = time.perf_counter() - start_total_time

    print("\n=== GPT-4o Thought Process ===\n")
    print(thought_response)

    print("\n=== Selected Driving Action ===")
    print(f"\n**\n{action_response}\nPID {pid_values}\nMPC {mpc_values}\n**")

    # rospy.loginfo("Execution completed. Exiting...")
    exit(0)
