import sys
from STR400_SDK.str400 import STR400
import time
import ast



def main():
    print("This is realtime robot control, enter (e.g., forward, backward, up, down). Type 'stop' to quit.")
    robot = STR400(host='localhost', port=8080)

    # Activate the robot
    robot.enable()
    time.sleep(0.5)
    print("Robot has been successfully enabled.")
    move_to_default(robot)

    print("Entering Realtime control mode...")
    # robot.start_real_time_position_control()
    time.sleep(0.5)

    while True:
        user_input = input("Enter a command: ").lower()
        
        if user_input == "stop":
            print("Exiting the program. Goodbye!")
            robot.stop()
            robot.disable()
            print("Realtime control demonstration has concluded. Robot is now disabled.")
            break
        if user_input == "default":
            move_to_default(robot)
        else:
            parsed_list = ast.literal_eval(user_input)

            print("Moving robot to ", parsed_list)
            robot.movej(parsed_list)

def parse_command(command):
    # Split the command into axis and direction
    axis, direction = command.split()
    print('move robot on ', command)
    # Initialize the dictionary with None values
    values = {"x": None, "y": None, "z": None, "roll": None, "pitch": None, "yaw": None}

    # Set the value based on the direction
    if direction == "forward":
        values[axis] = True
    elif direction == "backward":
        values[axis] = False

    return values


def handle_command(robot, command):
    # Separate each command using a case statement
    commandValue = parse_command(command)

    robot.real_time_position_control(commandValue)
    time.sleep(2)

    print("Halting robot movements for 1 seconds...")
    values = {"x": None, "y": None, "z": None,
            "roll": None, "pitch": None, "yaw": None}
    robot.real_time_position_control(values)
    time.sleep(2)

def move_to_default(robot):
    print("Initiating MoveJ to default angles [0, -90, 0, 0, 0, 0] over a duration of 5 seconds...")
    angles = [0, 0, 90, 0, 90, 0, 6]
    robot.movej(angles)
    time.sleep(6.5)
    print("Moved to default.")


if __name__ == "__main__":
    main()

    # Example usage:
    # user_command = "z backward"
    # parsed_values = parse_command(user_command)
    # print(parsed_values)  # Output: {'x': True, 'y': None, 'z': None, 'roll': None, 'pitch': None, 'yaw': None}

    
#     {action: "SetTask", payload: {type: "RealTimePositionControlTask",…}}

# {"action":"RealTimePositionControl","payload":{"x":null,"y":true,"z":null,"roll":null,"pitch":null,"yaw":null}}