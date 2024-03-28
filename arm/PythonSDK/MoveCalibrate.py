from STR400_SDK.str400 import STR400
import time


class RobotArm:
    def __init__(self, arm) -> None:
        # Initialize the robot with specified host and port
        port = 8080
        if arm == 'left':
            port = 8081
        if arm == 'right':
            port = 8080

        self.robot = STR400(host='localhost', port=port)
        self.robot.enable()
        print("Robot successfully activated.")
        time.sleep(1)

    def Enable(self):
        self.robot.enable()

    def Disable(self):
        self.robot.disable()

    def ResetError(self):
        self.robot.reset()

    def MoveToAngle(self, angles):
        # Commanding the robot to move its joints to specified angles using MoveJ
        print("MoveJ to target angles")
        self.robot.movej(angles)
        # Brief pause to ensure the MoveJ command is processed
        time.sleep(0.5)
        # Monitoring task status and waiting for the completion of the MoveJ operation
        print("Monitoring task status until MoveJ operation is completed...")
        while True:
            task_status = self.robot.get_task_status()
            if task_status.get('type') is None:  # Check if the task has concluded
                print("MoveJ operation completed.")
                break
            time.sleep(0.2)

    def MoveC(self, angles):
        print("MoveJ to target position")
        self.robot.movec(angles)
        time.sleep(0.5)
        # Monitoring task status and waiting for the completion of the MoveJ operation
        print("Monitoring task status until MoveJ operation is completed...")
        while True:
            task_status = self.robot.get_task_status()
            if task_status.get('type') is None:  # Check if the task has concluded
                print("movec operation completed.")
                break
            time.sleep(0.2)

    def MoveToDefault(self):
        angles= [0, -90, 0, 0, 0, 0, 10]
        # Commanding the robot to move its joints to specified angles using MoveJ
        print("MoveJ to target angles")
        self.robot.movej(angles)
        # Brief pause to ensure the MoveJ command is processed
        time.sleep(0.5)
        # Monitoring task status and waiting for the completion of the MoveJ operation
        print("Monitoring task status until MoveJ operation is completed...")
        while True:
            task_status = self.robot.get_task_status()
            if task_status.get('type') is None:  # Check if the task has concluded
                print("MoveJ operation completed.")
                break
            time.sleep(0.2)



if __name__ == "__main__":
    robot = RobotArm('right')
    # robot.ResetError()
    robot.MoveToDefault()
    time.sleep(1)


    # Move J examples
    # this example in general works

    leftBottom = [85, -90, 0, -90, 90, 0, 5]
    robot.MoveToAngle(leftBottom)
    time.sleep(5)

    leftTop = [85, -110, 0, -90, 90, -30, 5]
    robot.MoveToAngle(leftTop)
    time.sleep(5)

    TopMiddle = [105, -115, 0, -90, 75, -30, 5]
    robot.MoveToAngle(TopMiddle)
    time.sleep(5)

    # # Move C examples
    # leftBottom = [-292, 146, 53, -95, 10, -161, 10]
    # robot.MoveC(leftBottom)
    # time.sleep(5)

    # # LeftLow = [-370, 173, 18, 172, -53, -175, 10]
    # # robot.MoveC(LeftLow)
    # # time.sleep(5)

    # # midBottom = [-378, 113, 95, 168, -36, 167, 10]
    # # robot.MoveC(midBottom)
    # # time.sleep(5)

    # # unreachable move c 82,-125,0,-89,87,-46,2

    # midBottom = [-267, 247, 27, -85, 4, -159, 10]
    # robot.MoveC(midBottom)
    # time.sleep(5)


    robot.MoveToDefault()
    robot.Disable()
