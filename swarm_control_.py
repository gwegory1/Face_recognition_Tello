import math
import time
import threading
import socket

# IP and port of Tello drones
tello_ip = "192.168.2.11"
tello_ip2 = "192.168.2.12"
# up for dabate
tello_ip3 = "192.168.2.14"

command_port = 8889

host_ip = "0.0.0.0"

response_port = 9000

end_goal = (100, 30)

print("\nUpdated Tello Command Program\n")

# Drone class
class drone:
    def __init__(self, ip):
        self.ip = ip
        self.coord = ([0, 0])
        self.yaw = 0
        self.goal = (0, 0)
    
    def get_pos(self):
        return self.coord
    
    def inc_pos(self, coordx, coordy):
        self.coord[0] = self.coord[0] + coordx
        self.coord[1] = self.coord[1] + coordy
    
    def get_yaw(self):
        return self.yaw
    
    def set_yaw(self, yaw):
        self.yaw = yaw
    
    def update_position(self, new_coord):
        self.coord = new_coord

    def set_goal(self, goal):
        self.goal = goal
    
    def get_goal(self):
        return self.goal
    
    def get_ip(self):
        return self.ip

# Tello
# Basic class for communication with Tello
class Tello:
    def __init__(self):
        self._running = True

        # 3 separate sockets for each drone, cuz it didnt work otherwise :/ 
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #self.sock.bind((host_ip, response_port))  # Bind for receiving
        self.sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


    def terminate(self):
        self._running = False
        self.sock.close()
        self.sock2.close()

    def recv(self):
        """ Handler for Tello response message """
        while self._running:
            try:
                msg, _ = self.sock.recvfrom(1024)  # Read 1024-bytes from UDP socket
                print("response: {}".format(msg.decode(encoding="utf-8")))
            except Exception as err:
                print(err)

    def send(self, msg, r_ip=None):
        """ Handler for send message to Tello """
        msg = msg.encode(encoding="utf-8")
        if r_ip is not None:
            if r_ip == tello_ip:
                self.sock.sendto(msg, (r_ip, command_port))
            if r_ip == tello_ip2:
                self.sock2.sendto(msg, (tello_ip2, command_port))
        else:
            self.sock.sendto(msg, (tello_ip, command_port))
            self.sock2.sendto(msg, (tello_ip2, command_port))
        print("message: {}".format(msg))  # Print message

# Tello controller
# Class for controlling multiple Tello drones
class Tello_controller:
    def start_drones(self):
        t.send("command", None)
        time.sleep(0.5)
        t.send("takeoff", None)
    
    def land_drones(self):
        t.send("land", None)

    # Calculate positions for drones:
    # Calculates positions for drones on a circle's
    # circumference with the given radius and goal
    def calculate_positions(self, num_drones, radius, goal):
        positions = []
        for i in range(num_drones):
            angle = 2 * math.pi * i / num_drones
            x = goal[0] + radius * math.cos(angle)
            y = goal[1] + radius * math.sin(angle)
            positions.append((x, y))
        return positions
    
    def distance_to_goal(self, drone, goal):
        current_pos = drone.get_pos()
        return math.sqrt((goal[0] - current_pos[0]) ** 2 + (goal[1] - current_pos[1]) ** 2)

    # Control a single drone
    # Checks if drone is faceing the right direction:
    # If not, rotates the drone to face the goal
    # If yes, moves the drone forward
    def control_drone(self, drone, goal, tolerance):
        current_pos = drone.get_pos()
        distance_to_goal = math.sqrt((goal[0] - current_pos[0]) ** 2 + (goal[1] - current_pos[1]) ** 2)
        if distance_to_goal <= 20:
            return True  # Reached the goal

        current_yaw = drone.get_yaw()
        target_yaw = math.atan2(goal[1] - current_pos[1], goal[0] - current_pos[0])
        angle_diff = target_yaw - current_yaw

        print("Drone current yaw:" + str(current_yaw))
        print ("Drone current angle diff:" + str(angle_diff))
        
        if abs(int(math.degrees(angle_diff))) != 0 :
            if angle_diff > 0:
                message = "ccw " + str(int(math.degrees(abs(angle_diff))))
                t.send(message, drone.get_ip())
                drone.set_yaw(current_yaw + angle_diff)
            if angle_diff < 0:
                message = "cw " + str(int(math.degrees(abs(angle_diff))))
                t.send(message, drone.get_ip())
                drone.set_yaw(current_yaw + angle_diff)
        else:
            if int(distance_to_goal) > 100.0:
                message = "forward 50"
            else:
                message = "forward 20"  # fixed the string format issue
            t.send(message, drone.get_ip())
            print("moved 20 with drone: " + drone.get_ip())
            drone.inc_pos(10 * math.cos(current_yaw), 10 * math.sin(current_yaw))

        return False

    # Controls multiple drones to reach their goals
    def control_multiple_drones(self, num_drones, radius, tolerance):
        positions = self.calculate_positions(num_drones, radius, end_goal)
        drones = [drone(tello_ip), drone(tello_ip2)]
        
        while True:
            all_reached_goal = True
            for i in range(num_drones):
                goal_x = positions[i][0]
                goal_y = positions[i][1]
                reached_goal = self.control_drone(drones[i], (goal_x, goal_y), tolerance)
                if not reached_goal:
                    all_reached_goal = False
            if all_reached_goal:
                self.land_drones()
                
                break
            time.sleep(4)  
# Example usage
num_drones = 2  # corrected the number of drones
radius = 40  # Radius of the circle
tolerance = 10  # Tolerance to the goal

t = Tello()
recvThread = threading.Thread(target=t.recv)
recvThread.start()

tc = Tello_controller()

while True:
    try:
        # Get input from CLI
        msg = input()
        # Check for "bye"
        if msg == "bye":
            break  # Break out of the loop if "bye" is entered

        if msg == "go":
            tc.start_drones()
            time.sleep(3)
            tc.control_multiple_drones(num_drones, radius, tolerance)
            print('initializing drones...')

        if msg == "stop":
            tc.land_drones()
            print('landing drones...')
        t.send(msg)

    except KeyboardInterrupt:
        tc.land_drones()
        t.terminate()
        recvThread.join()
        break

t.terminate()
recvThread.join()
print("\nGood Bye\n")
