from utils import *
import cv2
import mediapipe as mp

drone3 = initializeTello(host='192.168.2.11',vsport=11112 )
#drone1 = initializeTello(host='192.168.2.11',vsport=11112)
#drone2 = initializeTello(hos='192.168.2.12', vsport=11114)
drones = [drone3], #, drone2, drone3
w = 360
h = 240   
takeoff_allowed = False
pid = [0.5, 0.5, 0]
errors = [0, 0, 0]

# Variables to track the roles
primary_drone = None  # Index of the primary drone
primary_found_times = [0,0,0]  # Timestamp when a drone started continuous detection

errors_drone1 = [0,0,0]

while True:

    if takeoff_allowed:
        #drone1.takeoff()
        #drone2.takeoff()
        drone3.takeoff()
        takeoff_allowed = False

    # Capture frames from each drone
    frames = [telloGetFrame(drone3, w, h)] #, drone2.get_frame_read().frame, drone3.get_frame_read().frame

    # Assign primary and secondary roles
    result = assign_primary([drone3], frames, primary_drone, primary_found_times) #drone2, drone3
    if len(result) == 3:
        primary_drone, secondaries, primary_found_times = result
    else:
        primary_drone, secondaries = result

    if primary_drone is not None:
        # Primary drone tracks the face with feedback
        primary_frame, info = findFaceMediaPipe(frames[primary_drone])
        #if primary_drone == 0:
        #    errors_drone1 = trackFace(drone1, info, w, h, pid, errors_drone1, target='center')
        #elif primary_drone == 1:
        #   errors_drone2 = trackFace(drone2, info, w, h, pid, errors_drone2, target='center')
        if primary_drone == 2:
           errors_drone3 = trackFace(drone3, info, w, h, pid, errors_drone3, target='right')

        # Secondary drones track with feedback to align left or right
        if len(secondaries) > 0:
            for idx, sec_drone in enumerate(secondaries):
                
                if len(secondaries) == 1:
                    position = 'left'  # Default to 'left' if there's only one secondary drone
                else:
                    position = 'left' if idx == 0 else 'right'
                    
                frame, info = findFaceMediaPipe(frames[sec_drone])

                #if sec_drone == 0:
                #    errors_drone1 = trackFace(drone1, info, w, h, pid, errors_drone1, target=position)
                #elif sec_drone == 1:
                #   errors_drone2 = trackFace(drone2, info, w, h, pid, errors_drone2, target=position)
                if sec_drone == 2:
                    errors_drone3 = trackFace(drone3, info, w, h, pid, errors_drone3, target=position)


    # Display primary drone's POV
    cv2.imshow("Primary Drone POV", frames[0])
    #cv2.imshow("Primary Drone POV", frames[1])
    #cv2.imshow("Primary Drone POV", frames[2])

    # Handle quit and land all drones
    if cv2.waitKey(1) & 0xFF == ord('q'):
        for drone in [drone3]:
            drone.land()
        cv2.destroyAllWindows()
        break
   

   
    