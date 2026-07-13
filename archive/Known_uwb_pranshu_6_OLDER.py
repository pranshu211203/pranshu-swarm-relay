import threading
import socket
import sys
from turtle import distance
from djitellopy import Tello # type: ignore
import time
import json
from typing import Dict, Set, Any, List, Literal
import math
import os
import re
from UWB_ReadUDP import get_target_position
from swarmserverclient import MarkerClient

script_name = os.path.basename(__file__)

match = re.search(r'Known_uwb_rohan_(\d+)', script_name)

if match:
    drone_id = int(match.group(1))  # Extract the number and convert it to an integer
    print(f"Extracted ID: {drone_id}")
else:
    print("Script name does not match the expected pattern.")
    drone_id = 0  # Default drone ID

# The order matters! First ID = Farthest Drone (Full Path). Last ID = Closest Drone (Shortest Path).
# Example: Drone 8 goes to 5,0. Drone 10 goes to 4,0.
RELAY_HIERARCHY = [6, 7, 9, 12, 13, 23, 24, 25]
status = ""
course = 0

LAND_ID = 0 # set to 0 for no land
FLYING_STATE = False
landed_cleanly = False   

RC_SPEED_SCALE = 0.3
SYNC_TIMEOUT_BASE = 45  # seconds to wait for drone ahead before proceeding anyway (crash failsafe)

# UWB correction safeguards (new golobal variables)
MAX_CORRECTION_CM = 150       # Cap any single correction to this magnitude
UWB_STALE_LIMIT = 3           # Skip correction after N identical consecutive reads (frozen tag)
CONFIRMATION_READS = 3        # Number of re-reads to confirm a large correction
CONFIRMATION_DELAY = 0.3      # Seconds between confirmation reads

last_uwb_pos = None           # Previous UWB reading for staleness detection
uwb_stale_count = 0           # How many times in a row we've seen the exact same reading

waypoints = [] # to store executed waypoints and drone's current position
stream_ready = threading.Event()

###########################################################################################################

def load_drone_info(filename='drones.json'): # GTG
    try:
        with open(filename, 'r') as file:
            return json.load(file)
    except FileNotFoundError:
        print(f"{filename} not found!")
        return []
    except json.JSONDecodeError:
        print(f"Error decoding JSON in {filename}!")
        return []

DRONE_INFO = load_drone_info()
for drone in DRONE_INFO:
    if drone['id'] == drone_id:
        host = drone["TELLO_IP"]
        control_port = drone["TELLO_PORT"]
        state_port = drone["LOCAL_PORT"]
        video_port = drone["VIDEO_PORT"]
        delay = drone["delay"]
        SSID = drone["TELLO_SSID"]
        match = re.search(r'RMTT-TAG(\d+)', SSID)
        if match:
            tag_id = int(match.group(1))  # Extract the number and convert it to an integer
            print(f"Tag ID: {tag_id}")
        else:
            print("SSID does not match the expected pattern.")
            tag_id = 99 


class CustomTello(Tello):
    def __init__(self):
        
        global host, control_port, state_port, video_port

        
        # Store custom configuration
        self.TELLO_IP = host
        self.CONTROL_UDP_PORT = control_port
        self.STATE_UDP_PORT = state_port
        self.VS_UDP_PORT = video_port
        self.RESPONSE_TIMEOUT = 12  # default is 7, give more headroom for 5 drones
        
        Tello.STATE_UDP_PORT = state_port
        Tello.CONTROL_UDP_PORT = control_port
        Tello.RESPONSE_TIMEOUT = 12
        
        # Call parent's init with our custom host
        super().__init__(host)
        
        # Override the connection parameters
        self.address = (self.TELLO_IP, self.CONTROL_UDP_PORT)
        
        # Override video port
        self.vs_udp_port = video_port  

###########################################################################################################
def load_master_relay(use_tcp=False, port=6000):
    """
    Load master_relay.json either via TCP or from local file.
    """
    if use_tcp:
        print("[INFO] Waiting for master_relay.json via TCP...")
        import socket
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', port))
        sock.listen(1)
        
        conn, addr = sock.accept()
        print(f"[OK] Connection from {addr[0]}:{addr[1]}")
        
        # Read length header
        length_data = b''
        while len(length_data) < 8:
            length_data += conn.recv(8 - len(length_data))
        expected = int(length_data.decode('utf-8'))
        
        # Read JSON payload
        received = b''
        while len(received) < expected:
            received += conn.recv(min(4096, expected - len(received)))
        
        conn.close()
        sock.close()
        
        data = json.loads(received.decode('utf-8'))
        
        # Save a local copy as backup
        with open('master_relay.json', 'w') as f:
            json.dump(data, f, indent=4)
        print(f"[OK] Received and saved master_relay.json ({len(data['wp'])} waypoints)")
        
    else:
        print("[INFO] Loading master_relay.json from local file...")
        with open('master_relay.json', 'r') as f:
            data = json.load(f)
        print(f"[OK] Loaded {len(data['wp'])} waypoints from file")
    
    return data

def get_relay_flight_plan(drone_id, override_rank=None):
    global relay_data
    import copy
    data = copy.deepcopy(relay_data)
    all_waypoints = list(data['wp'])

    # Use override rank (for backup drones) or derive from hierarchy
    if override_rank is not None:
        rank = override_rank
        print(f"[INFO] Drone {drone_id} using OVERRIDE rank {rank} (backup mission).")
    elif drone_id in RELAY_HIERARCHY1:
        rank = RELAY_HIERARCHY1.index(drone_id)
    else:
        print(f"[WARNING] Drone {drone_id} not in any hierarchy. Using full path.")
        return data

    total_points = len(all_waypoints)
    points_to_keep = total_points - rank

    if points_to_keep <= 0:
        print(f"[INFO] Drone {drone_id} is Rank {rank}. No waypoints assigned.")
        return {"wp": []}

    truncated_waypoints = all_waypoints[:points_to_keep]
    last_wp = truncated_waypoints[-1].copy()
    original_dist = last_wp['dist_cm']
    last_wp['dist_cm'] = 0
    truncated_waypoints[-1] = last_wp

    print(f"[INFO] Drone {drone_id} is Rank {rank}. Keeping {points_to_keep}/{total_points} waypoints.")
    print(f"[INFO] Modified Final Waypoint: Changed dist_cm from {original_dist} to 0.")

    data['wp'] = truncated_waypoints
    return data

def ascend(drone, altitude):
    global height, status

    # FIX: Get the current height from the drone if the global variable isn't set yet
    try:
        current_h = drone.get_height()
    except:
        current_h = 90
    
    # Use current_h instead of the uninitialized global 'height'
    if abs(current_h - altitude) > 10:
        drone.go_xyz_speed(0, 0, int(altitude - current_h), 100)
        print(f"Ascending to height {altitude}")
        status = f"Ascending to height {altitude}"
    else:
        print(f"Already at {current_h}")
        status = f"Already at {current_h}"

# ADD 1
def stream_video(drone):
    global heading, pos, ang, height, marker_IDs, marker_list, status, dis, id, sys, course

    while True:
        ret = True
        height = drone.get_height()
        battery = drone.get_battery()
        heading = drone.get_yaw() - start_heading  
    #cap.release()
    cv2.destroyAllWindows()
    sys.exit()

###########################################################################################################

def uwb_reading(drone):
    uwb_raw = (0,0,0)
    retry_count = 0
    while uwb_raw == (0,0,0) and retry_count < 10:
        uwb_raw = get_target_position(tag_id)
        retry_count += 1
    if uwb_raw == (0,0,0):
        return [0,0]
    uwb_pos = [uwb_raw[0]*100, uwb_raw[1]*100]    
    return uwb_pos

def uwb_averaged_reading(samples=6, interval=0.5):
    """Take multiple UWB readings over a period and return the average position.
    Default: 6 samples over 3 seconds."""
    readings = []
    for i in range(samples):
        uwb_raw = (0, 0, 0)
        retry = 0
        while uwb_raw == (0, 0, 0) and retry < 10:
            uwb_raw = get_target_position(tag_id)
            retry += 1
        if uwb_raw != (0, 0, 0):
            readings.append([uwb_raw[0] * 100, uwb_raw[1] * 100, uwb_raw[2] * 100])
        if i < samples - 1:
            time.sleep(interval)

    if len(readings) < 2:
        print(f"[UWB AVG] Only {len(readings)} valid reads. Not enough for averaging.")
        return None

    avg_x = sum(r[0] for r in readings) / len(readings)
    avg_y = sum(r[1] for r in readings) / len(readings)
    avg_z = sum(r[2] for r in readings) / len(readings)
    print(f"[UWB AVG] {len(readings)} samples → avg=({avg_x:.0f}, {avg_y:.0f}, {avg_z:.0f})")
    return [avg_x, avg_y, avg_z]

def uwb_correction(drone):
    global pos, heading, last_uwb_pos, uwb_stale_count

    # --- STEP 1: Get UWB reading ---
    uwb_raw = (0, 0, 0)
    retry_count = 0
    while uwb_raw == (0, 0, 0) and retry_count < 15:
        uwb_raw = get_target_position(tag_id)
        retry_count += 1
    if uwb_raw == (0, 0, 0):
        print("[UWB] No data after 15 retries. Skipping correction.")
        return "No UWB data"

    uwb_pos = [uwb_raw[0] * 100, uwb_raw[1] * 100]
    uwb_height = uwb_raw[2] * 100
    height = uwb_height - uwb_ground_height
    print(f"Height: {height}")

    # --- STEP 2: Staleness detection (frozen tag) ---
    if last_uwb_pos is not None and uwb_pos[0] == last_uwb_pos[0] and uwb_pos[1] == last_uwb_pos[1]:
        uwb_stale_count += 1
        if uwb_stale_count >= UWB_STALE_LIMIT:
            print(f"[UWB] STALE: {uwb_stale_count} identical reads ({uwb_pos}). Tag may be frozen. Skipping correction.")
            return "Stale UWB"
    else:
        uwb_stale_count = 0
    last_uwb_pos = uwb_pos[:]

    # --- STEP 3: Compute correction ---
    x_diff = pos[0] - uwb_pos[0]
    y_diff = pos[1] - uwb_pos[1]
    x_corr = x_diff * math.cos(math.radians(heading)) - y_diff * math.sin(math.radians(heading))
    y_corr = y_diff * math.cos(math.radians(heading)) + x_diff * math.sin(math.radians(heading))

    correction_magnitude = math.sqrt(x_corr**2 + y_corr**2)
    print(f"Dead Reckoning Position: {pos}, UWB Position: {uwb_pos}")
    print(f"Deviation X: {x_corr:.1f}, Y: {y_corr:.1f}, Magnitude: {correction_magnitude:.1f}cm")

    # --- STEP 4: Large correction confirmation ---
    if correction_magnitude > MAX_CORRECTION_CM:
        print(f"[UWB] Large correction ({correction_magnitude:.0f}cm > {MAX_CORRECTION_CM}cm). Confirming with {CONFIRMATION_READS} re-reads...")

        confirm_count = 0
        for attempt in range(CONFIRMATION_READS):
            time.sleep(CONFIRMATION_DELAY)
            re_raw = (0, 0, 0)
            rc = 0
            while re_raw == (0, 0, 0) and rc < 10:
                re_raw = get_target_position(tag_id)
                rc += 1
            if re_raw == (0, 0, 0):
                print(f"  Re-read {attempt+1}: No data. Skipping.")
                continue

            re_pos = [re_raw[0] * 100, re_raw[1] * 100]
            re_x_diff = pos[0] - re_pos[0]
            re_y_diff = pos[1] - re_pos[1]
            re_x_corr = re_x_diff * math.cos(math.radians(heading)) - re_y_diff * math.sin(math.radians(heading))
            re_y_corr = re_y_diff * math.cos(math.radians(heading)) + re_x_diff * math.sin(math.radians(heading))
            re_mag = math.sqrt(re_x_corr**2 + re_y_corr**2)

            # Check same direction: dot product of original correction vector and re-read vector
            dot = x_corr * re_x_corr + y_corr * re_y_corr
            if re_mag > MAX_CORRECTION_CM and dot > 0:
                confirm_count += 1
                print(f"  Re-read {attempt+1}: {re_mag:.0f}cm, same direction. Confirmed ({confirm_count}/{CONFIRMATION_READS})")
            else:
                print(f"  Re-read {attempt+1}: {re_mag:.0f}cm, inconsistent. Likely glitch.")

        if confirm_count < CONFIRMATION_READS:
            print(f"[UWB] Only {confirm_count}/{CONFIRMATION_READS} confirmations. Glitch detected. Skipping correction.")
            return "Glitch rejected"

        # All confirmations passed — drone really is off-position. Apply capped correction.
        scale = MAX_CORRECTION_CM / correction_magnitude
        x_corr = x_corr * scale
        y_corr = y_corr * scale
        print(f"[UWB] Confirmed large drift. Applying capped correction: X={x_corr:.1f}, Y={y_corr:.1f}")

    # --- STEP 5: Apply correction ---
    if abs(int(x_corr)) >= 30 or abs(int(y_corr)) >= 30:
        try:
            drone.go_xyz_speed(int(y_corr), int(-x_corr), 0, 100)
            time.sleep(1.5)
        except:
            pass

def uwb_correction_precise(drone):
    """High-accuracy UWB correction using averaged readings. Use before landing."""
    global pos, heading

    avg = uwb_averaged_reading(samples=6, interval=0.5)
    if avg is None:
        return "No averaged UWB data"

    uwb_pos = [avg[0], avg[1]]
    height = avg[2] - uwb_ground_height
    print(f"[PRECISE] Height: {height}")

    x_diff = pos[0] - uwb_pos[0]
    y_diff = pos[1] - uwb_pos[1]
    x_corr = x_diff * math.cos(math.radians(heading)) - y_diff * math.sin(math.radians(heading))
    y_corr = y_diff * math.cos(math.radians(heading)) + x_diff * math.sin(math.radians(heading))

    magnitude = math.sqrt(x_corr**2 + y_corr**2)
    print(f"[PRECISE] Deviation X: {x_corr:.1f}, Y: {y_corr:.1f}, Magnitude: {magnitude:.1f}cm")

    if magnitude > MAX_CORRECTION_CM:
        scale = MAX_CORRECTION_CM / magnitude
        x_corr *= scale
        y_corr *= scale
        print(f"[PRECISE] Capped to {MAX_CORRECTION_CM}cm")

    if abs(int(x_corr)) >= 20 or abs(int(y_corr)) >= 20:  # Lower threshold for precision
        try:
            drone.go_xyz_speed(int(y_corr), int(-x_corr), 0, 60)  # Slower speed for accuracy
            time.sleep(2)
        except:
            pass

def validate_waypoints():
    global start_wpt

    # Spare drones don't know their plan yet — skip validation
    if drone_id not in RELAY_HIERARCHY1:
        print(f"[INFO] Drone {drone_id} is a spare. Skipping validation.")
        start_wpt = [0, 0]
        return True
    
    # 1. LOAD PLAN DYNAMICALLY (Replaces the if/elif group logic)
    data = get_relay_flight_plan(drone_id)

    # 2. HANDLE EMPTY/SHORT PATHS
    # If a drone is meant to stay at 0,0 (Rank is high), it might have 0 waypoints.
    if not data or not data.get('wp'):
        print(f"[INFO] Drone {drone_id} has no waypoints assigned. Validation skipped (Safe).")
        # We need a default start_wpt to prevent errors later, assume 0,0 or current UWB pos
        start_wpt = [0, 0] 
        return True

    # 3. SET START POINT
    # Extracted from the first waypoint in the sliced list
    start_wpt = [data['wp'][0]['position_cm']['x'], data['wp'][0]['position_cm']['y']]
    print(f"Starting waypoint is {start_wpt}")
    
    # 4. RUN VALIDATION CHECKS (Keep your existing logic)
    valid = True
    for i, wp in enumerate(data['wp']): 
        # Check distance
        if wp['dist_cm'] < 20 and not i+1 == len(data['wp']):
            print(f"[WARNING] Waypoint {i+1} distance ({wp['dist_cm']}cm) is below minimum 20cm")
            valid = False
        if wp['dist_cm'] > 500:
            print(f"[INFO] Waypoint {i+1} distance ({wp['dist_cm']}cm) will be split into multiple commands")
        
        # Check angle
        if abs(wp['angle_deg']) > 360:
            print(f"[WARNING] Waypoint {i+1} angle ({wp['angle_deg']}°) exceeds 360 degrees")
            valid = False
    
        # Check for dist_cm = 0 condition (only valid if it is last waypoint)
        if wp['dist_cm'] == 0 and i+1 == len(data['wp']):
            print(f"[INFO] Final Waypoint {i+1} distance is zero. End of routine.")
        elif wp['dist_cm'] == 0:
            print(f"[WARNING] Waypoint {i+1} distance is zero but is not final. Invalid.") 
            valid = False

    return valid

def execute_waypoints(drone):   # edit this up for all heading instances and waypoint updates
    global LAND_ID, FLYING_STATE, waypoints, status, course, pos, waypoint_id, landed_cleanly

    data = get_relay_flight_plan(drone_id)
    # Extracted from the first waypoint in the sliced list
    start_wpt = [data['wp'][0]['position_cm']['x'], data['wp'][0]['position_cm']['y']]
    print(f"Starting waypoint is {start_wpt}")
    
    waypoint_id = 0 # Waypoint 0 is the starting waypoint

    # --- NEW: RANK-BASED STARTUP STAGGER ---
    # Each drone waits for the drone ahead to have DEPARTED from WP0 before claiming it.
    # This prevents the initial race where all drones claim WP0 simultaneously.
    my_rank = RELAY_HIERARCHY1.index(drone_id)
    SYNC_TIMEOUT = SYNC_TIMEOUT_BASE + (my_rank * 15)  # Rank 0: 45s, ........
    print(f"[SYNC] Drone {drone_id} rank {my_rank}, sync timeout = {SYNC_TIMEOUT}s")
    sync_start = time.time()  # Initialize for all ranks (used in subsequent waits too)

    if my_rank > 0:
        print(f"[SYNC] Drone {drone_id} is rank {my_rank}. Waiting for drone {RELAY_HIERARCHY1[my_rank-1]} to LEAVE WP0...")
        sync_start = time.time()
        while not marker_client.can_proceed_to_waypoint(0, RELAY_HIERARCHY1):
            drone.send_rc_control(0, 0, 0, 0)
            # Send a status heartbeat to the server — this triggers the server to re-broadcast drone_progress
            marker_client.send_update('status', status_message=f"Waiting for rank {my_rank-1} to leave WP0")
            time.sleep(1)
            if time.time() - sync_start > SYNC_TIMEOUT:
                print(f"[SYNC WARNING] Timeout ({SYNC_TIMEOUT}s)! Drone ahead may have crashed. Proceeding anyway.")
                break
        print(f"[SYNC] Drone {drone_id}: drone ahead has left WP0. Proceeding.")

    while marker_client.is_waypoint_available(waypoint_id) is False:
        print(f"waiting for waypoint {waypoint_id} to be available")
        drone.send_rc_control(0, 0, 0, 0)
        marker_client.send_update('status', status_message=f"Waiting for WP{waypoint_id} to be free")
        time.sleep(1)
        if time.time() - sync_start > SYNC_TIMEOUT:
            print(f"[SYNC WARNING] WP{waypoint_id} still occupied after timeout. Proceeding anyway.")
            break
    
    # IMMEDIATELY CLAIM THE STARTING WAYPOINT
    marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)
    
    print(int(start_wpt[1]-sy),int(sx-start_wpt[0]))

    # Navigate to starting position — chunked to max 200cm per command
    dx = int(start_wpt[1] - sy)   # go_xyz Y component
    dy = int(sx - start_wpt[0])   # go_xyz X component
    total_dist = math.sqrt(dx**2 + dy**2)

    if total_dist > 20:
        CHUNK = 200
        if total_dist > CHUNK:
            num_chunks = math.ceil(total_dist / CHUNK)
            chunk_dx = int(dx / num_chunks)
            chunk_dy = int(dy / num_chunks)
            print(f"Start position {total_dist:.0f}cm away. Splitting into {num_chunks} chunks.")

            for c in range(num_chunks):
                drone.send_command_without_return(f"go {chunk_dx} {chunk_dy} 0 100")
                time.sleep(max(3, int(math.sqrt(chunk_dx**2 + chunk_dy**2) / 100)))
                pos[0] += (start_wpt[0] - sx) / num_chunks
                pos[1] += (start_wpt[1] - sy) / num_chunks
                time.sleep(0.5)
                uwb_correction(drone)
        else:
            drone.send_command_without_return(f"go {dx} {dy} 0 100")
            time.sleep(max(3, int(total_dist / 100)))
            pos[0] += start_wpt[0] - sx
            pos[1] += start_wpt[1] - sy

        print(f"Starting position: {pos}")
    else:
        print("Already at starting waypoint")
    
    time.sleep(2)
    uwb_correction(drone)
    # Re-assert claim on the start point after correcting (UDP safety)
    marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)
    
    # NOTE: Do NOT send_progress(0) here! We are still AT WP0.
    # Progress is only announced when we DEPART from a waypoint.
    
    # Initialize flight plan using the "Hierarchy/Rank" logic
    data = get_relay_flight_plan(drone_id) 
    abs_position = {"x": start_wpt[0], "y": start_wpt[1]}
    orientation = 180

    try:
        current_wp_id = 0 # We are starting at the start point (0)
        
        # enumerate gives us 'i' (0, 1, 2...) alongside the waypoint data
        for i, wp in enumerate(data['wp']):
            target_wp_id = i + 1
            released_previous = False

            # --- FINAL WAYPOINT: Skip sync wait entirely ---
            # dist_cm=0 means "land here" — we're not flying to target_wp_id,
            # so there's no reason to wait for the drone ahead to clear it.
            if wp['dist_cm'] == 0:
                print(f"[INFO] WP {target_wp_id} has dist_cm=0 (final waypoint). Skipping sync wait and movement.")
                marker_client.send_update('waypoint', marker_id=current_wp_id, detected=False)
                marker_client.send_progress(current_wp_id)
                print(f"[SYNC] Drone {drone_id} departed WP {current_wp_id}, follower unblocked.")
                current_wp_id = target_wp_id
                break  # Exit loop, fall through to landing

            # --- TRAFFIC CONTROL: SEQUENTIAL ORDERING + CLAIM ---
            # STEP A: Wait for the drone ahead of us to have DEPARTED from this waypoint
            #         This ensures we never fly TO a waypoint the leader is still sitting at.
            #         Drones follow in order: 22 → 23 → 24 → 25
            print(f"[SYNC] Waiting for drone ahead to LEAVE WP {target_wp_id}...")
            sync_start = time.time()
            while not marker_client.can_proceed_to_waypoint(target_wp_id, RELAY_HIERARCHY1):
                drone.send_rc_control(0, 0, 0, 0)
                time.sleep(1)
                # Re-assert our current waypoint claim so it doesn't timeout
                marker_client.send_update('waypoint', marker_id=current_wp_id, detected=True)
                if time.time() - sync_start > SYNC_TIMEOUT:
                    print(f"[SYNC WARNING] Timeout ({SYNC_TIMEOUT}s) waiting for drone ahead at WP {target_wp_id}! Proceeding anyway.")
                    break
            
            # STEP B: Also check the waypoint isn't physically occupied (belt + suspenders)
            sync_start = time.time()
            while marker_client.is_waypoint_available(target_wp_id) is False:
                drone.send_rc_control(0, 0, 0, 0)
                time.sleep(1)
                marker_client.send_update('waypoint', marker_id=current_wp_id, detected=True)
                if time.time() - sync_start > SYNC_TIMEOUT:
                    print(f"[SYNC WARNING] Timeout ({SYNC_TIMEOUT}s) waiting for WP {target_wp_id} occupancy! Proceeding anyway.")
                    break
            
            # STEP C: Claim the destination waypoint
            marker_client.send_update('waypoint', marker_id=target_wp_id, detected=True)
            print(f"[SYNC] Drone {drone_id} claimed WP {target_wp_id}, moving now.")
            # --------------------------------------------------------

            # --- STEP 1: ROTATION ---
            status = "Orienting"
            if wp['angle_deg'] != 0:
                orientation += wp['angle_deg']
                orientation %= 360
                if orientation > 180:
                    orientation -= 360
                
                course = int(-orientation) + 180
                if course > 180:
                    course -= 360 

                time.sleep(1)

                rotation_attempts = 0
                while abs(course - heading) > 3 and rotation_attempts < 10:
                    diff = course - heading
                    if abs(int(diff if -180 < diff < 180 else (diff - 360 if diff > 180 else diff + 360))) < 3:
                        break  # Too small for Tello to execute
                                        
                    if -180 < diff < 180:
                        cmd_val = int(diff)
                    elif diff > 180:
                        cmd_val = int(diff) - 360
                    else:
                        cmd_val = int(diff) + 360
                    if abs(cmd_val) < 1:  # Tello rejects cw 0
                        break

                    drone.send_command_without_return(f"cw {cmd_val}")
                    time.sleep(2)
                    rotation_attempts += 1
                
                update_position(waypoints, abs_position, orientation)
                uwb_correction(drone) 

            # Ensure facing forward before moving (with max retries to prevent infinite loop)
            rotation_attempts = 0
            while abs(course - heading) > 5 and rotation_attempts < 5:
                diff = course - heading
                if abs(int(diff if -180 < diff < 180 else (diff - 360 if diff > 180 else diff + 360))) < 3:
                    break  # Too small for Tello to execute meaningfully
                
                if -180 < diff < 180:
                    cmd_val = int(diff)
                elif diff > 180:
                    cmd_val = int(diff) - 360
                else:
                    cmd_val = int(diff) + 360
                if abs(cmd_val) < 1:  # Tello rejects cw 0
                    break

                drone.send_command_without_return(f"cw {cmd_val}")
                time.sleep(2)
                rotation_attempts += 1

            # --- STEP 2: MOVEMENT (Chunked for Safety) ---
            distance = wp['dist_cm']
            status = "Proceeding forward"

            if distance > 250:
                while distance > 200:
                    try:
                        drone.send_command_without_return(f"forward 200")
                        time.sleep(3)  # ~2s for 200cm at default speed + buffer
                    except Exception as e:
                        print(f"[WARNING] 200cm move dropped. Relying on UWB correction. Error: {e}")

                    if not released_previous:
                        marker_client.send_update('waypoint', marker_id=current_wp_id, detected=False)
                        marker_client.send_progress(current_wp_id)
                        print(f"[SYNC] Drone {drone_id} departed WP {current_wp_id}, follower unblocked.")
                        released_previous = True

                    update_position(waypoints, abs_position, orientation, 200)
                    time.sleep(1.5)
                    uwb_correction(drone)

                    distance -= 200
                    time.sleep(0.5)

            if distance > 20:
                try:
                    drone.send_command_without_return(f"forward {distance}")
                    time.sleep(max(2, int(distance / 100) + 1))
                except Exception as e:
                    print(f"[WARNING] {distance}cm move dropped. Relying on UWB correction. Error: {e}")

                if not released_previous:
                    marker_client.send_update('waypoint', marker_id=current_wp_id, detected=False)
                    marker_client.send_progress(current_wp_id)
                    print(f"[SYNC] Drone {drone_id} departed WP {current_wp_id}, follower unblocked.")
                    released_previous = True

                update_position(waypoints, abs_position, orientation, distance)
                time.sleep(1.5)
                uwb_correction(drone)

            # Safety fallback: if somehow we didn't release yet
            if not released_previous:
                marker_client.send_update('waypoint', marker_id=current_wp_id, detected=False)
                marker_client.send_progress(current_wp_id)
                print(f"[SYNC] Drone {drone_id} departed WP {current_wp_id}, follower unblocked.")

            # Update our tracker for the next loop
            current_wp_id = target_wp_id

        # --- STEP 3: MISSION END (LANDING) ---

        # Extra UWB corrections for landing accuracy
        bat = drone.get_battery()
        if bat > 15:
            print("[LANDING] Running final UWB corrections for accuracy...")
            uwb_correction_precise(drone)
            time.sleep(1)
            uwb_correction_precise(drone)
        else:
            print(f"[LANDING] Battery low ({bat}%). Quick correction only.")
            uwb_correction(drone)

        # The loop is done. We have reached the final coordinate in the JSON.
        print("Final waypoint reached. Initiating landing.")
        status = "Landing at Relay Point"
        try:
            drone.land()
        except Exception as e:
            if "Auto land" in str(e) or "error" in str(e):
                print(f"[LANDING] Land command failed ({e}), but drone is likely already landing.")
            else:
                raise

        # Verify landing position against target
        time.sleep(3)
        final_pos = uwb_reading(drone)
        target_x = data['wp'][-1]['position_cm']['x']
        target_y = data['wp'][-1]['position_cm']['y']

        if final_pos != [0, 0]:
            landing_error = math.sqrt((final_pos[0] - target_x)**2 + (final_pos[1] - target_y)**2)
            print(f"[LANDING] Target: ({target_x}, {target_y}), Actual: ({final_pos[0]:.0f}, {final_pos[1]:.0f}), Error: {landing_error:.0f}cm")

            if landing_error > 20:
                print(f"[LANDING] Landed too far from target ({landing_error:.0f}cm). Marking as failed.")
                marker_client.send_update('status', status_message="Landed off target")
                marker_client.send_progress(900)
                landed_cleanly = False
            else:
                print(f"[LANDING] Landing accuracy OK ({landing_error:.0f}cm).")
                marker_client.send_update('status', status_message="Landed at Relay")
                marker_client.send_progress(999)
                landed_cleanly = True
        else:
            print("[LANDING] No UWB data to verify. Assuming success.")
            marker_client.send_update('status', status_message="Landed at Relay")
            marker_client.send_progress(999)
            landed_cleanly = True

    except Exception as e:
        print(f"Error occurred: {e}")

    finally:
        if not landed_cleanly:
            marker_client.send_progress(900)
        if drone.is_flying:
            drone.land()

def execute_backup_waypoints(drone, assigned_rank):
    """Simplified execute_waypoints for backup drones — no sync waits needed."""
    global status, course, pos

    data = get_relay_flight_plan(drone_id, override_rank=assigned_rank)

    if not data or not data.get('wp'):
        print(f"[BACKUP] No waypoints for rank {assigned_rank}. Landing.")
        drone.land()
        return

    start_wpt = [data['wp'][0]['position_cm']['x'], data['wp'][0]['position_cm']['y']]
    print(f"[BACKUP] Target start waypoint: {start_wpt}")

    # Navigate to starting position — chunked to max 200cm per command
    dx = int(start_wpt[1] - sy)
    dy = int(sx - start_wpt[0])
    total_dist = math.sqrt(dx**2 + dy**2)

    if total_dist > 20:
        CHUNK = 200
        if total_dist > CHUNK:
            num_chunks = math.ceil(total_dist / CHUNK)
            chunk_dx = int(dx / num_chunks)
            chunk_dy = int(dy / num_chunks)
            print(f"[BACKUP] Start position {total_dist:.0f}cm away. Splitting into {num_chunks} chunks.")

            for c in range(num_chunks):
                drone.send_command_without_return(f"go {chunk_dx} {chunk_dy} 0 100")
                time.sleep(max(3, int(math.sqrt(chunk_dx**2 + chunk_dy**2) / 100)))
                pos[0] += (start_wpt[0] - sx) / num_chunks
                pos[1] += (start_wpt[1] - sy) / num_chunks
                time.sleep(0.5)
                uwb_correction(drone)
        else:
            drone.send_command_without_return(f"go {dx} {dy} 0 100")
            time.sleep(max(3, int(total_dist / 100)))
            pos[0] += start_wpt[0] - sx
            pos[1] += start_wpt[1] - sy

        print(f"[BACKUP] Starting position: {pos}")
    else:
        print("[BACKUP] Already at starting waypoint")

    time.sleep(2)
    uwb_correction(drone)

    abs_position = {"x": start_wpt[0], "y": start_wpt[1]}
    orientation = 180

    try:
        for i, wp in enumerate(data['wp']):
            if wp['dist_cm'] == 0:
                print(f"[BACKUP] Final waypoint reached. Landing.")
                break

            # --- ROTATION (same as primary) ---
            if wp['angle_deg'] != 0:
                orientation += wp['angle_deg']
                orientation %= 360
                if orientation > 180:
                    orientation -= 360
                course = int(-orientation) + 180
                if course > 180:
                    course -= 360

                rotation_attempts = 0
                while abs(course - heading) > 5 and rotation_attempts < 5:
                    diff = course - heading
                    if abs(int(diff if -180 < diff < 180 else (diff - 360 if diff > 180 else diff + 360))) < 3:
                        break
                    if -180 < diff < 180:
                        cmd_val = int(diff)
                    elif diff > 180:
                        cmd_val = int(diff) - 360
                    else:
                        cmd_val = int(diff) + 360
                    if abs(cmd_val) < 1:
                        break
                    drone.send_command_without_return(f"cw {cmd_val}")
                    time.sleep(2)
                    rotation_attempts += 1

            # --- MOVEMENT (same chunked logic, no sync waits) ---
            distance = wp['dist_cm']
            if distance > 250:
                while distance > 200:
                    try:
                        drone.send_command_without_return(f"forward 200")
                        time.sleep(3)  # ~2s for 200cm at default speed + buffer
                    except Exception as e:
                        print(f"[BACKUP WARNING] 200cm move error: {e}")
                    update_position(waypoints, abs_position, orientation, 200)
                    time.sleep(1.5)
                    uwb_correction(drone)
                    distance -= 200
                    time.sleep(0.5)

            if distance > 20:
                try:
                    drone.send_command_without_return(f"forward {distance}")
                    time.sleep(max(2, int(distance / 100) + 1))
                except Exception as e:
                    print(f"[BACKUP WARNING] {distance}cm move error: {e}")
                update_position(waypoints, abs_position, orientation, distance)
                time.sleep(1.5)
                uwb_correction(drone)
        
        bat = drone.get_battery()
        if bat > 15:
            print("[BACKUP] Running final UWB corrections for accuracy...")
            uwb_correction_precise(drone)
            time.sleep(1)
            uwb_correction_precise(drone)
        else:
            print(f"[BACKUP] Battery low ({bat}%). Quick correction only.")
            uwb_correction(drone)

        # Land
        print("[BACKUP] Landing at relay point.")
        drone.land()
        marker_client.send_update('status', status_message="Backup Landed at Relay")

    except Exception as e:
        print(f"[BACKUP] Error: {e}")
        if drone.is_flying:
            drone.land()

def update_position(waypoints, position, orientation, distance=0):
    global pos

    rad = math.radians(orientation)
    pos[1] -= int(distance * math.cos(rad))
    pos[0] += int(distance * math.sin(rad))
    position["x"] += int(distance * math.cos(rad))
    position["y"] += int(distance * math.sin(rad))
    waypoints.append({"x": position["x"], "y": position["y"], "orientation": orientation, "distance": distance})

###########################################################################################################

def flight_routine(drone):
    global landed_cleanly

    try:
        ascend(drone,100)
        if abs(heading) > 1:  # Tello rejects 0-degree rotations
            drone.rotate_counter_clockwise(heading)
        time.sleep(2)
        
        execute_waypoints(drone)
    except Exception as e:
        print(f"[CRITICAL] flight_routine crashed: {e}")
    finally:
        # If we crash anywhere in flight_routine (ascend, rotate, etc.),
        # still unblock all followers so they don't wait forever
        try:
            if not landed_cleanly:
                marker_client.send_progress(900)  # 900 = crashed / abnormal exit
                print(f"[SYNC] Drone {drone_id} flight_routine ended. Sent Progress 900")
            else:
                print(f"[SYNC] Drone {drone_id} flight_routine ended. Sent Progress 999")
        except:
            pass

def spare_drone_routine(drone):
    """Hover while primary relay runs. If any primary drone fails, fill its position."""
    global marker_client

    print(f"[SPARE] Drone {drone_id} is a spare. Hovering while primary relay runs...")

    # --- PHASE 1: Wait for ALL primary drones to finish ---
    # "Finished" = they've all sent progress(999), meaning their flight_routine ended
    SPARE_TIMEOUT = 180  # 3 minutes max hover (tune based on battery)
    start_time = time.time()

    while time.time() - start_time < SPARE_TIMEOUT:
        drone.send_rc_control(0, 0, 0, 0)  # Keepalive
        time.sleep(2)

        # Check if all primary drones have completed (progress 999)
        all_done = True
        for did in RELAY_HIERARCHY1:
            progress = marker_client.drone_progress.get(str(did), -1)
            if progress < 900:  # Not done yet (900 = crashed, 999 = landed successfully)
                all_done = False
                break

        if all_done:
            print(f"[SPARE] All primary drones finished. Checking for failures...")
            break

        # Print status every 10s
        elapsed = int(time.time() - start_time)
        if elapsed % 10 == 0:
            bat = drone.get_battery()
            print(f"[SPARE] Drone {drone_id} hovering... {elapsed}s, battery={bat}%")
            marker_client.send_update('status', status_message=f"Spare hovering {elapsed}s bat={bat}%")

            # Bail if battery is too low to be useful
            if bat < 25:
                print(f"[SPARE] Battery too low ({bat}%). Landing.")
                drone.land()
                return

    # --- PHASE 2: Detect which primary drones failed ---
    # Give a few seconds for final status messages to propagate
    time.sleep(3)

    failed_ranks = []
    for rank, did in enumerate(RELAY_HIERARCHY1):
        progress = marker_client.drone_progress.get(str(did), -1)

        if progress < 900:
            failed_ranks.append(rank)
            print(f"[SPARE] Drone {did} (rank {rank}) FAILED — never completed (progress={progress})")
        elif progress == 900:
            failed_ranks.append(rank)
            print(f"[SPARE] Drone {did} (rank {rank}) FAILED — crashed (progress=900)")

    if not failed_ranks:
        print(f"[SPARE] All primary drones succeeded! No backup needed. Landing.")
        drone.land()
        marker_client.send_update('status', status_message="Spare - no backup needed")
        return

    # --- PHASE 3: Am I assigned to a failed rank? ---
    spare_index = SPARE_DRONES.index(drone_id)
    if spare_index >= len(failed_ranks):
        print(f"[SPARE] Drone {drone_id} is spare #{spare_index} but only {len(failed_ranks)} failures. Not needed. Landing.")
        drone.land()
        return

    assigned_rank = failed_ranks[spare_index]
    print(f"[SPARE] Drone {drone_id} assigned to fill RANK {assigned_rank} (replacing drone {RELAY_HIERARCHY1[assigned_rank]})")
    marker_client.send_update('status', status_message=f"Backup: filling rank {assigned_rank}")

    # --- PHASE 4: Execute the backup flight plan ---
    # Use the same execute_waypoints logic but with overridden rank
    # and NO sync waits (primary chain is already down, we're just filling a gap)
    execute_backup_waypoints(drone, assigned_rank)

def main():
    global marker_client, pos, sx, sy, uwb_ground_height, start_heading, relay_data, landed_cleanly
    global RELAY_HIERARCHY1, SPARE_DRONES

    num_wps = len(relay_data['wp'])
    RELAY_HIERARCHY1 = RELAY_HIERARCHY[:num_wps]
    SPARE_DRONES = RELAY_HIERARCHY[num_wps:]
    print(f"[RELAY] {num_wps} waypoints → Primary: {RELAY_HIERARCHY1}, Spares: {SPARE_DRONES}")

    # Initialize the drone, connect to it...
    drone = CustomTello()
    print("test mark")
    print("[PORTS]", host, control_port, state_port, video_port)

    #print(f"[DEBUG] Connecting to {getattr(drone, 'host', getattr(drone, '_host', '??'))}:{getattr(drone, 'port', getattr(drone, '_port', '??'))}")
    drone.connect()

    marker_client = MarkerClient(drone_id)
    uwb_raw = (0,0,0)
    while uwb_raw == (0,0,0):
        uwb_raw = get_target_position(tag_id)
    uwb_pos = [uwb_raw[0]*100, uwb_raw[1]*100]
    uwb_ground_height = uwb_raw[2]*100
    print(f" Ground pos: {uwb_pos}")
    pos = uwb_pos
    sx, sy = uwb_pos
    start_heading = drone.get_yaw()
    marker_client.client_takeoff_simul([99], f'Battery: {drone.get_battery()} Pos: {int(pos[0]), int(pos[1])}')

    #time.sleep(180)
    try:
        drone.takeoff()
    except Exception as e:
        print(f"[CRITICAL] Takeoff failed: {e}")
        marker_client.send_progress(900)
        print(f"[SYNC] Drone {drone_id} takeoff failed. Progress 900 sent.")
        return

    delay_count = 0
    while delay_count < delay:
        drone.send_rc_control(0, 0, 0, 0)
        print(drone.get_battery())
        time.sleep(5)
        delay_count += 5
        print(delay_count)

    # --- STAGGERED STARTUP (NEW) ---
    # Rank 0 starts immediately, Rank 1 waits 5s, Rank 2 waits 10s, etc.
    # This avoids all 5 drones hitting WiFi simultaneously during the first move.
    if drone_id in RELAY_HIERARCHY:
        rank = RELAY_HIERARCHY.index(drone_id)
        startup_delay = rank * 3
        if startup_delay > 0:
            print(f"[SYNC] Drone {drone_id} is rank {rank}. Waiting {startup_delay}s before starting routine...")
            stagger_count = 0
            while stagger_count < startup_delay:
                drone.send_rc_control(0, 0, 0, 0)  # Keep alive while waiting
                time.sleep(1)
                stagger_count += 1

    # Telemetry thread (no video stream to save bandwidth)
    print("Starting telemetry thread.\n")
    stream_thread = threading.Thread(target=stream_video, args=(drone,))
    stream_thread.daemon = True
    stream_thread.start()

    # After takeoff and stagger delay, replace the flight_routine call:

    if drone_id in RELAY_HIERARCHY1:
        # Primary relay drone — normal flow
        flight_routine(drone)
    elif drone_id in SPARE_DRONES:
        # Spare drone — hover and wait for failures
        spare_drone_routine(drone)
    else:
        print(f"[WARNING] Drone {drone_id} not in any group. Landing.")
        drone.land()

    print("Flight routine ended.")

    # Reboot the drone at the end
    #drone.reboot()

if __name__ == "__main__":
    relay_data = load_master_relay(use_tcp=False)  # Must load before validation
    num_wps = len(relay_data['wp'])
    RELAY_HIERARCHY1 = RELAY_HIERARCHY[:num_wps]
    SPARE_DRONES = RELAY_HIERARCHY[num_wps:]
    print("Validating waypoints...")
    if validate_waypoints():
        print("Validation passed. Starting execution...")
        main()
    else:
        print("Validation failed. Please check warnings above.")