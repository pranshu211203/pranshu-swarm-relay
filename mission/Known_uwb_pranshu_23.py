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

match = re.search(r'Known_uwb_pranshu_(\d+)', script_name)

if match:
    drone_id = int(match.group(1))  # Extract the number and convert it to an integer
    print(f"Extracted ID: {drone_id}")
else:
    print("Script name does not match the expected pattern.")
    drone_id = 0  # Default drone ID

# The order matters! First ID = Farthest Drone (Full Path). Last ID = Closest Drone (Shortest Path).
# Example: Drone 8 goes to 5,0. Drone 10 goes to 4,0.
RELAY_HIERARCHY = [19, 20, 21, 28, 29, 22, 23, 24, 25, 14]
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
            # Update pos so the next call doesn't recompute the same deviation
            pos[0] = uwb_pos[0]
            pos[1] = uwb_pos[1]
            print(f"[PRECISE] pos updated to {pos}")
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

    # --- MERGE COLLINEAR WAYPOINTS (optimize path, keep sync IDs) ---
    SLOPE_TOLERANCE = 3  # degrees
    original_wps = data['wp']
    merged_wps = []
    original_indices = []  # which original WP indices each merged entry covers

    i = 0
    while i < len(original_wps):
        wp = original_wps[i]

        if wp['dist_cm'] == 0:
            merged_wps.append(wp)
            original_indices.append([i])
            i += 1
            continue

        if i + 1 >= len(original_wps):
            merged_wps.append(wp)
            original_indices.append([i])
            i += 1
            continue

        cur_pos = wp['position_cm']
        next_pos = original_wps[i + 1]['position_cm']
        cur_slope = math.degrees(math.atan2(
            next_pos['x'] - cur_pos['x'],
            next_pos['y'] - cur_pos['y']
        ))

        merged_idx = [i]
        j = i + 1
        while j < len(original_wps) and original_wps[j]['dist_cm'] != 0:
            if j + 1 < len(original_wps):
                j_pos = original_wps[j]['position_cm']
                j_next = original_wps[j + 1]['position_cm']
                j_slope = math.degrees(math.atan2(
                    j_next['x'] - j_pos['x'],
                    j_next['y'] - j_pos['y']
                ))
                if abs(cur_slope - j_slope) < SLOPE_TOLERANCE:
                    merged_idx.append(j)
                    j += 1
                else:
                    break
            else:
                break

        if len(merged_idx) > 1:
            target_pos = original_wps[j]['position_cm']
            total_dist = int(math.sqrt(
                (target_pos['x'] - cur_pos['x'])**2 +
                (target_pos['y'] - cur_pos['y'])**2
            ))
            merged_wps.append({
                'dist_cm': total_dist,
                'angle_deg': wp['angle_deg'],
                'position_cm': wp['position_cm']
            })
            print(f"[MERGE] WPs {merged_idx[0]}→{merged_idx[-1]} same slope ({cur_slope:.1f}°), combined into {total_dist}cm")
        else:
            merged_wps.append(wp)

        original_indices.append(merged_idx)
        i = j

    print(f"[MERGE] {len(original_wps)} waypoints → {len(merged_wps)} after merge")
    data['wp'] = merged_wps
    # --- END MERGE ---

    # Extracted from the first waypoint in the sliced list
    start_wpt = [data['wp'][0]['position_cm']['x'], data['wp'][0]['position_cm']['y']]
    print(f"Starting waypoint is {start_wpt}")
    
    waypoint_id = 0 # Waypoint 0 is the starting waypoint

    # --- NEW: RANK-BASED STARTUP STAGGER ---
    # Each drone waits for the drone ahead to have DEPARTED from WP0 before claiming it.
    # This prevents the initial race where all drones claim WP0 simultaneously.
    my_rank = RELAY_HIERARCHY1.index(drone_id)
    SYNC_TIMEOUT = SYNC_TIMEOUT_BASE + my_rank * 15 # Rank 0: 45s, ........
    print(f"[SYNC] Drone {drone_id} rank {my_rank}, sync timeout = {SYNC_TIMEOUT}s")
    sync_start = time.time()  # Initialize for all ranks (used in subsequent waits too)

    if my_rank > 0:
        ahead_id = RELAY_HIERARCHY1[my_rank - 1]
        print(f"[SYNC] Drone {drone_id} is rank {my_rank}. Waiting for drone {RELAY_HIERARCHY1[my_rank-1]} to LEAVE WP0...")
        sync_start = time.time()
        while not marker_client.can_proceed_to_waypoint(0, RELAY_HIERARCHY1):
            drone.send_rc_control(0, 0, 0, 0)
            ahead_progress = marker_client.drone_progress.get(str(ahead_id), -1)
            if ahead_progress >= 900:
                print(f"[SYNC] Drone ahead ({ahead_id}) sent progress {ahead_progress}. Proceeding immediately.")
                break
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
    
    time.sleep(1)
    uwb_correction_precise(drone)
    # Re-assert claim on the start point after correcting (UDP safety)
    marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)
    
    # NOTE: Do NOT send_progress(0) here! We are still AT WP0.
    # Progress is only announced when we DEPART from a waypoint.

    abs_position = {"x": start_wpt[0], "y": start_wpt[1]}
    orientation = 180

    try:
        current_wp_id = 0 # We are starting at the start point (0)
        
        # enumerate gives us 'i' (0, 1, 2...) alongside the waypoint data
        for i, wp in enumerate(data['wp']):
            # Replace: target_wp_id = i + 1
            target_wp_id = original_indices[i][-1] + 1 if i < len(original_indices) else i + 1

            # --- FINAL WAYPOINT: Skip sync wait entirely ---
            # dist_cm=0 means "land here" — we're not flying to target_wp_id,
            # so there's no reason to wait for the drone ahead to clear it.
            if wp['dist_cm'] == 0:
                print(f"[INFO] WP {target_wp_id} has dist_cm=0 (final waypoint). Skipping sync wait and movement.")
                wp_ids = original_indices[i] if i < len(original_indices) else [current_wp_id]
                for wid in wp_ids:
                    marker_client.send_update('waypoint', marker_id=wid, detected=False)
                    marker_client.send_progress(wid)
                print(f"[SYNC] Drone {drone_id} departed WPs {wp_ids}, followers unblocked.")
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
                # Check if ahead drone failed
                if my_rank > 0:
                    ahead_id = RELAY_HIERARCHY1[my_rank - 1]
                    ahead_progress = marker_client.drone_progress.get(str(ahead_id), -1)
                    if ahead_progress >= 900:
                        print(f"[SYNC] Drone ahead ({ahead_id}) progress={ahead_progress}. Proceeding.")
                        break
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

            # --- DIRECT MOVEMENT (skip rotation, use go command) ---

            distance = wp['dist_cm']
            status = "Moving to next waypoint"
            # Get next waypoint position
            if i + 1 < len(data['wp']):
                next_pos = data['wp'][i + 1]['position_cm']
                target_x = next_pos['x']
                target_y = next_pos['y']
            else:
                target_x = abs_position['x']
                target_y = abs_position['y']

            # World-frame displacement
            world_dx = target_x - abs_position['x']
            world_dy = target_y - abs_position['y']

            # Convert to body frame (same math as uwb_correction)
            rad = math.radians(heading)
            body_x = world_dx * math.cos(rad) - world_dy * math.sin(rad)
            body_y = world_dy * math.cos(rad) + world_dx * math.sin(rad)

            # go command params (same convention as uwb_correction)
            go_fwd = int(body_y)
            go_left = int(-body_x)
            dist = math.sqrt(go_fwd**2 + go_left**2)
            print(f"[MOVE] WP{current_wp_id}→WP{target_wp_id}: fwd={go_fwd}, left={go_left}, dist={dist:.0f}cm")

            if dist > 200:
                num_chunks = math.ceil(dist / 200)
                cfwd = int(go_fwd / num_chunks)
                cleft = int(go_left / num_chunks)
                # Release immediately — follower can start approaching while we move
                wp_ids = original_indices[i] if i < len(original_indices) else [current_wp_id]
                for wid in wp_ids:
                    marker_client.send_update('waypoint', marker_id=wid, detected=False)
                    marker_client.send_progress(wid)
                print(f"[SYNC] Drone {drone_id} departed WPs {wp_ids}, followers unblocked.")

                for c in range(num_chunks):
                    drone.send_command_without_return(f"go {cfwd} {cleft} 0 100")
                    time.sleep(2)
                    # Update pos progressively so uwb_correction has the right target
                    pos[0] += (target_x - abs_position['x']) / num_chunks
                    pos[1] += (target_y - abs_position['y']) / num_chunks
                    uwb_correction(drone)
            elif dist > 20:
                # Release immediately — follower can start approaching while we move
                wp_ids = original_indices[i] if i < len(original_indices) else [current_wp_id]
                for wid in wp_ids:
                    marker_client.send_update('waypoint', marker_id=wid, detected=False)
                    marker_client.send_progress(wid)
                print(f"[SYNC] Drone {drone_id} departed WPs {wp_ids}, followers unblocked.")

                drone.send_command_without_return(f"go {go_fwd} {go_left} 0 100")
                time.sleep(max(2, int(dist / 100) + 1))
                # Update pos before correction
                pos[0] = target_x
                pos[1] = target_y
                uwb_correction(drone)

            # Update position to target
            abs_position['x'] = target_x
            abs_position['y'] = target_y
            pos[0] = target_x
            pos[1] = target_y

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
            time.sleep(1)
            uwb_correction(drone)

        bat = drone.get_battery()
        print(f"[LANDING] Battery level before landing: {bat}%")

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

        # Verify landing position — compare against drone ahead's actual position
        time.sleep(3)
        avg = uwb_averaged_reading(samples=6, interval=0.5)
        if avg is not None:
            final_pos = [avg[0], avg[1]]
        else:
            final_pos = uwb_reading(drone)

        my_rank = RELAY_HIERARCHY1.index(drone_id)

        if my_rank > 0 and final_pos != [0, 0]:
            # Get ahead drone's tag_id from drones.json
            ahead_drone_id = RELAY_HIERARCHY1[my_rank - 1]
            ahead_tag_id = None
            for d in DRONE_INFO:
                if d['id'] == ahead_drone_id:
                    ssid = d.get('TELLO_SSID', '')
                    m = re.search(r'RMTT-TAG(\d+)', ssid)
                    if m:
                        ahead_tag_id = int(m.group(1))
                    break

            if ahead_tag_id is not None:
                ahead_raw = get_target_position(ahead_tag_id)
                if ahead_raw != (0, 0, 0):
                    ahead_pos = [ahead_raw[0] * 100, ahead_raw[1] * 100]
                    separation = math.sqrt((final_pos[0] - ahead_pos[0])**2 + (final_pos[1] - ahead_pos[1])**2)
                    print(f"[LANDING] Me: ({final_pos[0]:.0f}, {final_pos[1]:.0f}), Ahead drone {ahead_drone_id}: ({ahead_pos[0]:.0f}, {ahead_pos[1]:.0f}), Separation: {separation:.0f}cm")

                    if separation <= 105:
                        print(f"[LANDING] Separation OK ({separation:.0f}cm).")
                        marker_client.send_update('status', status_message="Landed at Relay")
                        marker_client.send_progress(999)
                        landed_cleanly = True
                    else:
                        print(f"[LANDING] Too far from drone ahead ({separation:.0f}cm >= 100cm). Marking as failed.")
                        marker_client.send_update('status', status_message="Relay broken")
                        marker_client.send_progress(900)
                        landed_cleanly = False
                else:
                    print("[LANDING] Can't read ahead drone's UWB. Assuming success.")
                    marker_client.send_update('status', status_message="Landed at Relay")
                    marker_client.send_progress(999)
                    landed_cleanly = True
            else:
                print("[LANDING] Can't find ahead drone's tag_id. Assuming success.")
                marker_client.send_update('status', status_message="Landed at Relay")
                marker_client.send_progress(999)
                landed_cleanly = True
        elif final_pos == [0, 0]:
            # Tag worked at takeoff but is now dead — likely a crash or battery disconnect
            try:
                bat = drone.get_battery()
            except:
                bat = 0  # Can't reach drone — almost certainly crashed
            if bat > 20:
                print(f"[LANDING] No UWB data but battery OK ({bat}%). Assuming success.")
                marker_client.send_update('status', status_message="Landed at Relay (no UWB)")
                marker_client.send_progress(999)
                landed_cleanly = True
            else:
                print(f"[LANDING] No UWB data AND battery low ({bat}%). Likely crashed. Marking failed.")
                marker_client.send_update('status', status_message="Landed - UWB dead, low bat")
                marker_client.send_progress(900)
                landed_cleanly = False
        else:
            # Rank 0 — no drone ahead to compare against
            print("[LANDING] Rank 0, no drone ahead. Assuming success.")
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
    """Simplified execute_waypoints for backup drones — no sync waits needed.
    Uses same body-frame go movement and collinear merge as primary."""
    global status, course, pos, landed_cleanly

    data = get_relay_flight_plan(drone_id, override_rank=assigned_rank)

    if not data or not data.get('wp'):
        print(f"[BACKUP] No waypoints for rank {assigned_rank}. Landing.")
        drone.land()
        return

    # --- MERGE COLLINEAR WAYPOINTS (same logic as primary) ---
    SLOPE_TOLERANCE = 3
    original_wps = data['wp']
    merged_wps = []
    original_indices = []

    i = 0
    while i < len(original_wps):
        wp = original_wps[i]

        if wp['dist_cm'] == 0:
            merged_wps.append(wp)
            original_indices.append([i])
            i += 1
            continue

        if i + 1 >= len(original_wps):
            merged_wps.append(wp)
            original_indices.append([i])
            i += 1
            continue

        cur_pos = wp['position_cm']
        next_pos = original_wps[i + 1]['position_cm']
        cur_slope = math.degrees(math.atan2(
            next_pos['x'] - cur_pos['x'],
            next_pos['y'] - cur_pos['y']
        ))

        merged_idx = [i]
        j = i + 1
        while j < len(original_wps) and original_wps[j]['dist_cm'] != 0:
            if j + 1 < len(original_wps):
                j_pos = original_wps[j]['position_cm']
                j_next = original_wps[j + 1]['position_cm']
                j_slope = math.degrees(math.atan2(
                    j_next['x'] - j_pos['x'],
                    j_next['y'] - j_pos['y']
                ))
                if abs(cur_slope - j_slope) < SLOPE_TOLERANCE:
                    merged_idx.append(j)
                    j += 1
                else:
                    break
            else:
                break

        if len(merged_idx) > 1:
            target_pos = original_wps[j]['position_cm']
            total_dist = int(math.sqrt(
                (target_pos['x'] - cur_pos['x'])**2 +
                (target_pos['y'] - cur_pos['y'])**2
            ))
            merged_wps.append({
                'dist_cm': total_dist,
                'angle_deg': wp['angle_deg'],
                'position_cm': wp['position_cm']
            })
            print(f"[BACKUP MERGE] WPs {merged_idx[0]}→{merged_idx[-1]} same slope ({cur_slope:.1f}°), combined into {total_dist}cm")
        else:
            merged_wps.append(wp)

        original_indices.append(merged_idx)
        i = j

    print(f"[BACKUP MERGE] {len(original_wps)} waypoints → {len(merged_wps)} after merge")
    data['wp'] = merged_wps
    # --- END MERGE ---

    start_wpt = [data['wp'][0]['position_cm']['x'], data['wp'][0]['position_cm']['y']]
    print(f"[BACKUP] Target start waypoint: {start_wpt}")

    # Navigate to starting position — chunked, body-frame go
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

    time.sleep(1)
    uwb_correction_precise(drone)

    abs_position = {"x": start_wpt[0], "y": start_wpt[1]}

    try:
        for i, wp in enumerate(data['wp']):
            # Final waypoint — skip movement, just land
            if wp['dist_cm'] == 0:
                print(f"[BACKUP] Final waypoint reached (dist_cm=0). Proceeding to land.")
                break

            # --- DIRECT MOVEMENT (body-frame go, no sync waits) ---
            distance = wp['dist_cm']
            if i + 1 < len(data['wp']):
                next_pos = data['wp'][i + 1]['position_cm']
                target_x = next_pos['x']
                target_y = next_pos['y']
            else:
                target_x = abs_position['x']
                target_y = abs_position['y']

            # World-frame displacement
            world_dx = target_x - abs_position['x']
            world_dy = target_y - abs_position['y']

            # Convert to body frame using heading
            rad = math.radians(heading)
            body_x = world_dx * math.cos(rad) - world_dy * math.sin(rad)
            body_y = world_dy * math.cos(rad) + world_dx * math.sin(rad)

            go_fwd = int(body_y)
            go_left = int(-body_x)
            dist = math.sqrt(go_fwd**2 + go_left**2)
            print(f"[BACKUP MOVE] WP{i}→WP{i+1}: fwd={go_fwd}, left={go_left}, dist={dist:.0f}cm")

            if dist > 200:
                num_chunks = math.ceil(dist / 200)
                cfwd = int(go_fwd / num_chunks)
                cleft = int(go_left / num_chunks)

                for c in range(num_chunks):
                    drone.send_command_without_return(f"go {cfwd} {cleft} 0 100")
                    time.sleep(2)
                    pos[0] += (target_x - abs_position['x']) / num_chunks
                    pos[1] += (target_y - abs_position['y']) / num_chunks
                    uwb_correction(drone)
            elif dist > 20:
                drone.send_command_without_return(f"go {go_fwd} {go_left} 0 100")
                time.sleep(max(2, int(dist / 100) + 1))
                pos[0] = target_x
                pos[1] = target_y
                uwb_correction(drone)

            abs_position['x'] = target_x
            abs_position['y'] = target_y
            pos[0] = target_x
            pos[1] = target_y

        # --- LANDING ---
        bat = drone.get_battery()
        if bat > 15:
            print("[BACKUP] Running final UWB corrections for accuracy...")
            uwb_correction_precise(drone)
            time.sleep(1)
            uwb_correction_precise(drone)
        else:
            print(f"[BACKUP] Battery low ({bat}%). Quick correction only.")
            uwb_correction(drone)
            time.sleep(1)
            uwb_correction(drone)

        bat = drone.get_battery()
        print(f"[BACKUP] Battery level before landing: {bat}%")
        print("[BACKUP] Landing at relay point.")

        try:
            drone.land()
        except Exception as e:
            if "Auto land" in str(e) or "error" in str(e):
                print(f"[BACKUP] Land command failed ({e}), but drone is likely already landing.")
            else:
                raise

        # Landing verification — same logic as primary
        time.sleep(3)
        avg = uwb_averaged_reading(samples=6, interval=0.5)
        if avg is not None:
            final_pos = [avg[0], avg[1]]
        else:
            final_pos = uwb_reading(drone)

        if assigned_rank > 0 and final_pos != [0, 0]:
            ahead_drone_id = RELAY_HIERARCHY1[assigned_rank - 1]
            ahead_tag_id = None
            for d in DRONE_INFO:
                if d['id'] == ahead_drone_id:
                    ssid = d.get('TELLO_SSID', '')
                    m = re.search(r'RMTT-TAG(\d+)', ssid)
                    if m:
                        ahead_tag_id = int(m.group(1))
                    break

            if ahead_tag_id is not None:
                ahead_raw = get_target_position(ahead_tag_id)
                if ahead_raw != (0, 0, 0):
                    ahead_pos = [ahead_raw[0] * 100, ahead_raw[1] * 100]
                    separation = math.sqrt((final_pos[0] - ahead_pos[0])**2 + (final_pos[1] - ahead_pos[1])**2)
                    print(f"[BACKUP] Me: ({final_pos[0]:.0f}, {final_pos[1]:.0f}), Ahead drone {ahead_drone_id}: ({ahead_pos[0]:.0f}, {ahead_pos[1]:.0f}), Separation: {separation:.0f}cm")

                    if separation <= 115:
                        print(f"[BACKUP] Separation OK ({separation:.0f}cm).")
                        marker_client.send_update('status', status_message="Backup Landed at Relay")
                        landed_cleanly = True
                    else:
                        print(f"[BACKUP] Too far from drone ahead ({separation:.0f}cm). Marking as failed.")
                        marker_client.send_update('status', status_message="Backup Relay broken")
                        landed_cleanly = False
                else:
                    print("[BACKUP] Can't read ahead drone's UWB. Assuming success.")
                    marker_client.send_update('status', status_message="Backup Landed at Relay")
                    landed_cleanly = True
            else:
                print("[BACKUP] Can't find ahead drone's tag_id. Assuming success.")
                marker_client.send_update('status', status_message="Backup Landed at Relay")
                landed_cleanly = True
        elif final_pos == [0, 0]:
            print("[BACKUP] No UWB data to verify. Assuming success.")
            marker_client.send_update('status', status_message="Backup Landed at Relay")
            landed_cleanly = True
        else:
            print("[BACKUP] Rank 0, no drone ahead. Assuming success.")
            marker_client.send_update('status', status_message="Backup Landed at Relay")
            landed_cleanly = True

    except Exception as e:
        print(f"[BACKUP] Error: {e}")
        landed_cleanly = False
    finally:
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
    rank = RELAY_HIERARCHY.index(drone_id)
    height = 70 if rank % 2 == 0 else 80
    try:
        ascend(drone, height)
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
    uwb_start = time.time()
    while uwb_raw == (0,0,0):
        uwb_raw = get_target_position(tag_id)
        if time.time() - uwb_start > 30:
            print(f"[CRITICAL] UWB ground read timed out after 30s. Tag {tag_id} not responding.")
            break
    if uwb_raw == (0,0,0):
        print(f"[CRITICAL] No UWB data. Cannot fly safely. Exiting.")
        try:
            marker_client.send_progress(900)
        except:
            pass
        return
    uwb_pos = [uwb_raw[0]*100, uwb_raw[1]*100]
    uwb_ground_height = uwb_raw[2]*100
    print(f" Ground pos: {uwb_pos}")
    pos = uwb_pos
    sx, sy = uwb_pos
    start_heading = drone.get_yaw()
    marker_client.client_takeoff_simul([99], f'Battery: {drone.get_battery()} Pos: {int(pos[0]), int(pos[1])}')

    # --- STAGGERED TAKEOFF LOGIC ---
    if drone_id in RELAY_HIERARCHY:
        rank = RELAY_HIERARCHY.index(drone_id)
        stagger_delay = rank * 1  # 1 second delay per rank level
    else:
        stagger_delay = 0  # Fallback just in case

    if stagger_delay > 0:
        print(f"[INFO] Drone {drone_id} (Rank {rank}) waiting {stagger_delay}s for staggered takeoff.")
        time.sleep(stagger_delay)
    # -------------------------------

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
        try:
            main()
        except Exception as e:
            print(f"[CRITICAL] main() crashed: {e}")
            try:
                marker_client.send_progress(900)
                print(f"[SYNC] Drone {drone_id} crashed. Progress 900 sent.")
            except:
                print(f"[SYNC] Could not send progress 900 (marker_client not initialized).")