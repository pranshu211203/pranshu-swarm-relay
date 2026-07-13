"""
relay_receiver.py — Run this BEFORE the search mission starts.
Listens on TCP port 6000 for master_relay.json from Manish's swarmserver.
Once received, saves it locally, sends ACK, launches drones, and auto-triggers takeoff.

Usage:
    python relay_receiver.py              ← receive file + ACK (drones already running, no auto-takeoff)
    python relay_receiver.py --launch     ← receive file + ACK + launch bat + auto-takeoff

On competition day:
    1. Run this script with --launch on the relay laptop
    2. Search missions run on Manish's laptop
    3. Manish clicks relay button → swarmserver sends master_relay.json here
    4. This script saves it, launches drones, waits for them to register, then sends takeoff
    5. Fully automated — no manual clicking needed
"""

import socket
import json
import subprocess
import os
import sys
import time
import argparse

PORT = 6000
BAT_FILE = "launch_all_drones_relay1.bat"
OUTPUT_FILE = "master_relay.json"

# --- Swarmserver protocol settings (must match swarmserverclient.py) ---
SWARM_PORT = 5005
BROADCAST_IP = "192.168.1.255"

# How many drones are we expecting from the bat file?
# This is the total count of drone scripts launched (primary + spares)
EXPECTED_DRONE_COUNT = 11  # Adjust to match your bat file (22, 23, 24, 25, 14 = 5)

# Timing
READY_SETTLE_TIME = 5     # seconds to wait after last new drone registers (let stragglers catch up)
MAX_WAIT_TIME = 60        # absolute max seconds to wait before sending takeoff anyway


def wait_for_drones_and_takeoff(expected_count):
    """
    Listen on the swarmserver UDP port for takeoff_request messages from drone scripts.
    Once all expected drones have registered (or timeout), broadcast the takeoff signal.
    """
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    udp.bind(("0.0.0.0", SWARM_PORT))
    udp.settimeout(1.0)  # 1s poll interval

    ready_drones = {}       # drone_id -> client_addr (ip, port)
    start_time = time.time()
    last_new_drone_time = time.time()

    print(f"\n[TAKEOFF] Waiting for {expected_count} drones to register on port {SWARM_PORT}...")
    print(f"[TAKEOFF] Will send takeoff {READY_SETTLE_TIME}s after all register, or after {MAX_WAIT_TIME}s max.\n")

    while True:
        elapsed = time.time() - start_time

        # --- Absolute timeout ---
        if elapsed > MAX_WAIT_TIME:
            print(f"[TAKEOFF] Max wait ({MAX_WAIT_TIME}s) reached. Proceeding with {len(ready_drones)} drones.")
            break

        # --- All drones registered + settle time passed ---
        if len(ready_drones) >= expected_count:
            since_last = time.time() - last_new_drone_time
            if since_last >= READY_SETTLE_TIME:
                print(f"[TAKEOFF] All {len(ready_drones)} drones registered. Settle time passed.")
                break

        # --- Listen for takeoff_request messages ---
        try:
            data, addr = udp.recvfrom(4096)
            msg = json.loads(data.decode())

            if msg.get("type") == "takeoff_request" and msg.get("ready"):
                did = msg["drone_id"]
                if did not in ready_drones:
                    ready_drones[did] = addr
                    last_new_drone_time = time.time()
                    bat_str = msg.get("status", "")
                    print(f"[TAKEOFF] Drone {did} ready ({len(ready_drones)}/{expected_count}) — {bat_str}")
                else:
                    # Update address in case it changed (shouldn't, but be safe)
                    ready_drones[did] = addr

        except socket.timeout:
            # Print periodic status
            if int(elapsed) % 5 == 0 and int(elapsed) > 0:
                print(f"[TAKEOFF] {int(elapsed)}s elapsed, {len(ready_drones)}/{expected_count} drones ready...")
            continue
        except (json.JSONDecodeError, KeyError):
            continue

    # --- Send takeoff signal ---
    if not ready_drones:
        print("[TAKEOFF] No drones registered! Cannot send takeoff.")
        udp.close()
        return

    drone_ids = list(ready_drones.keys())
    takeoff_msg = json.dumps({"type": "takeoff", "takeoff_list": drone_ids}).encode()

    print(f"\n[TAKEOFF] Sending takeoff signal to drones: {drone_ids}")

    # Send to each drone's specific address (same as server does)
    for did, client_addr in ready_drones.items():
        for _ in range(5):  # 5x for reliability (UDP is lossy)
            try:
                udp.sendto(takeoff_msg, client_addr)
            except Exception as e:
                print(f"[WARNING] Failed to send to drone {did} at {client_addr}: {e}")
            time.sleep(0.01)

    # Also broadcast on the subnet as a fallback
    for _ in range(5):
        try:
            udp.sendto(takeoff_msg, (BROADCAST_IP, SWARM_PORT))
        except:
            pass
        time.sleep(0.01)

    print(f"[TAKEOFF] ✅ Takeoff signal sent to {len(drone_ids)} drones!")
    udp.close()


def main():
    parser = argparse.ArgumentParser(description="Receive master_relay.json from swarm server")
    parser.add_argument("--launch", action="store_true",
                        help="Auto-launch bat + send takeoff signal after receiving")
    args = parser.parse_args()

    if args.launch and not os.path.isfile(BAT_FILE):
        print(f"[ERROR] --launch specified but {BAT_FILE} not found in {os.getcwd()}")
        sys.exit(1)

    # --- Delete stale master_relay.json so drones don't load old data ---
    if os.path.exists(OUTPUT_FILE):
        os.remove(OUTPUT_FILE)
        print(f"[RECEIVER] Deleted stale {OUTPUT_FILE}")

    print(f"[RECEIVER] Listening on port {PORT} for master_relay.json...")
    if args.launch:
        print(f"[RECEIVER] Will auto-launch {BAT_FILE} + send takeoff signal after receiving.")
    else:
        print(f"[RECEIVER] File-only mode — drones should already be running.")
    print(f"[RECEIVER] Waiting for connection from swarmserver...\n")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', PORT))
    sock.listen(1)

    try:
        conn, addr = sock.accept()
        print(f"[RECEIVER] Connection from {addr[0]}:{addr[1]}")

        # Read 8-byte length header
        length_data = b''
        while len(length_data) < 8:
            chunk = conn.recv(8 - len(length_data))
            if not chunk:
                print("[ERROR] Connection closed while reading header")
                return
            length_data += chunk
        expected = int(length_data.decode('utf-8'))
        print(f"[RECEIVER] Expecting {expected} bytes...")

        # Read JSON payload
        received = b''
        while len(received) < expected:
            chunk = conn.recv(min(4096, expected - len(received)))
            if not chunk:
                print("[ERROR] Connection closed while reading payload")
                return
            received += chunk

        # Parse and validate
        data = json.loads(received.decode('utf-8'))
        num_wps = len(data.get('wp', []))
        if num_wps == 0:
            print("[ERROR] Received JSON has 0 waypoints! Aborting.")
            conn.close()
            sock.close()
            return

        # Save locally
        with open(OUTPUT_FILE, 'w') as f:
            json.dump(data, f, indent=4)
        print(f"[RECEIVER] Saved {OUTPUT_FILE} ({num_wps} waypoints)")

        # Print waypoint summary
        for i, wp in enumerate(data['wp']):
            pos = wp.get('position_cm', {})
            print(f"  WP{i}: ({pos.get('x', '?')}, {pos.get('y', '?')}) dist={wp.get('dist_cm', '?')}cm angle={wp.get('angle_deg', '?')}°")

        # Send ACK back to server BEFORE closing connection
        ack_msg = f"ACK:{num_wps}".encode('utf-8')
        try:
            conn.sendall(ack_msg)
            print(f"[RECEIVER] Sent ACK: 'ACK:{num_wps}'")
        except Exception as e:
            print(f"[WARNING] Failed to send ACK: {e}")

        conn.close()
        sock.close()

        time.sleep(0.5)

        print(f"\n[RECEIVER] ✅ master_relay.json received and saved successfully!")
        print(f"[RECEIVER]    {num_wps} waypoints, {expected} bytes")

        # Launch drone scripts + auto-takeoff
        if args.launch:
            print(f"\n[RECEIVER] Launching {BAT_FILE}...")
            subprocess.Popen(BAT_FILE, shell=True)
            print(f"[RECEIVER] Drones launched. Waiting for them to initialize...\n")

            # Wait for drones to register as ready, then send takeoff signal
            wait_for_drones_and_takeoff(EXPECTED_DRONE_COUNT)
        else:
            print(f"[RECEIVER]    Relay drones can now load the file and take off.")

    except KeyboardInterrupt:
        print("\n[RECEIVER] Cancelled by user.")
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        try:
            sock.close()
        except:
            pass

if __name__ == "__main__":
    main()
