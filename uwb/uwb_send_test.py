"""
UWB Bridge Script
-----------------
Listens for incoming UWB position data (e.g., from Decawave system)
and republishes it via UDP to another IP:port.

Usage:
    python UWB_Bridge.py

Ensure:
    - UWB_SendUDP.py (or your real UWB system) sends to port 5000.
    - Adjust PUBLISHER_IP and PUBLISHER_PORT to where you want to forward data.
"""

import socket
import time
import pandas as pd
from UWB_ReadUDP import parse_data_to_df

# ===== CONFIGURATION =====
LISTEN_IP = '0.0.0.0'
LISTEN_PORT = 5000         # Incoming UWB data
PUBLISHER_IP = '127.0.0.1' # Where to send the processed data (change if needed)
PUBLISHER_PORT = 5000      # Outgoing port for republished data
PERIOD_S = 0.1             # 10 Hz republish rate

# ==========================

def receive_data(sock, timeout=0.2):
    """Receive UDP data and parse into a DataFrame."""
    sock.settimeout(timeout)
    try:
        data, addr = sock.recvfrom(4096)
        df = parse_data_to_df(data)
        if df is not None and not df.empty:
            return df
    except socket.timeout:
        pass
    except Exception as e:
        print(f"[ERROR] Receiving/parsing data: {e}")
    return pd.DataFrame()


def main():
    # Setup receiving socket
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    recv_sock.bind((LISTEN_IP, LISTEN_PORT))
    print(f"[INFO] Listening for UWB data on {LISTEN_IP}:{LISTEN_PORT}")

    # Setup publishing socket
    pub_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target_address = (PUBLISHER_IP, PUBLISHER_PORT)
    print(f"[INFO] Republishing data to {PUBLISHER_IP}:{PUBLISHER_PORT}")

    try:
        while True:
            df = receive_data(recv_sock)
            if not df.empty:
                for _, row in df.iterrows():
                    # Format message for each tag
                    line = (
                        f"{int(row['id'])},0,{row['x']:.2f},{row['y']:.2f},{row['z']:.2f},"
                        f"{row['dist1']:.2f},{row['dist2']:.2f},{row['dist3']:.2f},"
                        f"{row['dist4']:.2f},{row['dist5']:.2f},{row['dist6']:.2f},"
                        f"{row['dist7']:.2f},{row['dist8']:.2f}"
                    )
                    pub_sock.sendto(line.encode(), target_address)

                print(f"[OK] Sent {len(df)} tag(s) to {PUBLISHER_IP}:{PUBLISHER_PORT}")

            time.sleep(PERIOD_S)

    except KeyboardInterrupt:
        print("\n[INFO] Stopping bridge...")
    finally:
        recv_sock.close()
        pub_sock.close()
        print("[INFO] Sockets closed. Exiting.")


if __name__ == "__main__":
    main()
