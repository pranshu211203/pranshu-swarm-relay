1. Laptop 2: launch_drones_laptop2.bat
   Laptop 3: launch_drones_laptop3.bat
   → All 11 drone scripts start
   → Each connects to its RPi/drone, gets UWB ground pos, prints battery
   → Each prints "[READY] Hardware OK. Waiting for master_relay.json..."
   → You can see which drones are healthy and which have problems

2. Laptop 2: python relay_receiver.py
   Laptop 3: python relay_receiver.py
   → Both sit listening on port 6000 (no --launch flag needed anymore)

3. Search phase runs on Laptop 1...

4. A* generates master_relay.json

5. Laptop 1 swarmserver sends JSON → Laptop 2's Scorpion A IP:6000
   Laptop 1 swarmserver sends JSON → Laptop 3's Scorpion A IP:6000

6. Both relay_receivers save master_relay.json + ACK

7. Drone scripts detect the file, load it, validate, register
   with swarmserver, stagger takeoff, execute relay