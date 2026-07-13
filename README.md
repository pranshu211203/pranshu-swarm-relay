# pranshu-swarm-relay

**Autonomous deployment and trajectory execution of an ad-hoc aerial victim relay** — a
multi-drone system that flies a chain of DJI Robomaster TT (Tello) drones through a
**GPS-denied indoor arena** and lands them in a contiguous, sub-metre-spaced relay line.

Built for the **SAFMC 2026 Swarm Challenge** — placed **3rd Runners-Up** with a **Judge's
Commendation**.

---

## About this repository

This was a **team project**. The full system (including the centralised swarm server) was
developed collaboratively.

This repo contains **my portion** of the work — the drone-side autonomy — extracted and
organised as a portfolio piece. Where my code depends on a teammate's module, that module
is **not** reproduced here (see *What's mine vs. the team's* below).

### What's mine vs. the team's

**Mine (in this repo):**
- **Per-drone mission execution** (`mission/`) — takeoff, UWB-guided waypoint following,
  rank-based path truncation, progress synchronisation, and landing for each drone.
- **UWB positioning input** (`uwb/UWB_ReadUDP.py`) — opens the UDP socket, parses the raw
  LinkTrack tag stream, and converts it into arena coordinates used for drift correction.
- **Relay receiver** (`relay/relay_receiver.py`) — receives the generated relay assignment
  (`master_relay.json`) on the flight laptops and hands it to the drone scripts.
- **Launch / RPi orchestration** (`scripts/`) and result analysis (`analysis/`).

**Teammate's (referenced, not included):**
- `swarmserverclient.py` — the swarm server and `MarkerClient` coordination layer. My
  mission code *uses* its interface for inter-drone TCP/UDP messaging (progress sentinels,
  waypoint locking) but I did not write it. Its interface is documented in
  [`relay/SWARMSERVER_INTERFACE.md`](relay/SWARMSERVER_INTERFACE.md).

---

## What it does

The relay deploys up to **11 drones** from a start area to a bonus-victim location, forming
a contiguous path of landed drones spaced within one metre of each other to activate a
score multiplier. The interesting problems are all consequences of flying many drones along
one shared corridor indoors, with no GPS:

- **Rank-based path truncation** — an A* path is generated once, then each drone flies a
  truncated version of it so drones land in sequence along the corridor, forming the chain.
- **Pipeline traffic control** — staggered take-off, **waypoint-based progress
  synchronisation**, and departure-based flow control keep up to 11 drones from colliding on
  the shared linear path.
- **UWB drift correction** — periodic position correction in flight and high-accuracy
  correction just before landing, with safeguards against **stale readings** and sensor
  glitches (staleness detection, large-correction confirmation).
- **Fault tolerance** — an automatic **spare-drone** mechanism detects a failed drone and
  deploys a backup into its relay slot.

For the full design and results, see the FYP report below.

---

## Repository layout

```
mission/    Final per-drone mission scripts — one per drone, named by drone ID
uwb/        UWB tag input: UDP listener + coordinate parsing (mine)
relay/      Relay receiver + interface doc for the (excluded) swarm server
scripts/    Windows .bat launchers + Raspberry Pi wi-fi / SSH / shutdown helpers
config/     Sample configs (drones.json, master_relay.example.json)
analysis/   Post-mission plotting
archive/    Older iterations of the mission script (history, not run)
docs/       drone_flow.md + the FYP report PDF
```

> **Note on layout / running it.** The system originally ran with all scripts in a single
> flat working directory, and the imports (`from UWB_ReadUDP import ...`,
> `from swarmserverclient import ...`) assume that. The folders here are for *readability*.
> It also isn't runnable stand-alone — it needs the physical hardware (Tello drones, a
> LinkTrack UWB setup, the Raspberry Pi bridges) and the teammate's swarm server. This repo
> is meant to be **read**, not executed end-to-end.

### Drone → relay-group mapping

The 11 drones are split across two routers / relay groups. Filenames encode the drone ID
(e.g. `Known_uwb_pranshu_22.py` runs on drone 22).

| Relay group | Drone IDs |
| --- | --- |
| Group 1 (Router 1) | 19, 20, 21, 28, 29 |
| Group 2 (Router 2) | 14, 22, 23, 24, 25, 27 |

The ordering used at runtime is the `RELAY_HIERARCHY` list inside each mission script. Exact
laptop / router / Raspberry-Pi assignments are described in the FYP report (§3.4, Network
Topology).

---

## FYP report

The design, literature review, implementation details, and mission results are in my final
year project report:

**[`docs/FYP_report.pdf`](docs/FYP_report.pdf)** — *Autonomous Deployment and Trajectory
Execution of an Ad-Hoc Aerial Victim Relay* (NTU, School of Mechanical & Aerospace
Engineering, AY 2025/26, Project D052).

It covers the relay strategy (rank-based truncation, altitude separation, dynamic direction
reversal), the collision-avoidance / traffic model, the UWB positioning pipeline and its
safeguards, and the spare-drone fault-tolerance mechanism.

---

## Excluded from this repo

- `swarmserverclient.py` — teammate's module (see interface doc above).
- `nlink_unpack_COMx_udp.exe` — third-party binary from the **LinkTrack** UWB console that
  unpacks the tag stream and rebroadcasts it over UDP. It runs on a separate Windows PC
  connected to the UWB console; it is not redistributed here.

---

## Setup

```bash
pip install -r requirements.txt
```

---

## Author

**Pranshu Agarwal** — Aerospace Engineering (Autonomous Systems), Nanyang Technological
University. GitHub: [@pranshu211203](https://github.com/pranshu211203)
