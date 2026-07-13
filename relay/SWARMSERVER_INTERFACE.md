# Swarm Server Interface (`MarkerClient`)

The mission scripts in `mission/` depend on a `MarkerClient` class imported from a
`swarmserverclient` module:

```python
from swarmserverclient import MarkerClient
```

**This module was written by a teammate and is intentionally not included in this
repository.** It provides the centralised swarm server plus the per-drone client used
for TCP/UDP coordination between drones. My mission code is a *consumer* of this
interface — I used it to synchronise progress and share waypoint state across the swarm,
but I did not author it.

For transparency, these are the `MarkerClient` members my mission code calls, so the
role of the excluded module is clear:

| Member | How the mission code uses it |
| --- | --- |
| `can_proceed_to_waypoint(wp_id, hierarchy)` | Gate: may this drone advance to a waypoint given the relay hierarchy / drones ahead. |
| `is_waypoint_available(wp_id)` | Check whether a waypoint slot is currently unoccupied before moving into it. |
| `send_update('waypoint', marker_id=..., detected=...)` | Report that a waypoint has been reached / vacated. |
| `send_update('status', status_message=...)` | Push a human-readable status string to the server. |
| `send_progress(wp_id)` | Report this drone's forward progress along the shared path. |
| `drone_progress` | Dict of each drone's latest progress, read for departure-based flow control. |

To actually run the swarm end-to-end you would supply your own coordination server that
exposes this interface. The FYP report (`docs/FYP_report.pdf`, §3.5.1–3.5.2) describes
the swarm server and `MarkerClient` design in more detail.
