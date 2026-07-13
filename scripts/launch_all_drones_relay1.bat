@echo off
echo Launching all 5 drone scripts...
echo.

cd C:\safmc_26\UWBPathPlanningGUI\KNOWN_11_MARCH

start "Drone 19" cmd /k python Known_uwb_pranshu_19.py
start "Drone 20" cmd /k python Known_uwb_pranshu_20.py
start "Drone 21" cmd /k python Known_uwb_pranshu_21.py
REM start "Drone 22" cmd /k python Known_uwb_pranshu_22.py
REM start "Drone 23" cmd /k python Known_uwb_pranshu_23.py
REM start "Drone 24" cmd /k python Known_uwb_pranshu_24.py
REM start "Drone 25" cmd /k python Known_uwb_pranshu_25.py
REM start "Drone 26" cmd /k python Known_uwb_pranshu_26.py
REM start "Drone 27" cmd /k python Known_uwb_pranshu_27.py
start "Drone 28" cmd /k python Known_uwb_pranshu_28.py
start "Drone 29" cmd /k python Known_uwb_pranshu_29.py

echo All 5 drones launched.
pause
