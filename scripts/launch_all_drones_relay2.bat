@echo off
echo Launching all 6 drone scripts...
echo.

cd C:\safmc_26\UWBPathPlanningGUI\KNOWN_11_MARCH

REM start "Drone 19" cmd /k python Known_uwb_pranshu_19.py
REM start "Drone 20" cmd /k python Known_uwb_pranshu_20.py
REM start "Drone 21" cmd /k python Known_uwb_pranshu_21.py
start "Drone 22" cmd /k python Known_uwb_pranshu_22.py
start "Drone 23" cmd /k python Known_uwb_pranshu_23.py
start "Drone 24" cmd /k python Known_uwb_pranshu_24.py
start "Drone 25" cmd /k python Known_uwb_pranshu_25.py
start "Drone 14" cmd /k python Known_uwb_pranshu_14.py
start "Drone 27" cmd /k python Known_uwb_pranshu_27.py
REM start "Drone 28" cmd /k python Known_uwb_pranshu_28.py
REM start "Drone 29" cmd /k python Known_uwb_pranshu_29.py

echo All 6 drones launched.
pause
