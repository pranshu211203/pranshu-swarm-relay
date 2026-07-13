@echo off
setlocal enabledelayedexpansion

REM ===== CONFIG =====
set USER=telloswarm
set PASS=telloswarm

REM ===== PI IP LIST =====
set IPS=192.168.0.122 192.168.0.123 192.168.0.124 192.168.0.125 192.168.0.126 192.168.0.127

echo =========================================
echo        SCORPION C RPi CHECK
echo =========================================

for %%i in (%IPS%) do (
    echo.
    echo ===== %%i =====
    plink -ssh %USER%@%%i -pw %PASS% -batch ^
    "echo SSID: & nmcli -t -f ACTIVE,SSID dev wifi | findstr \"^yes:\" & echo State: & nmcli -t -f DEVICE,STATE device status | findstr \"wlan0\""
)

echo.
echo Done.
pause
