@echo off
setlocal enabledelayedexpansion

REM ===== CONFIG =====
set USER=telloswarm
set PASS=telloswarm

REM ===== PI IP LIST =====
set IPS=192.168.0.128 192.168.0.129 192.168.0.130 192.168.0.131 192.168.0.132

echo =========================================
echo        SCORPION 4 RPi CHECK
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
