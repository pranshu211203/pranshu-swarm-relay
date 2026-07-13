@echo off
start "RPi28" cmd /k ssh -t telloswarm@192.168.0.128 "sudo iwlist wlan0 scan | grep ESSID && sudo nmcli device wifi connect 'RMTT-TAG28' password 'telloswarm'; bash"
start "RPi29" cmd /k ssh -t telloswarm@192.168.0.129 "sudo iwlist wlan0 scan | grep ESSID && sudo nmcli device wifi connect 'RMTT-TAG29' password 'telloswarm'; bash"
start "RPi30" cmd /k ssh -t telloswarm@192.168.0.130 "sudo iwlist wlan0 scan | grep ESSID && sudo nmcli device wifi connect 'RMTT-TAG19' password 'telloswarm'; bash"
start "RPi31" cmd /k ssh -t telloswarm@192.168.0.131 "sudo iwlist wlan0 scan | grep ESSID && sudo nmcli device wifi connect 'RMTT-TAG20' password 'telloswarm'; bash"
start "RPi32" cmd /k ssh -t telloswarm@192.168.0.132 "sudo iwlist wlan0 scan | grep ESSID && sudo nmcli device wifi connect 'RMTT-TAG21' password 'telloswarm'; bash"