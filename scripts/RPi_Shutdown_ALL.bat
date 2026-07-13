@echo off
start "RPi22" cmd /k ssh -t telloswarm@192.168.0.122 "sudo shutdown now; bash"
start "RPi23" cmd /k ssh -t telloswarm@192.168.0.123 "sudo shutdown now; bash"
start "RPi24" cmd /k ssh -t telloswarm@192.168.0.124 "sudo shutdown now; bash"
start "RPi25" cmd /k ssh -t telloswarm@192.168.0.125 "sudo shutdown now; bash"
start "RPi26" cmd /k ssh -t telloswarm@192.168.0.126 "sudo shutdown now; bash"
start "RPi27" cmd /k ssh -t telloswarm@192.168.0.127 "sudo shutdown now; bash"
start "RPi28" cmd /k ssh -t telloswarm@192.168.0.128 "sudo shutdown now; bash"
start "RPi29" cmd /k ssh -t telloswarm@192.168.0.129 "sudo shutdown now; bash"
start "RPi30" cmd /k ssh -t telloswarm@192.168.0.130 "sudo shutdown now; bash"
start "RPi31" cmd /k ssh -t telloswarm@192.168.0.131 "sudo shutdown now; bash"
start "RPi32" cmd /k ssh -t telloswarm@192.168.0.132 "sudo shutdown now; bash"

