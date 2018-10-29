## Deploy

```bash
[Unit]
Description=HangoverHack
Requires=roscore.service
After=roscore.serivce

[Service]
EnvironmentFile=/lib/systemd/system/roscore.env
ExecStart=/home/pi/catkin_ws/src/four_from_the_hangover/scripts/hack.py
Restart=on-abort

[Install]
WantedBy=multi-user.target
```

```bash
[Unit]
Description=HangoverLenta
Requires=roscore.service
After=roscore.serivce

[Service]
EnvironmentFile=/lib/systemd/system/roscore.env
ExecStart=/home/pi/catkin_ws/src/four_from_the_hangover/scripts/lenta.py
Restart=on-abort

[Install]
WantedBy=multi-user.target
```
