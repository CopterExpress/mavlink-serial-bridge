[Unit]
Description=MAVLink serial to UDP bridge for %I
After=network-online.target
Requires=network-online.target
PartOf=network-online.target
Documentation=https://github.com/CopterExpress/mavlink-serial-bridge

[Service]
Type=simple
WorkingDirectory=/etc/mavlink-serial-bridge/
ExecStart=@CMAKE_INSTALL_PREFIX@/bin/mavlink-serial-bridge /etc/mavlink-serial-bridge/%i.conf 
RestartSec=2s
Restart=on-failure

[Install]
WantedBy=multi-user.target
