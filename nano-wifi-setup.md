# setting up new wifi on Nano
Using nmcli. Note that if you choose to set network settings in `/etc/network/interfaces` you must verify that in `/etc/NetworkManager/NetworkManager.conf`, the lines reading:

`[ifupdown]
managed=false
`

are changed to true. Otherwise settings will be ignored.

## find wifi list 
shouldnt need this, but had to run before nmcli could see all avail networks 

`sudo iw dev wlan0 scan | grep SSID`

then 

`nmcli dev wifi `

worked.

## add new network

`sudo nmcli dev wifi connect NETGEAR88 password "livelywater854"`

## update network settings

`sudo nmcli con mod "NETGEAR88 1" ipv4.addresses "192.168.1.10/24" ipv4.gateway "192.168.1.1" ipv4.dns "8.8.8.8" ipv4.method manual`
