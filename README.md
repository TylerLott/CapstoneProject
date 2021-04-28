USU MAE Captstone Project - L3Harris Self-Leveling Platform 
===========================================================

# Table of Contents:

* [About the Team](#about-the-team)
	* [Members](#members)
* [About the Project](#about-the-project)
	* [Specifications](#specifications)
	* [Implementation](#implementation)
* [Installation](#installation)
 	* [Run on Boot](#run-on-boot)
	* [Raspberry Pi as Wifi Host](#raspberry-pi-as-wifi-host)
* [Using Software](#using-software)
	* [Physical Interface](#physical-interface)
		* [LCD Display](#lcd-display)
		* [Button Control](#button-control)
	* [Debug Interface](#debug-interface)
		* [Platform Interface](#platform-interface)
		* [User Input](#user-input)
		* [Terminal Output](#terminal-output)


# About the Team

text

## Members

<img src="https://github.com/TylerLott/CapstoneProject/blob/main/WebsiteFiles/images/Tyler.png" height="200"/> <img src="https://github.com/TylerLott/CapstoneProject/blob/main/WebsiteFiles/images/Ben.png" height="200"/> <img src="https://github.com/TylerLott/CapstoneProject/blob/main/WebsiteFiles/images/Thomas.png" height="200"/> <img src="https://github.com/TylerLott/CapstoneProject/blob/main/WebsiteFiles/images/Reed.png" height="200"/>

Tyler Lott* &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; Ben Allen &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; Thomas Lloyd &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; Reed Thurber

*code contributer


# About the Project

This project was a senior capstone project for Utah State Univerity sponsored by L3Harris. The goal of the project was to create a tripod antenna leveling system for legacy tripods. The design employed was to use three separate platforms under each foot, which would raise and lower to make the platform level.

## Specifications

- level to 0.05 degrees
- withstand wind load of 70mph with maximum dish diameter of 6ft
- adapt to tripods from 3ft to 8ft in diameter
- comsume less than 120W
- setup and level within 15 minutes

## Implementation

The electronics system consisted of a single board computer (raspberry pi) which was attached to 5 arduino microcontrollers.Three of these microcontrollers were attached to linear actuators on the plarform. The other two were attached to sensors which were attached between the platforms to give a relative angle. The microcontrollers relayed information back to the main computer, which send back the movement amounts. This was iterated until level. The raspberry pi also served as a wifi hotspot which served a web application to offer a debug interface with more fine control over the platforms. 

# Installation

### Run On Boot

To get the programs to execute on start up the following must be done

1. Open terminal

2. Enter 
	

	$sudo nano /etc/rc.local$


3. Below the line containing "fi" add the code


	sudo python /home/pi/Desktop/CapstoneProject/src/main.py &


4. Enter 

	$ sudo nano /home/pi/.bashrc
	
5. Add this code to the end

	sudo python /home/pi/Desktop/CapstoneProject/WebsiteFiles/websocketServer.py

Test by rebooting the Pi.

### Raspberry Pi as Wifi Host

To get the pi to host a webserver through it's own wifi network

1. run these

	$sudo apt-get install hostapd$
	$sudo apt-get install dnsmasq$

	$sudo systemctl stop hostapd$
	$sudo systemctl stop dnsmasq$

2. create this file 

	$sudo nano /etc/hostapd/hostapd.conf$


interface=wlan0
driver=nl80211
ssid=RPiHotSpot
hw_mode=g
channel=6
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=1234567890
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP


3. change this file 

	$ sudo nano /etc/default/hostapd$ 

uncomment the 
	#DAEMON_CONF=""

change to 
	 #DAEMON_CONF="/etc/default/hostapd"

4. change this file

	$ sudo nano /etc/dnsmasq.conf$

add this to the end:

#RPiHotspot config - No Intenet
interface=wlan0
domain-needed
bogus-priv
dhcp-range=192.168.50.150,192.168.50.200,255.255.255.0,12h

5. open this and change

	$sudo nano /etc/dhcpcd.conf$

add this to end:

nohook wpa_supplicant
interface wlan0
static ip_address=192.168.50.10/24
static routers=192.168.50.1

Done, see [this website](https://superuser.com/questions/1503862/raspberry-pi-4-hostapd-hotspot-not-visible) for a detailed overview

# Using Software

## Physical Interface

### LCD Display

### Button Control

## Debug Interface

### Platform Interface

### User Input

### Terminal Output
