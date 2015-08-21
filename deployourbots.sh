#!/bin/bash
#
#This script creates a Name Service at one Ourbot and deploys all the Ourbots by calling its `run.ops` script.
#
## Adapt HOSTS to a list of multiple ip's for multiple Odroids
## Adapt SERVER to set the ip address of the device which creates the Name Service.
#
#Questions? ourBot@kuleuven.be

OROCOSDIR=/home/odroid/orocos
USERNAME=odroid
HOSTS="192.168.0.2"
SERVER="192.168.0.2"	#The host that sets up the CORBA Name Service
PASSWORD="odroid"

for HOSTNAME in ${HOSTS} ; do
	echo "Deploying host "$HOSTNAME
	if [ "$HOSTNAME"="$SERVER" ]
	then
		# screen -A -m -d -S "$HOSTNAME" sshpass -p $PASSWORD ssh ${USERNAME}@${HOSTNAME} "
		sshpass -p $PASSWORD ssh ${USERNAME}@${HOSTNAME} "
		ulimit -r 10
		echo $PASSWORD | sudo -S killall omniNames
		echo $PASSWORD | sudo -S rm /var/lib/omniorb/*
		sudo omniNames -start &
		cd orocos
		deployer-corba-gnulinux run.ops
		"
	else
		sshpass -p $PASSWORD ssh ${USERNAME}@${HOSTNAME} "
		ulimit -r 10
		cd orocos
		deployer-corba-gnulinux -s run.ops
		"
	fi
done
