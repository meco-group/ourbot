#!/bin/bash
#
#This file copies all component src files to the Odroids and builds the components (pass argument 'build')
#
## Adapt SOURCEDIR to the path where your ourbot components are written
## Adapt HOSTS to a list of multiple ip's for multiple Odroids
#
#Questions? ourBot@kuleuven.be

SOURCEDIR=/home/ruben/ourbot/orocos/ourbot/
OROCOSDIR=/home/odroid/orocos
USERNAME=odroid
HOSTS="192.168.10.248"
PASSWORD="odroid"

for HOSTNAME in ${HOSTS} ; do
	#Sync source files
	echo ""
	echo "Send src files to "$HOSTNAME
	echo "-----------------------------"
	sshpass -p $PASSWORD rsync -avz --include=*/ --include=*/src/** --include=*.ops --include=*.lua --include=Coordinator/*.lua --include=Configuration/*.cpf --exclude=*   -e ssh $SOURCEDIR ${USERNAME}@${HOSTNAME}:$OROCOSDIR
done

if [ "X$1" = "Xbuild" ]
then
	for HOSTNAME in ${HOSTS} ; do
		#Build components
		echo ""
		echo "Build components on "$HOSTNAME
		echo "-------------------------------"
		sshpass -p $PASSWORD ssh ${USERNAME}@${HOSTNAME} "
		cd $OROCOSDIR
		for d in */ ; do
			if [ !"X\$d" = "XCoordinator/" -a ! "X\$d" = "XConfiguration/" ]
			then
				cd $OROCOSDIR/\$d
				echo "Build "\$d
				make -j4
				cd ..
			fi
		done
		"
	done
fi
