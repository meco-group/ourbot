#!/usr/bin/python

# This script
# - makes 1 odroid omniorb server
# - deploys ourbots
# - removes report.nc file on emperor
# - deploys emperor after 10s

# Run ./deploy.py --help for available options.

# Usage: ./deploy.py [options]
#
# Options:
#   -u, --username: ssh username
#   -p, --password: ssh password
#   -s, --server: host serving the naming service
#
# Ruben Van Parys - 2015

import subprocess
import optparse
import os

# Default parameters
remote_root = '/home/odroid/orocos'
current_dir = os.path.dirname(os.path.realpath(__file__))
local_root = os.path.join(current_dir, 'orocos/emperor')
username = 'odroid'
password = 'odroid'
hosts = ['192.168.11.121']
server = hosts[0]
hostnames = {hosts[0]: 'dave'}

if __name__ == "__main__":
    usage = ("Usage: %prog [options]")
    op = optparse.OptionParser(usage=usage)
    op.add_option("-u", "--username", dest="username",
                  default=username, help="ssh username")
    op.add_option("-p", "--password", dest="password",
                  default=password, help="ssh password")
    op.add_option("-s", "--server", dest="server",
                  default=server, help="host serving the naming service")

    options, args = op.parse_args()

    username = options.username
    password = options.password
    server = options.server

    command = ['gnome-terminal']
    # deploy hosts
    for host in hosts:
        if host == server:
            command.extend(['--tab', '-t', hostnames[host], '-e', '''
                bash -c '
                sshpass -p %s ssh %s@%s "
                ulimit -r 10
                echo %s | sudo -S killall omniNames
                echo %s | sudo -S rm /var/lib/omniorb/*
                sudo omniNames -start &
                cd %s
                deployer-corba-gnulinux run.ops
                "'
                ''' % (password, username, host,
                       password, password, remote_root)
                ])
        else:
            command.extend(['--tab', '-t', hostnames[host], '-e', '''
                bash -c '
                sshpass -p %s ssh %s@%s "
                ulimit -r 10
                cd %s
                deployer-corba-gnulinux run.ops
                "'
                ''' % (password, username, host, remote_root)
                ])
    # deploy emperor
    command.extend(['--tab', '-t', 'emperor', '-e', '''
        bash -c '
        cd %s
        rm reports.nc
        ulimit -r 10
        sleep 4
        deployer-corba-gnulinux run.ops
        '
        ''' % (local_root)
        ])

    subprocess.call(command)
