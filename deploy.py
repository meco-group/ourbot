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
import paramiko
import xml.etree.ElementTree as et
import collections as col
import numpy as np

# Default parameters
remote_root = '/home/odroid/orocos'
current_dir = os.path.dirname(os.path.realpath(__file__))
local_root = os.path.join(current_dir, 'orocos/emperor')
username = 'odroid'
password = 'odroid'

hosts = col.OrderedDict()
# hosts['dave'] = '192.168.11.120'
hosts['kurt'] = '192.168.11.121'
hosts['krist'] = '192.168.11.122'
server = 'kurt'


def send_file(ftp, loc_file, rem_file):
    try:
        ftp.put(loc_file, rem_file)
    except:
        raise ValueError('Could not send ' + loc_file + '!')


def distributed_mp_adaptations(index, N, address):
    local_files = []
    remote_files = []
    # adapt system-config file
    local_files.append(os.path.join(current_dir,
                                    'orocos/ourbot/Configuration/system-config.cpf'))
    remote_files.append(os.path.join(remote_root,
                                     'Configuration/system-config.cpf'))
    tree = et.parse(local_files[-1])
    root = tree.getroot()
    for elem in root.findall('simple'):
        if elem.attrib['name'] == 'distributed_mp':
            elem.find('value').text = 'true'
        if elem.attrib['name'] == 'index':
            elem.find('value').text = str(index)
        if elem.attrib['name'] == 'motionplanning':
            elem.find('value').text = 'DistributedMotionPlanning'
    for elem in root.findall('struct'):
        if elem.attrib['name'] == 'neighbors':
            for e in elem.findall('simple'):
                if e.attrib['name'] == 'Element0':
                    e.find('value').text = str((N+index+1) % N)
                if e.attrib['name'] == 'Element1':
                    e.find('value').text = str((N+index-1) % N)
    file = open(local_files[-1]+'_', 'w')
    file.write(
        '<?xml version="1.0" encoding="UTF-8"?>\n<!DOCTYPE properties SYSTEM "cpf.dtd">\n')
    tree.write(file)
    file.close()
    # adapt motionplanning-config file
    radius = 0.2
    local_files.append(
        os.path.join(current_dir, 'orocos/ourbot/Configuration/motionplanning-config.cpf'))
    remote_files.append(os.path.join(remote_root, 'Configuration/motionplanning-config.cpf'))
    tree = et.parse(local_files[-1])
    root = tree.getroot()
    for elem in root.findall('struct'):
        if elem.attrib['name'] == 'rel_pos_c':
            for e in elem.findall('simple'):
                if e.attrib['name'] == 'Element0':
                    e.find('value').text = str(radius*np.cos(index*2*np.pi/N))
                if e.attrib['name'] == 'Element1':
                    e.find('value').text = str(radius*np.sin(index*2*np.pi/N))
    file = open(local_files[-1]+'_', 'w')
    file.write(
        '<?xml version="1.0" encoding="UTF-8"?>\n<!DOCTYPE properties SYSTEM "cpf.dtd">\n')
    tree.write(file)
    file.close()
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(address, username=username, password=password)
    ftp = ssh.open_sftp()
    for lf, rf in zip(local_files, remote_files):
        send_file(ftp, lf+'_', rf)
        os.remove(lf+'_')
    ftp.close()
    ssh.close()


def deploy_blind():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    for host, address in hosts.items():
        print 'deploying ' + host + '...'
        ssh.connect(address, username=username, password=password)
        commands = []
        commands.append('ulimit -r 10')
        if host == server:
            commands.append('echo %s | sudo -S killall omniNames' % password)
            commands.append('echo %s | sudo -S rm /var/lib/omniorb/*' % password)
            commands.append('sudo omniNames -start &')
        commands.append('killall deployer-corba-gnulinux')
        commands.append('rm ' + os.path.join(remote_root, 'reports.nc'))
        commands.append('deployer-corba-gnulinux ' + os.path.join(remote_root, 'run.ops &'))
        for cmd in commands:
            print cmd
            stdin, stdout, stderr = ssh.exec_command(cmd)
            # print stdout.read()
            err = stderr.read()
            if err:
                print err
        ssh.close()
    command = ['gnome-terminal', '-e', '''
        bash -c '
        cd %s
        rm reports.nc
        ulimit -r 10
        sleep 8
        echo I am emperor
        deployer-corba-gnulinux run.ops
        '
        ''' % (local_root)
    ]
    subprocess.call(command)


def deploy():
    command = ['gnome-terminal']
    for host, address in hosts.items():
        if host == server:
            command.extend(['--tab', '-e', '''
                bash -c '
                sshpass -p %s ssh %s@%s "
                ulimit -r 10
                echo %s | sudo -S killall omniNames
                echo %s | sudo -S rm /var/lib/omniorb/*
                sudo omniNames -start &
                cd %s
                rm reports.nc
                echo I am %s
                deployer-corba-gnulinux run.ops
                "'
                ''' % (password, username, address,
                       password, password, remote_root, host)
            ])
        else:
            command.extend(['echo This is "' + host + '"', '--tab', '-e', '''
                bash -c '
                sshpass -p %s ssh %s@%s "
                ulimit -r 10
                cd %s
                rm reports.nc
                echo I am %s
                deployer-corba-gnulinux run.ops
                "'
                ''' % (password, username, address, remote_root, host)
            ])
    command.extend(['--tab', '-e', '''
        bash -c '
        cd %s
        rm reports.nc
        ulimit -r 10
        sleep 8
        echo I am emperor
        deployer-corba-gnulinux run.ops
        '
        ''' % (local_root)
    ])
    subprocess.call(command)


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

    distributed = False if (len(hosts) == 1) else True
    if distributed:
        for index, address in enumerate(hosts.values()):
            distributed_mp_adaptations(index, len(hosts.values()), address)

    command = ['gnome-terminal']
    # deploy hosts
    deploy()
