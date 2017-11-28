#!/usr/bin/python

"""
This script deploys remote odroids and local pc.
Run ./deploy.py --help for available options.

Ruben Van Parys - 2016/2017
"""

import subprocess
import optparse
import os
import paramiko
import xml.etree.ElementTree as et
import collections as col
import errno
import sys
import math
import socket

# parameters
user = os.getenv('USER')
password = user
remote_root = os.path.join('/home/' + user, 'orocos/ourbot/')
current_dir = os.path.dirname(os.path.realpath(__file__))
local_root = os.path.join(current_dir, 'orocos/emperor')

robots = ['dave', 'kurt', 'krist']
eagles = ['eagle0', 'eagle1']
addresses = col.OrderedDict([('kurt', '192.168.11.121'),
                             ('krist', '192.168.11.120'),
                             ('dave', '192.168.11.122'),
                             ('eagle0', '192.168.11.139'),
                             ('eagle1', '192.168.11.123')])


def send_file(ftp, ssh, loc_file, rem_file):
    directory = os.path.dirname(rem_file)
    try:
        ftp.lstat(directory)
    except IOError, e:
        if e.errno == errno.ENOENT:
            stdin, stdout, stderr = ssh.exec_command('mkdir ' + directory)
    try:
        ftp.put(loc_file, rem_file)
    except:
        raise ValueError('Could not send ' + loc_file)


def send_files(ftp, ssh, loc_files, rem_files, fancy_print=False):
    n_blocks = len(loc_files)
    interval = int(math.ceil(len(loc_files)/n_blocks*1.))
    if fancy_print:
        string = '['
        for k in range(len(loc_files)/interval):
            string += ' '
        string += ']'
        cnt = 0
    for lf, rf in zip(loc_files, rem_files):
        send_file(ftp, ssh, lf, rf)
        if fancy_print:
            string2 = string
            for k in range(cnt/interval):
                string2 = string2[:k+1] + '=' + string2[k+2:]
            sys.stdout.flush()
            sys.stdout.write("\r"+string2)
            cnt += 1
    if fancy_print:
        print ''


def modify_robot_config(robot, flexonomy=False):
    local_files, remote_files = [], []
    # modify system-config
    local_files.append(os.path.join(
        current_dir, 'orocos/ourbot/Configuration/system-config.cpf'))
    remote_files.append(os.path.join(
        remote_root, 'Configuration/system-config.cpf'))
    tree = et.parse(local_files[-1])
    root = tree.getroot()
    for elem in root.findall('simple'):
        if elem.attrib['name'] == 'host':
            elem.find('value').text = robot
        if elem.attrib['name'] == 'flexonomy':
            elem.find('value').text = '1' if flexonomy else '0'
    file = open(local_files[-1]+'_', 'w')
    file.write('<?xml version="1.0" encoding="UTF-8"?>\n<!DOCTYPE properties SYSTEM "cpf.dtd">\n')
    tree.write(file)
    file.close()
    return [lf+'_' for lf in local_files], remote_files


def write_settings(flexonomy):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    robots_tmp = robots[:]
    for robot in robots_tmp:
        # send all deploy scripts and configuration files
        local_files, remote_files = [], []
        files = ['deploy.lua', 'deploy_fsm.lua']
        files += [('Configuration/'+ff) for ff in os.listdir(current_dir+'/orocos/ourbot/Configuration')]
        files += [('Coordinator/'+ff) for ff in os.listdir(current_dir+'/orocos/ourbot/Coordinator')]
        for file in files:
            local_files.append(os.path.join(current_dir+'/orocos/ourbot', file))
            remote_files.append(os.path.join(remote_root, file))
        # modify robot's config files
        local_files_mod, remote_files_mod = modify_robot_config(robot, flexonomy)
        # open ssh connection
        try:
            ssh.connect(addresses[robot], username=user, password=password, timeout=0.5)
        except socket.error:
            print 'Could not connect to %s' % robot
            robots.remove(robot)
            continue
        ftp = ssh.open_sftp()
        # send files
        send_files(ftp, ssh, local_files+local_files_mod, remote_files+remote_files_mod)
        # remove modified files
        for lfa in local_files_mod:
            os.remove(lfa)
        # close ssh connection
        ftp.close()
        ssh.close()


def deploy(robots, eagles=None, flexonomy=False):
    write_settings(flexonomy)
    # if len(robots) == 0:
    #     return
    command = ['gnome-terminal']
    if eagles is not None:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        for eagle in eagles:
            try:
                ssh.connect(addresses[eagle], username='odroid', password='odroid', timeout=0.5)
            except socket.error:
                print 'Could not connect to %s' % eagle
                eagles.remove(eagle)
                continue
        for eagle in eagles:
            address = addresses[eagle]
            command.extend(['--tab', '-e', '''
                bash -c '
                sshpass -p %s ssh %s@%s "
                killall -9 EagleTransmitter
                cd /home/odroid/ProjectEagle/eagle/build
                echo I am %s
                ./bin/EagleTransmitter %s
                "'
                ''' % ('odroid', 'odroid', address, eagle, eagle)])
    for robot in robots:
        address = addresses[robot]
        command.extend(['--tab', '-e', '''
            bash -c '
            sshpass -p %s ssh %s@%s "
            killall deployer-gnulinux
            source /home/%s/orocos/setup.bash
            cd %s
            rm reports.nc
            echo I am %s
            deployer-gnulinux -s run.ops
            "'
            ''' % (password, user, address, user, remote_root, robot)
        ])
    command.extend(['--tab', '-e', '''
        bash -c '
        killall deployer-gnulinux
        cd %s
        rm reports.nc
        echo I am emperor
        deployer-gnulinux -s run.ops
        '
        ''' % (local_root)
    ])
    subprocess.call(command)

if __name__ == "__main__":
    usage = ("Usage: %prog [options]")
    op = optparse.OptionParser(usage=usage)
    op.add_option("-e", "--eagles", action="store_true",
                  dest="deploy_eagles", default=False,
                  help="deploy eagles")
    op.add_option("-f", "--flexonomy", action="store_true",
                  dest="flexonomy", default=False,
                  help="switch immediately to flexonomy state")
    options, args = op.parse_args()
    if options.deploy_eagles:
        deploy(robots, eagles, options.flexonomy)
    else:
        deploy(robots, [], options.flexonomy)
