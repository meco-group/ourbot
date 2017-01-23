#!/usr/bin/python

"""
This script deploys remote odroids and local pc.
Run ./deploy.py --help for available options.

Ruben Van Parys - 2016
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

# default parameters
remote_root = '/home/odroid/orocos/ourbot/'
# remote_root = os.path.join(os.getenv('HOME'), 'orocos/ourbot/')
current_dir = os.path.dirname(os.path.realpath(__file__))
local_root = os.path.join(current_dir, 'orocos/emperor')
# username = os.getenv('USER')
# password = os.getenv('USER')
username = 'odroid'
password = 'odroid'

hosts = ['kurt', 'krist', 'dave']
# hosts = ['kurt']# obstacle = 'dave'
obstacle = None

coop_hosts = [h for h in hosts if h != obstacle]
addresses = col.OrderedDict([('kurt', '192.168.11.121'), ('krist', '192.168.11.122'), ('dave', '192.168.11.120')])
indices = {key: index for index, key in enumerate(addresses.keys())}


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


def modify_host_config(host):
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
            elem.find('value').text = host
        if host == obstacle:
            if elem.attrib['name'] == 'obstacle_mode':
                elem.find('value').text = str(1)
    file = open(local_files[-1]+'_', 'w')
    file.write('<?xml version="1.0" encoding="UTF-8"?>\n<!DOCTYPE properties SYSTEM "cpf.dtd">\n')
    tree.write(file)
    file.close()
    return [lf+'_' for lf in local_files], remote_files


def write_settings():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    hosts_tmp = hosts[:]
    for host in hosts_tmp:
        # send all deploy scripts and configuration files
        local_files, remote_files = [], []
        files = ['deploy.lua', 'deploy_fsm.lua']
        files += [('Configuration/'+ff) for ff in os.listdir(current_dir+'/orocos/ourbot/Configuration')]
        files += [('Coordinator/'+ff) for ff in os.listdir(current_dir+'/orocos/ourbot/Coordinator')]
        for file in files:
            local_files.append(os.path.join(current_dir+'/orocos/ourbot', file))
            remote_files.append(os.path.join(remote_root, file))
        # modify host's config files
        local_files_mod, remote_files_mod = modify_host_config(host)
        # open ssh connection
        try:
            ssh.connect(addresses[host], username=username, password=password, timeout=0.5)
        except socket.error:
            print 'Could not connect to %s' % host
            hosts.remove(host)
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


def deploy(hosts):
    write_settings()
    if len(hosts) == 0:
        return
    command = ['gnome-terminal']
    for host in hosts:
        address = addresses[host]
        command.extend(['--tab', '-e', '''
            bash -c '
            sshpass -p %s ssh %s@%s "
            killall deployer-gnulinux
            cd %s
            rm reports.nc
            echo I am %s
            deployer-gnulinux -s run.ops
            "'
            ''' % (password, username, address, remote_root, host)
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
    deploy(hosts)
