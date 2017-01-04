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

# default parameters
remote_root = '/home/odroid/orocos'
current_dir = os.path.dirname(os.path.realpath(__file__))
local_root = os.path.join(current_dir, 'orocos/emperor')
username = 'odroid'
password = 'odroid'

# hosts = ['kurt', 'krist', 'dave']
# hosts = ['kurt', 'krist', 'dave']
hosts = ['kurt']
# hosts = ['dave']
# coop_hosts = hosts

# obstacle = 'dave'
obstacle = None

coop_hosts = [h for h in hosts if h != obstacle]
addresses = col.OrderedDict([('kurt', '192.168.11.121'), ('krist', '192.168.11.122'), ('dave', '192.168.11.120')])
indices = {key: index for index, key in enumerate(addresses.keys())}

# formation configuration
# configuration = col.OrderedDict([('kurt', [-0.2, -0.2]), ('krist', [-0.2, 0.2]), ('dave', [0.35, 0.0])])
configuration = col.OrderedDict([('kurt', [0., -0.2]), ('krist', [0., 0.2]), ('dave', [0.35, 0.0])])


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


def get_ip():
    arg = 'ip route list'
    p = subprocess.Popen(arg, shell=True, stdout=subprocess.PIPE)
    data = p.communicate()
    sdata = data[0].split()
    for k, word in enumerate(sdata):
        if (word == 'src'):
            if sdata[k+1].startswith('192.168.11'):
                return sdata[k+1]
    return '0.0.0.0'


def modify_emperor_config():
    # modify system-config
    file = os.path.join(
        current_dir, 'orocos/emperor/Configuration/system-config.cpf')
    tree = et.parse(file)
    root = tree.getroot()
    for elem in root.findall('struct'):
        if elem.attrib['name'] == 'trusted_hosts':
            for e in elem.findall('simple'):
                if e.attrib['name'] == 'emperor':
                    e.find('value').text = get_ip()
    f = open(file, 'w')
    f.seek(0)
    f.truncate()
    f.write('<?xml version="1.0" encoding="UTF-8"?>\n<!DOCTYPE properties SYSTEM "cpf.dtd">\n')
    tree.write(f)
    f.close()


def modify_host_config(host, distributed_mp=False):
    local_files, remote_files = [], []
    # modify system-config
    local_files.append(os.path.join(
        current_dir, 'orocos/ourbot/Configuration/system-config.cpf'))
    remote_files.append(os.path.join(
        remote_root, 'Configuration/system-config.cpf'))
    tree = et.parse(local_files[-1])
    root = tree.getroot()
    index = addresses.keys().index(host)
    for elem in root.findall('struct'):
        if elem.attrib['name'] == 'trusted_hosts':
            for e in elem.findall('simple'):
                if e.attrib['name'] == 'emperor':
                    e.find('value').text = get_ip()
    for elem in root.findall('simple'):
        if elem.attrib['name'] == 'host':
            elem.find('value').text = host
        if elem.attrib['name'] == 'index':
            elem.find('value').text = str(index)
        if host == obstacle:
            if elem.attrib['name'] == 'obstacle_mode':
                elem.find('value').text = str(1)
        elif obstacle is not None:
            if elem.attrib['name'] == 'robobs_index':
                elem.find('value').text = str(addresses.keys().index(obstacle))
    if distributed_mp:
        N = len(coop_hosts)
        for elem in root.findall('simple'):
            if elem.attrib['name'] == 'distributed_mp':
                elem.find('value').text = 'true'
            if elem.attrib['name'] == 'motionplanning':
                elem.find('value').text = 'DistributedMotionPlanning'
        # set neighbors
        neighbors = [coop_hosts[(N+index+1) % N], coop_hosts[(N+index-1) % N]]
        for elem in root.findall('struct'):
            if elem.attrib['name'] == 'neighbors':
                for e in elem.findall('simple'):
                    if e.attrib['name'] == 'neighbor0':
                        e.find('value').text = addresses[neighbors[0]]
                    if e.attrib['name'] == 'neighbor1':
                        e.find('value').text = addresses[neighbors[1]]
            if elem.attrib['name'] == 'nghb_index':
                for e in elem.findall('simple'):
                    if e.attrib['name'] == 'neighbor0':
                        e.find('value').text = str(indices[neighbors[0]])
                    if e.attrib['name'] == 'neighbor1':
                        e.find('value').text = str(indices[neighbors[1]])
    file = open(local_files[-1]+'_', 'w')
    file.write('<?xml version="1.0" encoding="UTF-8"?>\n<!DOCTYPE properties SYSTEM "cpf.dtd">\n')
    tree.write(file)
    file.close()
    if distributed_mp:
        # modify motionplanning-config file
        local_files.append(os.path.join(
            current_dir, 'orocos/ourbot/Configuration/motionplanning-config.cpf'))
        remote_files.append(os.path.join(
            remote_root, 'Configuration/motionplanning-config.cpf'))
        tree = et.parse(local_files[-1])
        root = tree.getroot()
        for elem in root.findall('struct'):
            if elem.attrib['name'] == 'rel_pos_c':
                for e in elem.findall('simple'):
                    if e.attrib['name'] == 'Element0':
                        e.find('value').text = str(configuration[host][0])
                        # e.find('value').text = str(radius*np.cos(index*2*np.pi/N))
                    if e.attrib['name'] == 'Element1':
                        e.find('value').text = str(configuration[host][1])
                        # e.find('value').text = str(radius*np.sin(index*2*np.pi/N))
        file = open(local_files[-1]+'_', 'w')
        file.write('<?xml version="1.0" encoding="UTF-8"?>\n<!DOCTYPE properties SYSTEM "cpf.dtd">\n')
        tree.write(file)
        file.close()
    return [lf+'_' for lf in local_files], remote_files


def write_settings(distributed_mp):
    # modify emperor's config files
    modify_emperor_config()
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    for host in hosts:
        # send all deploy scripts and configuration files
        local_files, remote_files = [], []
        files = ['deploy.lua', 'deploy_fsm.lua']
        files += [('Configuration/'+ff) for ff in os.listdir(current_dir+'/orocos/ourbot/Configuration')]
        for file in files:
            local_files.append(os.path.join(current_dir+'/orocos/ourbot', file))
            remote_files.append(os.path.join(remote_root, file))
        # modify host's config files
        local_files_mod, remote_files_mod = modify_host_config(host, distributed_mp)
        # open ssh connection
        ssh.connect(addresses[host], username=username, password=password)
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
    distributed_mp = False if (len(coop_hosts) == 1) else True
    write_settings(distributed_mp)
    command = ['gnome-terminal']
    for host in hosts:
        address = addresses[host]
        command.extend(['--tab', '-e', '''
            bash -c '
            sshpass -p %s ssh %s@%s "
            cd %s
            rm reports.nc
            echo I am %s
            deployer-gnulinux -s run.ops
            "'
            ''' % (password, username, address, remote_root, host)
        ])
    command.extend(['--tab', '-e', '''
        bash -c '
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
