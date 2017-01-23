#!/usr/bin/python

"""
This script
- copies the source files from PC
- creates new components on the remote odroids if desired
Run ./write2ourbots.py --help for available options.

Ruben Van Parys - 2016
"""

import optparse
import os
import sys
import paramiko
import collections as col
import math
import errno
import pickle
import hashlib
import socket

# Default parameters
current_dir = os.path.dirname(os.path.realpath(__file__))
local_root = os.path.join(current_dir, 'orocos/ourbot')
other_local_dirs = []
remote_root = '/home/odroid/orocos/ourbot/'
username = 'odroid'
password = 'odroid'
hosts = col.OrderedDict()
hosts['kurt'] = '192.168.11.121'
hosts['krist'] = '192.168.11.122'
hosts['dave'] = '192.168.11.120'
ignore = []
exclude = ['build', 'include', 'bin', 'lib', 'obj', '.tb_history']


def create_component(ssh, component):
    print component
    cmd = 'cd ' + remote_root + ' && rosrun ocl orocreate-pkg ' + component
    stdin, stdout, stderr = ssh.exec_command(cmd)
    err = stderr.read()
    if err:
        print err


def add_to_ros_env(ssh, component):
    comp_path = os.path.join(remote_root, component)
    stdin, stdout, stderr = ssh.exec_command('echo y | rosws set ' + comp_path)
    err = stderr.read()
    if err:
        print err


def change_cmakelist(ssh, component):
    comp_path = os.path.join(remote_root, component)
    path = os.path.join(comp_path, 'CMakeLists.txt')
    ftp = ssh.open_sftp()
    f_in = ftp.file(path, 'r')
    c_in = f_in.readlines()
    c_out = []
    for line in c_in:
        if line.find('orocos_typegen_headers(') >= 0:
            c_out.append('#'+line)
        else:
            c_out.append(line)
    f_in.close()
    f_out = ftp.file(path, 'w')
    f_out.writelines(c_out)
    f_out.close()
    ftp.close()


def send_file(ftp, ssh, loc_file, rem_file):
    directory = os.path.dirname(rem_file)
    try:
        ftp.lstat(directory)
    except IOError, e:
        if e.errno == errno.ENOENT:
            stdin, stdout, stderr = ssh.exec_command('mkdir -p ' + directory)
            err = stderr.read()
            if err:
                print err
    try:
        ftp.put(loc_file, rem_file)
    except IOError:
        raise ValueError('Could not send ' + loc_file)


def send_files(ftp, ssh, loc_files, rem_files):
    n_blocks = min(50, len(loc_files))
    if (n_blocks == 0):
        print 'no files to send'
        return
    interval = int(math.ceil(len(loc_files)/n_blocks*1.))
    string = '['
    for k in range(len(loc_files)/interval):
        string += ' '
    string += ']'
    cnt = 1
    for lf, rf in zip(loc_files, rem_files):
        string2 = string
        send_file(ftp, ssh, lf, rf)
        for k in range(cnt/interval):
            string2 = string2[:k+1] + '=' + string2[k+2:]
        sys.stdout.flush()
        sys.stdout.write("\r"+string2)
        cnt += 1
    print ''


def detect_changes(loc_files, rem_files, host):
    _lf, _rf = [], []
    try:
        db = pickle.load(open('.db'))
    except IOError:
        db = {}
    if host not in db:
        db[host] = {}
    for lf, rf in zip(loc_files, rem_files):
        checksum = hashlib.md5(open(lf).read()).hexdigest()
        if db[host].get(lf, None) != checksum:
            db[host][lf] = checksum
            _lf.append(lf)
            _rf.append(rf)
    pickle.dump(db, open('.db', 'w'))
    if options.send_all:
        return loc_files, rem_files
    else:
        return _lf, _rf


def get_source_files(host):
    loc_files, rem_files = [], []
    local_files = [f for f in os.listdir(local_root)
                   if os.path.isfile(os.path.join(local_root, f))]
    for f in local_files:
        if f.endswith('.ops') or f.endswith('.lua'):
            loc_file = os.path.join(local_root, f)
            rem_file = os.path.join(remote_root, f)
            loc_files.append(loc_file)
            rem_files.append(rem_file)
    for loc_dir in local_folders:
        rem_dir = loc_dir.replace(os.path.dirname(loc_dir), remote_root)
        for dirpath, dirnames, filenames in os.walk(loc_dir):
            if sum(['/'+d in dirpath for d in exclude]) == 0:
                for f in filenames:
                    loc_file = os.path.join(dirpath, f)
                    rem_file = loc_file.replace(loc_dir, rem_dir)
                    loc_files.append(loc_file)
                    rem_files.append(rem_file)
    return detect_changes(loc_files, rem_files, host)


if __name__ == "__main__":
    usage = ('Usage: %prog [options]')
    op = optparse.OptionParser(usage=usage)
    op.add_option("-c", "--createcomponents", action="store_true",
                  dest="createcomponents", default=False,
                  help="create missing components")
    op.add_option("-a", "--all", action="store_true",
                  dest="send_all", default=False,
                  help="send all files")

    options, args = op.parse_args()

    # relevant local folders
    local_folders = [os.path.join(local_root, d) for d in os.listdir(local_root)
                     if (os.path.isdir(os.path.join(local_root, d)) and d not in ignore)] + other_local_dirs
    # split between components and non-components
    comp_dir = [c for c in local_folders if 'src' in os.listdir(c)]
    noncomp_dir = [d for d in local_folders if d not in comp_dir]

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    for host, address in hosts.items():
        try:
            ssh.connect(address, username=username, password=password, timeout=0.5)
        except socket.error:
            print 'Could not connect to %s' % host
            continue
        print 'Writing to %s' % host
        ftp = ssh.open_sftp()
        remote_dir = [d for d in ftp.listdir(remote_root) if 'd'
                      in str(ftp.lstat(os.path.join(remote_root, d))).split()[0]]

        # create unexisting components
        if options.createcomponents:
            print 'Creating unexisting components:'
            new_comp_dir = [c for c in comp_dir if os.path.basename(c) not in remote_dir]
            new_comp_name = [os.path.basename(c) for c in new_comp_dir]
            new_comp_parent = [os.path.dirname(c) for c in new_comp_dir]
            for comp in new_comp_name:
                create_component(ssh, comp)
            for comp in new_comp_name:
                add_to_ros_env(ssh, comp)
            for comp in new_comp_name:
                change_cmakelist(ssh, comp)
            for comp_name, comp_parent in zip(new_comp_name, new_comp_dir):
                dir_ = os.path.join(comp_parent, comp)
                for dirpath, dirnames, filenames in os.walk(os.path.join(dir_, 'src')):
                    new_dir = dirpath.replace(comp_parent, remote_root)
                    ssh.exec_command('mkdir ' + new_dir)
            for nc_dir in noncomp_dir:
                if os.path.basename(nc_dir) not in remote_dir:
                    dir_ = nc_dir.replace(os.path.dirname(nc_dir), remote_root)
                    ssh.exec_command('mkdir ' + dir_)
            ssh.exec_command('source ~/.bashrc')

        # sending source files
        print 'Sending source files'
        # get source files to send
        loc_files, rem_files = get_source_files(host)
        send_files(ftp, ssh, loc_files, rem_files)

        ftp.close()
        ssh.close()
    os.system('clear')
