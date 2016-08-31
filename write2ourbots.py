#!/usr/bin/python

"""
This script
- creates new components on the remote odroids
- copies the source files from PC
- build the components if desired
Run ./write2ourbots.py --help for available options.

Ruben Van Parys - 2015
"""

import optparse
import os
import sys
import paramiko
import collections as col
import math
import errno

# Default parameters
current_dir = os.path.dirname(os.path.realpath(__file__))
local_root = os.path.join(current_dir, 'orocos/ourbot')
remote_root = '/home/odroid/orocos'
username = 'odroid'
password = 'odroid'
hosts = col.OrderedDict()
# hosts['dave'] = '192.168.11.120'
hosts['kurt'] = '192.168.11.121'
hosts['krist'] = '192.168.11.122'
ignore = ['TestCorba', 'VelocityCommandInterface', 'SPIMaster']
build_list = []


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
    n_blocks = 50
    interval = int(math.ceil(len(loc_files)/n_blocks*1.))
    string = '['
    for k in range(len(loc_files)/interval):
        string += ' '
    string += ']'
    cnt = 0
    for lf, rf in zip(loc_files, rem_files):
        string2 = string
        send_file(ftp, ssh, lf, rf)
        for k in range(cnt/interval):
            string2 = string2[:k+1] + '=' + string2[k+2:]
        sys.stdout.flush()
        sys.stdout.write("\r"+string2)
        cnt += 1
    print ''


def mp_adaptations(ftp, ssh):
    local_files = []
    remote_files = []
    # adapt CMakeLists.txt
    local_files.append(os.path.join(
        current_dir, 'orocos/ourbot/MotionPlanning/CMakeLists.txt'))
    remote_files.append(os.path.join(
        remote_root, 'MotionPlanning/CMakeLists.txt'))
    f1 = open(local_files[-1], 'r')
    body = f1.read()
    f1.close()
    for repl in ['include_directories', 'link_directories']:
        while(body.find(repl) > 0):
            index = body.find(repl)
            string = body[index:].split(')')[0]+')\n'
            body = body.replace(string, '')
    part1, part2 = body[:index], body[index:]
    body = part1 + '\n'
    body += 'link_directories('
    body += os.path.join(
        remote_root + '/MotionPlanning/src/MotionPlanning/Toolbox/bin/') + ')\n'
    body += 'link_directories('
    body += os.path.join(
        remote_root + '/MotionPlanning/src/DistributedMotionPlanning/Toolbox/bin/') + ')\n'
    body += 'include_directories('
    body += os.path.join(
        remote_root + '/MotionPlanning/src/MotionPlanning/Toolbox/src/') + ')\n'
    body += 'include_directories('
    body += os.path.join(
        remote_root + '/MotionPlanning/src/DistributedMotionPlanning/Toolbox/src/') + ')\n'

    body += part2
    f2 = open(local_files[-1]+'_', 'w')
    f2.write(body)
    f2.close()
    # adapt Makefiles
    for mp_type in ['MotionPlanning', 'DistributedMotionPlanning']:
        local_files.append(os.path.join(
            current_dir, ('orocos/ourbot/MotionPlanning/src/' +
                          mp_type + '/Toolbox/Makefile')))
        remote_files.append(os.path.join(
            remote_root, 'MotionPlanning/src/'+mp_type+'/Toolbox/Makefile'))
        f1 = open(local_files[-1], 'r')
        body = f1.read()
        f1.close()
        index = body.find('CASADILIB =')
        string = body[index:].split('\n')[0]
        body = body.replace(string, 'CASADILIB = /usr/local/lib/')
        index = body.find('CASADIINC =')
        string = body[index:].split('\n')[0]
        body = body.replace(string, 'CASADIINC = /usr/local/include/casadi/')
        index = body.find('CASADIOBJ =')
        string = body[index:].split('\n')[0]
        body = body.replace(
            string, 'CASADIOBJ = ' + os.path.join(
                remote_root, 'MotionPlanning/src/'+mp_type+'/Toolbox/bin/'))
        f2 = open(local_files[-1]+'_', 'w')
        f2.write(body)
        f2.close()
    # send files
    for lf, rf in zip(local_files, remote_files):
        send_file(ftp, ssh, lf+'_', rf)
        os.remove(lf+'_')


if __name__ == "__main__":
    usage = ('Usage: %prog [options]')
    op = optparse.OptionParser(usage=usage)
    op.add_option("-b", "--build", action="store_true", dest="build",
                  default=False, help="build all components")
    op.add_option("-l", "--buildlist", dest="build_list",
                  default=build_list, help="list of components to build")
    op.add_option("-c", "--createcomponents", action="store_true",
                  dest="createcomponents", default=False,
                  help="create missing components")

    options, args = op.parse_args()

    local_files = [f for f in os.listdir(local_root)
                   if os.path.isfile(os.path.join(local_root, f))]
    local_dir = [d for d in os.listdir(local_root)
                 if (os.path.isdir(os.path.join(local_root, d)) and
                     d not in ignore)]
    components = [c for c in local_dir
                  if 'src' in os.listdir(os.path.join(local_root, c))]
    comp_dir = [d for d in local_dir if d not in components]

    build_list = options.build_list
    if options.build:
        build_list = components

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    for host, address in hosts.items():
        print 'Host %s' % host
        ssh.connect(address, username=username, password=password)
        ftp = ssh.open_sftp()
        remote_dir = [d for d in ftp.listdir(remote_root) if 'd'
                      in str(ftp.lstat(os.path.join(remote_root, d))).split()[0]]

        # create unexisting components
        if options.createcomponents:
            print 'Creating unexisting components:'
            new_components = [c for c in components if c not in remote_dir]
            for comp in new_components:
                create_component(ssh, comp)
            for comp in new_components:
                add_to_ros_env(ssh, comp)
            for comp in new_components:
                change_cmakelist(ssh, comp)
            for comp in new_components:
                dir_ = os.path.join(local_root, comp)
                for dirpath, dirnames, filenames in os.walk(os.path.join(dir_, 'src')):
                    new_dir = dirpath.replace(local_root, remote_root)
                    ssh.exec_command('mkdir ' + new_dir)
            for c_dir in comp_dir:
                if c_dir not in remote_dir:
                    dir_ = os.path.join(remote_root, c_dir)
                    ssh.exec_command('mkdir ' + dir_)
            ssh.exec_command('source ~/.bashrc')


        # sending source files
        print 'Sending source files'
        loc_files = []
        rem_files = []
        for f in local_files:
            if f.endswith('.ops') or f.endswith('.lua'):
                loc_file = os.path.join(local_root, f)
                rem_file = os.path.join(remote_root, f)
                loc_files.append(loc_file)
                rem_files.append(rem_file)
        for d in local_dir:
            loc_dir = os.path.join(local_root, d)
            rem_dir = os.path.join(remote_root, d)
            if d in components:
                loc_dir = os.path.join(loc_dir, 'src')
                rem_dir = os.path.join(rem_dir, 'src')
            for dirpath, dirnames, filenames in os.walk(loc_dir):
                if not (dirpath.endswith('bin') or dirpath.endswith('obj')):
                    for f in filenames:
                        loc_file = os.path.join(dirpath, f)
                        rem_file = loc_file.replace(local_root, remote_root)
                        loc_files.append(loc_file)
                        rem_files.append(rem_file)
        send_files(ftp, ssh, loc_files, rem_files)

        # adapt stuff for MotionPlanning component
        mp_adaptations(ftp, ssh)

        # build components
        if options.build:
            if options.createcomponents:
                raise ValueError('''Before you can build you should first copy
                                    your local environment variables to
                                    ~/.ssh/environment''')
            print '\n'
            print 'Building components:'
            ssh.exec_command('source ~/.bashrc')
            for comp in components:
                print comp
                comp_path = os.path.join(remote_root, comp)
                stdin, stdout, stderr = ssh.exec_command(('cd ' + comp_path +
                                                          '/  && make -j4'))
                print stdout.read()
                err = stderr.read()
                if err:
                    print err
        ftp.close()
        ssh.close()
    os.system('clear')
