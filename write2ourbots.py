#!/usr/bin/python

# This script create new components on the ourbots, copies the source codes from PC and build the components.
# This done based on the directory structure on the local PC (starting from local_root).
# Run ./write2ourbots.py --help for available options.

# Usage: ./write2ourbots.py [options] arg
# Arguments:
#   1: local root folder of orocos components
#   2: remote root folder of orocos components
#  if arguments are not provided, default values are used:
#   1: current directory/orocos/ourbot
#   2; /home/odroid/orocos

# Options:
#   -b, --build: build all components
#   -n, --buildnew: build new created components
#   -l, --buildlist: list of components to build
#   -u, --username: ssh username
#   -p, --password: ssh password
#   -c, --createcomponents: create missing components
#
# Ruben Van Parys - 2015

import optparse, sys, os, paramiko, string

# Default parameters
current_dir = os.path.dirname(os.path.realpath(__file__))
local_root  = os.path.join(current_dir,'orocos/ourbot')
remote_root = '/home/odroid/orocos'
username    = 'odroid'
password    = 'odroid'
hosts       = ['192.168.10.248']
ignore      = ['TestCorba']
build_list  = []

def createComponent(ssh, component):
    comp_path = os.path.join(remote_root, component)

    print ('Creating component %s ...') % (component)
    cmd = 'cd ' + remote_root + ' && rosrun ocl orocreate-pkg '+ component
    ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command(cmd)
    print ssh_stdout.read()
    print 'DONE'
    print 'Add %s to ROS environment ...' % (component),
    ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command('rosws set ' + comp_path)
    ssh_stdin.write('y\n')
    ssh_stdin.flush()
    print 'DONE'
    ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command('source ~/.bashrc')

    print ('Changing CMakeListst.txt ...'),
    path    = os.path.join(comp_path, 'CMakeLists.txt')
    ftp     = ssh.open_sftp()
    f_in    = ftp.file(path, 'r')
    c_in    = f_in.readlines()
    c_out   = []
    for line in c_in:
        if line.find('orocos_typegen_headers(') >= 0:
            c_out.append('#'+line)
        else:
            c_out.append(line)
    f_in.close()
    f_out   = ftp.file(path, 'w')
    f_out.writelines(c_out)
    f_out.close()
    ftp.close()
    print 'DONE\n'

def send(ftp, loc_file, rem_file):
    print loc_file
    ftp.put(loc_file, rem_file)

if __name__ == "__main__":
    usage = ("Usage: %prog [options] arg. There are 2 arguments: local root path & remote root path. It can be left blank to use default values.")
    op = optparse.OptionParser(usage=usage)
    op.add_option("-b", "--build", action = "store_true", dest = "build", default = False, help = "build all components")
    op.add_option("-n", "--buildnew", action = "store_true", dest = "build_new", default = False, help = "build new created components")
    op.add_option("-l", "--buildlist", dest = "build_list", default = build_list, help = "list of components to build")
    op.add_option("-u", "--username", dest = "username", default = username, help = "ssh username")
    op.add_option("-p", "--password", dest = "password", default = password, help = "ssh password")
    op.add_option("-c", "--createcomponents", action = "store_true", dest = "createcomponents", default = True, help = "create missing components")

    options, args   = op.parse_args()

    if len(args) == 2:
        local_root  = args(0)
        remote_root = args(1)

    local_files     = [ f for f in os.listdir(local_root) if os.path.isfile(os.path.join(local_root,f)) ]
    local_dir       = [ d for d in os.listdir(local_root) if os.path.isdir(os.path.join(local_root,d)) and not d in ignore ]
    components      = [ c for c in local_dir if 'src' in os.listdir(os.path.join(local_root,c))]

    username        = options.username
    password        = options.password

    build_list      = options.build_list
    if options.build_new:
        build_list = [comp for comp in components if not comp in remote_dir]
    if options.build:
        build_list = components

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy( paramiko.AutoAddPolicy() )

    for host in hosts:
        print 'Host %s ...\n' % host
        ssh.connect(host, username = username, password = password)
        ftp = ssh.open_sftp()
        remote_dir   = [d for d in ftp.listdir(remote_root) if 'd' in str(ftp.lstat(os.path.join(remote_root,d))).split()[0]]

        # Create unexisting components
        if options.createcomponents:
            for comp in components:
                if not comp in remote_dir:
                    createComponent(ssh, comp)
                    dir_ = os.path.join(local_root,comp)
                    for dirpath, dirnames, filenames in os.walk(os.path.join(dir_,'src')):
                        new_dir = dirpath.replace(local_root,remote_root)
                        ssh.exec_command('mkdir ' + new_dir)

        # Sending source files
        print 'Sending source files:'

        for f in local_files:
            if f.endswith('.ops') or f.endswith('.lua'):
                loc_file = os.path.join(local_root, f)
                rem_file = os.path.join(remote_root, f)
                send(ftp, loc_file, rem_file)

        for d in local_dir:
            loc_dir = os.path.join(local_root,d)
            rem_dir = os.path.join(remote_root,d)
            if d in components:
                loc_dir = os.path.join(loc_dir,'src')
                rem_dir = os.path.join(rem_dir,'src')
            for dirpath, dirnames, filenames in os.walk(loc_dir):
                for f in filenames:
                    loc_file = os.path.join(dirpath,f)
                    rem_file = loc_file.replace(local_root, remote_root)
                    send(ftp, loc_file, rem_file)

        # Building components
        if options.build:
            print '\n'
            for comp in components:
                comp_path = os.path.join(remote_root,comp)
                print 'Building component %s ...\n' % (comp)
                ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command('cd ' + comp_path +'/  && make -j4')
                print 'Warning and Errors: \n %s' % (ssh_stderr.read())

        ftp.close()
        ssh.close()
