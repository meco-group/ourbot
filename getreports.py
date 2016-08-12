#!/usr/bin/python

# This script
# - pulls reports.nc data from odroids

# Run ./getreports.py --help for available options.

# Usage: ./getreports.py [options] arg
# Arguments:
#   1: local root folder of orocos components
#   2: remote root folder of orocos components
#  if arguments are not provided, default values are used:
#   1: current directory/orocos/ourbot
#   2: /home/odroid/orocos

# Options:
#   -u, --username: ssh username
#   -p, --password: ssh password
#

# Ruben Van Parys - 2016

import optparse, os, paramiko

# Default parameters
current_dir = os.path.dirname(os.path.realpath(__file__))
local_root = os.path.join(current_dir,'orocos/emperor')
remote_root = '/home/odroid/orocos'
username = 'odroid'
password = 'odroid'
hosts = ['192.168.11.121']


def get_file(ftp, rem_file, loc_file):
    ftp.get(rem_file, loc_file)

if __name__ == "__main__":
    usage = ('''Usage: %prog [options] arg. There are 2 arguments:
                local root path & remote root path. It can be left blank to use
                default values.''')
    op = optparse.OptionParser(usage=usage)
    op.add_option("-u", "--username", dest="username",
                  default=username, help="ssh username")
    op.add_option("-p", "--password", dest="password",
                  default=password, help="ssh password")

    options, args = op.parse_args()

    if len(args) == 2:
        local_root = args(0)
        remote_root = args(1)

    username = options.username
    password = options.password

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    for index, host in enumerate(hosts):
        print 'Host %s ...\n' % host
        ssh.connect(host, username=username, password=password)
        ftp = ssh.open_sftp()
        remote_dir = [d for d in ftp.listdir(remote_root) if 'd'
            in str(ftp.lstat(os.path.join(remote_root, d))).split()[0]]

        # Sending source files
        print '\n'
        print 'Pulling reports.nc file'
        get_file(ftp, os.path.join(remote_root, 'reports.nc'), os.path.join(local_root, 'reports_'+str(index)+'.nc'))
        ftp.close()
        ssh.close()
