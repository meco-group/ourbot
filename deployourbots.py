#!/usr/bin/python

# This script starts the naming service on 1 odroid and deploys all the ourbots.
# Run ./write2ourbots.py --help for available options.

# Usage: ./deployourbots.py [options]
#
# Options:
#   -u, --username: ssh username
#   -p, --password: ssh password
#   -s, --server: host serving the naming service
#
# Ruben Van Parys - 2015

import optparse, sys, os, paramiko

# Default parameters
remote_root = '/home/odroid/orocos'
username    = 'odroid'
password    = 'odroid'
hosts       = ['192.168.0.2']
server      = '192.168.0.2'

if __name__ == "__main__":
    usage = ("Usage: %prog [options]")
    op = optparse.OptionParser(usage=usage)
    op.add_option("-u", "--username", dest = "username", default = username, help = "ssh username")
    op.add_option("-p", "--password", dest = "password", default = password, help = "ssh password")
    op.add_option("-s", "--server", dest = "server", default = server, help = "host serving the naming service")

    options, args   = op.parse_args()

    username        = options.username
    password        = options.password
    server          = options.server

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy( paramiko.AutoAddPolicy() )

    for host in hosts:
        print 'Deploying host %s ...\n' % host
        ssh.connect(host, username = username, password = password)
        if host == server:
            ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command('ulimit -r 10')
            ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command('echo ' + password + ' | sudo -S killall omniNames')
            ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command('echo ' + password + ' | sudo -S rm /var/lib/omniorb/*')
            ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command('echo ' + password + ' | sudo omniNames -start &')
            ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command('deployer-corba-gnulinux ' + os.path.join(remote_root,'run.ops'))
            print ssh_stdout.read()
        else:
            ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command('ulimit -r 10')
            ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command('deployer-corba-gnulinux ' + os.path.join(remote_root,'run.ops'))
