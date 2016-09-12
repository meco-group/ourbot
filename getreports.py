#!/usr/bin/python

# This script
# - pulls reports.nc data from odroids
# - plots results

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
#   -s, --show: show plots

# Ruben Van Parys - 2016

import optparse, os, paramiko
from netCDF4 import Dataset
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TKAgg')
import collections as col

# Default parameters
current_dir = os.path.dirname(os.path.realpath(__file__))
local_root = os.path.join(current_dir,'orocos/emperor')
remote_root = '/home/odroid/orocos'
username = 'odroid'
password = 'odroid'
show_plots = True
hosts = col.OrderedDict()
# hosts['dave'] = '192.168.11.120'
hosts['kurt'] = '192.168.11.121'
# hosts['krist'] = '192.168.11.122'
server = 'kurt'


def get_file(ftp, rem_file, loc_file):
    ftp.get(rem_file, loc_file)


def plot_nc(file):
    data = Dataset(file, mode='r')
    time = data.variables['TimeStamp']
    signals = {}

    for key, value in data.variables.items():
        if key == 'TimeStamp':
            continue
        split = key.split('.')
        component = split[0]
        signal = split[1]
        if component not in signals:
            signals[component] = {}
        if signal not in signals[component]:
            signals[component][signal] = []
        signals[component][signal].append(value)

    for component, sgnls in signals.items():
        plt.figure()
        cnt = 1
        for signal, values in sgnls.items():
            plt.subplot(len(sgnls.values()), 1, cnt)
            plt.hold(True)
            legend = []
            for k, value in enumerate(values):
                plt.plot(time[:], value[:])
                legend.append(k)
            plt.legend(legend)
            plt.xlabel('time')
            plt.ylabel(signal)
            cnt += 1
        plt.title('component ' + component)
    data.close()

if __name__ == "__main__":
    usage = ('''Usage: %prog [options] arg. There are 2 arguments:
                local root path & remote root path. It can be left blank to use
                default values.''')
    op = optparse.OptionParser(usage=usage)
    op.add_option("-u", "--username", dest="username",
                  default=username, help="ssh username")
    op.add_option("-p", "--password", dest="password",
                  default=password, help="ssh password")
    op.add_option("-s", "--show", dest="show_plots",
                  default=show_plots, help="show plots")
    options, args = op.parse_args()

    if len(args) == 2:
        local_root = args(0)
        remote_root = args(1)

    username = options.username
    password = options.password
    show_plots = options.show_plots

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    index = 0
    for host, address in hosts.items():
        print 'Host %s ...' % host
        ssh.connect(address, username=username, password=password)
        ftp = ssh.open_sftp()
        remote_dir = [d for d in ftp.listdir(remote_root) if 'd'
            in str(ftp.lstat(os.path.join(remote_root, d))).split()[0]]

        # Sending source files
        print 'Pulling reports.nc file'
        get_file(ftp, os.path.join(remote_root, 'reports.nc'), os.path.join(local_root, 'reports_'+str(index)+'.nc'))
        ftp.close()
        ssh.close()
        index += 1

    if show_plots:
        for index, host in enumerate(hosts.values()):
            plot_nc(os.path.join(local_root, 'reports_'+str(index)+'.nc'))
        plt.show()
