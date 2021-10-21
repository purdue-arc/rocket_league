#!/usr/bin/env python3

import socket
import pathlib
import pty
import os
import yaml
import fcntl

username = 'arc'
hostnames = ['gerblick-laptop', 'gerblick-laptop.local']


class Host:
    def __init__(self, username, hostname, command, shutdown, ros_master):
        self.username = username
        self.hostname = hostname
        self.command = command
        self.shutdown = shutdown
        self.ros_master = ros_master
        if hostname == socket.gethostname():
            self.is_local = True
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                s.connect(("10.255.255.255", 80))
                self.ip = s.getsockname()[0]
            except:
                self.ip = '127.0.0.1'
            finally:
                s.close()
        else:
            self.is_local = False
            self.ip = socket.gethostbyname(hostname + '.local')
        self.pid = -1
        self.fd = -1

    def spawn(self, argv):
        pid, fd = pty.fork()
        if pid == 0:
            os.execvp(argv[0], argv)
        return pid, fd

    def start(self, hosts):
        print('Starting %s...' % self.hostname)
        argv = self.command
        if not self.is_local:
            argv = ['ssh', '%s@%s.local' %
                    (self.username, self.hostname), *argv]
        if '--net=host' in argv:
            i = argv.index('--net=host') + 1
            add_hosts = ['--add-host=%s:%s' %
                         (x.hostname, x.ip) for x in hosts]
            argv = [*argv[:i], *add_hosts, *argv[i:]]
        print(argv)
        self.pid, self.fd = self.spawn(argv)
        flag = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, flag | os.O_NONBLOCK)

    def stop(self):
        if self.fd != -1 and self.pid != -1:
            print('Shutting down %s...' % self.hostname)
            os.close(self.fd)
            os.waitpid(self.pid, 0)
            self.pid, self.fd = self.spawn(self.shutdown)

    def stopstop(self):
        os.waitpid(self.pid, 0)
        os.close(self.fd)
        print("Shut down %s" % self.hostname)


if __name__ == '__main__':
    with open(pathlib.Path().resolve() / 'hosts.yaml', 'r') as stream:
        try:
            hosts_info = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    hosts = [Host(**x) for x in hosts_info['hosts']]
    for host in hosts:
        host.start(hosts)
    try:
        while True:
            pass
    except KeyboardInterrupt:
        pass
    finally:
        for host in hosts:
            host.stop()
        for host in hosts:
            host.stopstop()
