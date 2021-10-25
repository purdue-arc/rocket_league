#!/usr/bin/env python3

from threading import Thread, Lock
from colorama import Fore
import socket
import pathlib
import pty
import os
import yaml
import signal


class Host:
    def __init__(self, username, hostname, run_cmd, exec_cmd, roscore):
        self.username = username
        self.hostname = hostname
        self.run_cmd = run_cmd
        self.exec_cmd = exec_cmd
        self.roscore = roscore
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
        self.color = Fore.RESET
        self.running = False

    def set_color(self, color):
        self.color = color

    def print(self, *args):
        self.mutex.acquire()
        try:
            print(*args)
        finally:
            self.mutex.release()

    def run(self, hosts, mutex):
        print('{}Starting {}{}'.format(self.color, self.hostname, Fore.RESET))
        argv = self.run_cmd
        if not self.is_local:
            argv = ['ssh', '-t', '%s@%s.local' %
                    (self.username, self.hostname), *argv]
        roscore_host = [x.hostname for x in hosts if x.roscore][0]
        argv += ['--add-host=%s:%s' % (x.hostname, x.ip) for x in hosts]
        argv.append('--env=ROS_MASTER_URI=http://{}:11311'.format(roscore_host))
        self.pid, self.fd = pty.fork()
        if self.pid == 0:
            os.execvp(argv[0], argv)
        self.running = True
        self.mutex = mutex
        command = 'exec %s \r\n' % self.exec_cmd
        os.write(self.fd, command.encode())
        prefix = "{}{:<{offset}}  | {}".format(
            self.color, self.hostname, Fore.RESET, offset=max([len(x.hostname) for x in hosts]))
        buffer = b''
        while self.running:
            try:
                buffer += os.read(self.fd, 1024)
                lines = buffer.split(b'\r\n')
                for line in lines[:-1]:
                    text = line.decode().replace('\r', '\r' + prefix)
                    self.print(prefix + text)
                buffer = lines[-1]
            except OSError:
                self.running = False

        self.print('{}Shut down {}{}'.format(
            self.color, self.hostname, Fore.RESET))
        os.waitpid(self.pid, 0)
        os.close(self.fd)

    def stop(self):
        if self.running:
            self.print('{}Shutting down {}...{}'.format(
                self.color, self.hostname, Fore.RESET))
            os.write(self.fd, '\x03'.encode())


if __name__ == '__main__':
    with open(pathlib.Path().resolve() / 'hosts.yaml', 'r') as stream:
        try:
            hosts_info = yaml.safe_load(stream)['hosts']
        except yaml.YAMLError as exc:
            print(exc)
    hosts = [Host(**x) for x in hosts_info]
    colors = [Fore.BLUE, Fore.CYAN, Fore.MAGENTA, Fore.RED, Fore.YELLOW, Fore.LIGHTBLACK_EX,
              Fore.LIGHTBLUE_EX, Fore.LIGHTCYAN_EX, Fore.LIGHTMAGENTA_EX, Fore.LIGHTRED_EX, Fore.LIGHTYELLOW_EX]
    color_idx = 0
    for host in hosts:
        host.set_color(colors[color_idx])
        color_idx = (color_idx + 1) % len(colors)

    def handler(signum, frame):
        print('\r', end='')
        for host in hosts:
            host.stop()
    signal.signal(signal.SIGINT, handler)

    mutex = Lock()
    threads = [Thread(target=x.run, args=(hosts, mutex)) for x in hosts]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
