#!/usr/bin/python3
import os
import subprocess
import sys
import argparse
from subprocess import Popen, PIPE

# Thrown when a called process yields a non-zero exit status
class ProcessNonZeroExitException(Exception):
    def __init__(self, code, stderr):
        super().__init__()
        self.code = code
        self.stderr = stderr

# Thrown when a called process times out
class ProcessTimedOutException(Exception):
    pass

# Prints an error to the stdout and exits the program with a status of 1
def err(msg):
    print("ERROR: {}".format(msg))
    sys.exit(1)

# Executes a given command and returns the standard output if the process exited
# with a non-zero status. If the process timed out, or a non-zero exit status occurred,
# an appropriate exception is thrown.
def execute(cmd, timeout=5):

    # TODO: although it's better to avoid sharing the same shell as the program, we do
    #   so to avoid having to reload the catkin workspace. No big deal.
    # NOTE: os.setsid is used to ensure we kill all children in the progress group
    with Popen(cmd, shell=True, stdout=PIPE, stderr=PIPE, preexec_fn=os.setsid) as p:
        try:
            # convert stdout and stderr to strings
            stdout, stderr = p.communicate(timeout=timeout)
            stdout = str(stdout)[2:-1]
            stderr = str(stderr)[2:-1]
            retcode = p.returncode

            if retcode != 0:
                raise ProcessNonZeroExitException(retcode, stderr)
            return stdout

        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            raise ProcessTimedOutException()


# TODO: Certain "respawing" nodes will refuse to die (http://wiki.ros.org/rosnode)
def kill_node(node_name):
    print("Killing node: {}".format(node_name))

    # sanity check: is this a legal ROS node name? If not, we could abuse this to perform
    # shell injection. Probably not an issue in the context we're using it, but it's worth
    # keeping in mind.

    # sanity check: is ROS running?

    # sanity check: is there a node with the given name running?
    
    try:
        execute("rosnode kill {}".format(node_name))
    except ProcessNonZeroExitException as e:
        err("failed to kill node - non-zero exit status ({}):\n{}".format(e.code, e.stderr))
    except ProcessTimedOutException as e:
        err("failed to kill node - request timed out")

def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers() 
    parser.add_argument('--version', action='version', version='0.0.1')

    generate_parser = subparsers.add_parser('kill-node')
    generate_parser.add_argument('name', \
                                 help='name of the ROS node that should be killed')
    generate_parser.set_defaults(func=lambda args: kill_node(args.name))

    args = parser.parse_args() 
    if 'func' in vars(args):
        args.func(args)

if __name__ == "__main__":
    main()
