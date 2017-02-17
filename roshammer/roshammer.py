#!/usr/bin/python3
from subprocess import Popen, PIPE
import sys
import argparse

def err(msg):
    print("ERROR: {}".format(msg))
    sys.exit(1)

# TODO: Certain "respawing" nodes will refuse to die (http://wiki.ros.org/rosnode)
def kill_node(node_name):
    print("Killing node: {}".format(node_name))

    # sanity check: is this a legal ROS node name? If not, we could abuse this to perform
    # shell injection. Probably not an issue in the context we're using it, but it's worth
    # keeping in mind.

    # sanity check: is ROS running?

    # sanity check: is there a node with the given name running?
    
    # TODO: although it's better to avoid sharing the same shell as the program, we do
    #   so to avoid having to reload the catkin workspace. No big deal.
    # NOTE: os.setsid is used to ensure we kill all children in the progress group
    time_limit = 5.0 # seconds
    cmd = "roskill {}".format(node_name)
    with Popen(cmd, shell=True, stdout=PIPE, stderr=PIPE, preexec_fn=os.setsid) as p:
        try:
            # convert stdout and stderr to strings
            stdout, stderr = p.communicate(timeout=time_limit)
            stdout = str(stdout)[2:-1]
            stderr = str(stderr)[2:-1]
            retcode = p.returncode

        except TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
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
