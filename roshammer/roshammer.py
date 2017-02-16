#!/usr/bin/python3
import argparse

def action_kill_node(args):
    return kill_node(args.name)

# TODO: Certain "respawing" nodes will refuse to die (http://wiki.ros.org/rosnode)
def kill_node(node_name):
    print("Killing node: {}".format(node_name))

    # sanity check: is this a legal ROS node name? If not, we could abuse this to perform
    # shell injection. Probably not an issue in the context we're using it, but it's worth
    # keeping in mind.

    # sanity check: is ROS running?

    # sanity check: is there a node with the given name running?

    # did we kill the node?


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers() 
    parser.add_argument('--version', action='version', version='0.0.1')

    generate_parser = subparsers.add_parser('kill-node')
    generate_parser.add_argument('name', \
                                 help='name of the ROS node that should be killed')
    generate_parser.set_defaults(func=action_kill_node)

    args = parser.parse_args() 
    if 'func' in vars(args):
        args.func(args)

if __name__ == "__main__":
    main()
