#!/usr/bin/python3
import argparse

def action_kill_node(args):
    return kill_node(args.name)
def kill_node(node_name):
    print("Killing node: {}".format(node_name))

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
