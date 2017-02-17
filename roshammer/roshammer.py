#!/usr/bin/python3
import os
import subprocess
import sys
import argparse
import xmlrpclib
import rostopic
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

# Uses ROS Master API (http://wiki.ros.org/ROS/Master_API) to unregister publishers (pub),
# subscribers (sub) and services (ser). 
def unregister(node_name, topic_service_name, task_sub_pub_ser):
    node_name = check_format(node_name)
    topic_service_name = check_format(topic_service_name)
    m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
    MainCode, STMAIN, URI = m.lookupNode(node_name, node_name)
    if task_sub_pub_ser == 'sub':
        code, statusMessage, numUnregistered = m.unregisterSubscriber(node_name, topic_service_name, URI)        
        print(statusMessage)
    if task_sub_pub_ser == 'pub':
        code, statusMessage, numUnregistered = m.unregisterPublisher(node_name, topic_service_name, URI)
        print(statusMessage)
    elif task_sub_pub_ser == 'ser':
        MainCode, STMAIN, URI = m.lookupService(node_name, topic_service_name)
        code, statusMessage, numUnregistered = m.unregisterService(node_name, topic_service_name, URI)
        print(statusMessage)

# TODO: Certain "respawing" nodes will refuse to die (http://wiki.ros.org/rosnode)
def kill_node(node_name):


    node_name = check_format(node_name)
    print("Killing node: {}".format(node_name))

    # sanity check: is this a legal ROS node name? If not, we could abuse this to perform
    # shell injection. Probably not an issue in the context we're using it, but it's worth
    # keeping in mind.( ~ Check ~)

    # sanity check: is ROS running? (Check)

    # sanity check: is there a node with the given name running?
    
    try:
        execute("rosnode kill {}".format(node_name))
    except ProcessNonZeroExitException as e:
        err("failed to kill node - non-zero exit status ({}):\n{}".format(e.code, e.stderr))
    except ProcessTimedOutException as e:
        err("failed to kill node - request timed out")

# Verifies that the topic or node that is going to be processed complies with ROS guidelines.
def check_format(_node_topic_service):
    node_topic_service = _node_topic_service
    if node_topic_service[0] != '/':
        node_topic_service = '/' + _node_topic_service
        print('Missing ' + repr('/') + ' at the begining of the node-topic. Modifying ' + _node_topic_service + ' to ' + node_topic_service)+'.'

    print('')
    return node_topic_service

# Checks if rosmaster is running. 
def check_ros_running():
    try:
        rostopic.get_topic_class('/rosout')
        return True
    except:
        print('Unable to detect rosmaster. Is rosmaster running?')
        return False

def main():
    ros_running = check_ros_running()
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers() 
    parser.add_argument('--version', action='version', version='0.0.1')
    parser.add_argument('-i', '--ignore_rosmaster_state', action='store_true', required=False)

    generate_parser = subparsers.add_parser('kill-node')
    generate_parser.add_argument('name', \
                                 help='name of the ROS node that should be killed')
    generate_parser.set_defaults(func=lambda args: kill_node(args.name))

    generate_parser = subparsers.add_parser('unregister')
    generate_parser.add_argument('-t', '--type', choices=['sub', 'pub', 'ser'], required=True)
    generate_parser.add_argument('-n', '--node', \
                                 help='Node that handles either the publisher or the subscriber.', required=True)
    generate_parser.add_argument('-nm', '--topic_or_service_name', \
                                 help='Topic that is going to be modified.', required=True)
    generate_parser.set_defaults(func=lambda args: unregister(args.node, args.topic_or_service_name, args.type))

    args = parser.parse_args() 
    if (args.ignore_rosmaster_state) == False and (ros_running == False):
        print('ROS not detected. If you still want to run try adding -i (ex. roshammer.py -i kill-node /example )')
        exit()
    if 'func' in vars(args):
        args.func(args)
    

if __name__ == "__main__":
    main()
