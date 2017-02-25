#!/usr/bin/python2.7
import os
import subprocess
import sys
import argparse
import xmlrpclib
import rostopic
import time
from subprocess import Popen, PIPE
from datetime import datetime


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
# TODO: double check if line 37 is working properly 
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
            raise ProcessTimedOutException()
        finally:
            os.killpg(p.pid, signal.SIGKILL)

# Causes a specified fraction (x out of y) of messages published to a given
# topic to be dropped.
#
# TODO: is there a Python API for topic_tools?
def drop(topic, x, y, interval, drop_count):
    # TODO: sanity check topic name
    assert x >= 0, "expected x to be an integer greater or equal to zero"
    assert y > 0, "expected y to be an integer greater than zero"
    assert isinstance(x, int), "expected x to be an integer"
    assert isinstance(y, int), "expected y to be an integer"

    for count in range(drop_count):
        print("dropping {} out of {} messages on topic {}...".format(x, y, topic))
        cmd = "rosrun topic_tools {} {} {}".format(topic, x, y)
        execute(cmd, timeout=None)
        time.sleep(interval)
    
    print("stopped dropping messages on topic {}".format(topic))

# Uses ROS Master API (http://wiki.ros.org/ROS/Master_API) to unregister publishers (pub),
# subscribers (sub) and services (ser).
#
# TODO: Why return an integer rather than a boolean?
# The integer that we get returned can give us an insight in what went wrong and why the
# task was not successfully completed. 
def unregister(node_name, topic_service_name, task_sub_pub_ser, quiet):
    timeStamp = str(datetime.now())
    m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
    MainCode, STMAIN, URI = m.lookupNode(node_name, node_name)


    if task_sub_pub_ser == 'pub':
        code, statusMessage, numUnregistered = m.unregisterPublisher(node_name, topic_service_name, URI)
        if not quiet:
            print('['+ timeStamp +'] ::: ' + statusMessage)
        return code
    if task_sub_pub_ser == 'sub':
        code, statusMessage, numUnregistered = m.unregisterSubscriber(node_name, topic_service_name, URI)
        if not quiet:
            print('['+ timeStamp +'] ::: '+statusMessage)
        return code
    elif task_sub_pub_ser == 'ser':
        MainCode, STMAIN, URI = m.lookupService(node_name, topic_service_name)
        code, statusMessage, numUnregistered = m.unregisterService(node_name, topic_service_name, URI)
        if not quiet:
            print('['+ timeStamp +'] ::: ' + statusMessage)
        return code

    return 0

# Uses unregister to automatically unregister all publishers, subscribers and services of a given
# node. It also allows the user to provide an specific type to unregister and an interval time
# between the unregistrations. 
def isolate(node_name, task_sub_pub_ser_all, quiet, interval):
    m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
    code, statusMessage, systemState = m.getSystemState(node_name)
    system_publishers = systemState[0]
    system_subscribers = systemState[1]
    system_services = systemState[2]

    if task_sub_pub_ser_all == 'pub':
        report_success_failure(isolate_type(node_name, 'pub', system_publishers, quiet, interval))

    elif task_sub_pub_ser_all == 'sub':
        report_success_failure(isolate_type(node_name, 'sub', system_subscribers, quiet, interval))

    elif task_sub_pub_ser_all == 'ser':
        report_success_failure(isolate_type(node_name, 'ser', system_services, quiet, interval))

    elif task_sub_pub_ser_all == 'all':
        resultsSuccesFailure = [isolate_type(node_name, 'pub', system_publishers, quiet, interval), \
        isolate_type(node_name, 'sub', system_subscribers, quiet, interval), \
        isolate_type(node_name, 'ser', system_services, quiet, interval)]
        print('Success: pulishers: '+ str(resultsSuccesFailure[0][0]) + ', subscribers: ' + \
                str(resultsSuccesFailure[1][0]) + ', services: ' + str(resultsSuccesFailure[2][0]))
        print('Failure: pulishers: '+ str(resultsSuccesFailure[0][1]) + ', subscribers: ' + \
                str(resultsSuccesFailure[1][1]) + ', services: ' + str(resultsSuccesFailure[2][1]))

def isolate_type(node_name, _type, system_pub_sub_ser, quiet, interval):
    types = {'pub' : 'publishers', 'sub' : 'subscribers', 'ser' : 'services'}
    to_remove = get_publishers_subscribers_providers(node_name, system_pub_sub_ser)
    status = isolate_print_status(types[_type], node_name, len(to_remove), interval)
    success_failure = [0,0]
    if status:
        for current_type in to_remove:
            returnedCodeValue = unregister(node_name, current_type, _type, quiet)
            if returnedCodeValue == 1:
                success_failure[0] = success_failure[0] + 1
            else:
                success_failure[1] = success_failure[1] + 1
                print ('Failed by code: ' + returnedCodeValue + '. Node: ' + node_name + \
                                                        '. Topic/Service: ' + current_type)
            time.sleep(interval)
        return success_failure


# Prints out the data adquired (number of tasks). If there are more than 0 tasks (unregister) 
# then returns a true state which starts the unregistration process otherwise prints that nothing 
# will be done.
def isolate_print_status(from_type, node_name, size, interval):
    print('Found [' + str(size) + '] '+ from_type +' by [' + node_name +']')
    if size is not 0:
        print('Starting automatic unregistration with an interval of ' +  str(interval) + ' seconds.')
        return True
    else:
        print('Nothing to be done.')
        return False


# Reports the number of successes and failures encountered
def report_success_failure(successFailure):
    print('Success: ' + str(successFailure[0]))
    print('Failure: ' + str(successFailure[1]))

# Iterates through the list acquired by m.getSystemState(node_name) and
# determines which publishers, subscribers and services are linked to the given
# node then returns the filtered data corresponding to the provided type. 
#
# TODO: use a tuple as the return type
def get_publishers_subscribers_providers(node_name, _list, all_types = False):
    toReturn_all = [[],[],[]]
    toReturn = []
    index = -1
    if all_types:
        for pub_sub_ser in _list:
            index += 1
            for topic_ser in pub_sub_ser:
                if node_name in topic_ser[1]:
                    toReturn_all[index].append(topic_ser[0])

        return toReturn_all
    else:
        for topic_ser in _list:
            if node_name in topic_ser[1]:
                toReturn.append(topic_ser[0])
        return toReturn

# TODO: Certain "respawing" nodes will refuse to die (http://wiki.ros.org/rosnode)
def kill_node(node_name, quiet):
    print("Killing node: {}".format(node_name))

    # sanity check: is this a legal ROS node name? If not, we could abuse this to perform
    # shell injection. Probably not an issue in the context we're using it, but it's worth
    # keeping in mind.( ~ Check ~)


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
        print('Missing ' + repr('/') + ' at the begining of the node-topic. Modifying '\
                                 + _node_topic_service + ' to ' + node_topic_service)+'.'

    print('')
    return node_topic_service

# Checks if rosmaster is running
def check_ros_running():
    try:
        rostopic.get_topic_class('/rosout')
        return True
    except:
        print('WARNING: Unable to detect rosmaster. Is rosmaster running?\n')
        return False

def main():
    ros_running = check_ros_running()
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers() 
    parser.add_argument('--version', action='version', version='0.0.1')
    parser.add_argument('-i', '--ignore_rosmaster_state', action='store_true', \
                                                                            required=False)
    parser.add_argument('-q', '--quiet', action='store_true', required=False, default=False)

    # kill node
    generate_parser = subparsers.add_parser('kill-node')
    generate_parser.add_argument('name', \
                                 help='name of the ROS node that should be killed')
    generate_parser.set_defaults(func=lambda args: kill_node(check_format(args.name), \
                                                                            args.quiet))

    # drop
    # TODO: is a fraction or a real better?
    drop_parser = subparsers.add_parser('drop')
    drop_parser.add_argument('topic', \
                             help='Topic whose messages should be subject to dropping')
    drop_parser.add_argument('x', type=int, \
                             help='Numerator of fraction of messages that should be dropped')
    drop_parser.add_argument('y', type=int, \
                             help='Denominator of fraction of messages that should be dropped')
    drop_parser.add_argument('-in', '--interval', \
                            help='Time in between each drop.', type=float, default=1.0, required=False)
    drop_parser.add_argument('-dc', '--drop_count', \
                            help='Determine amount of drops needed.', type=int, default=1, required=False )
    drop_parser.set_defaults(func=lambda args: drop(check_format(args.topic), args.x, args.y, args.interval, \
                                args.drop_count))

    # unregister
    unregister_parser = subparsers.add_parser('unregister')
    unregister_parser.add_argument('-t', '--type', choices=['pub', 'sub', 'ser'], \
                                 required=True, \
                                 help='Select type to be unregister subscriber, publisher or service')
    # TODO: may be better as a positional argument?
    # We could set it up as positional, but I think it gives more structure to the command to clarify
    # what node we are refering to (at least in the beginning stage). 
    unregister_parser.add_argument('-n','--node', \
                                 required=True,\
                                 help='Node that handles the publisher, subscriber or service.')
    unregister_parser.add_argument('-nm', '--topic_or_service_name', \
                                 required=True,\
                                 help='Topic or service that is going to be unregister.')
    unregister_parser.set_defaults(func=lambda args: unregister(check_format(args.node), \
                                    check_format(args.topic_or_service_name), args.type, args.quiet))

    # Isolates a given node for a type or all types. For example; isolate -n gazebo -t all.
    isolate_parser = subparsers.add_parser('isolate')
    isolate_parser.add_argument('-n', '--node', \
                                 help='Node that handles the publishers, subscribers or services.', \
                                                                                        required=True)
    isolate_parser.add_argument('-t', '--type', choices=['pub', 'sub', 'ser', 'all'], \
                                 help='Select type to unregister; subscriber, publisher or service. \
    Use all to automatically remove any service, publisher or subscrieber linked to the given \
    node.', required=True)
    isolate_parser.add_argument('-in', '--interval', \
                                help='Time in between each unregistration', action='store', type=float, \
                                                                             default=0.5,required=False)
    isolate_parser.set_defaults(func=lambda args: isolate(check_format(args.node), args.type, \
                                                                             args.quiet, args.interval))

    args = parser.parse_args() 
    if not args.ignore_rosmaster_state and not ros_running:
        err('ROS not detected. If you still want to run try adding -i (ex. roshammer.py -i kill-node /example)')
    if 'func' in vars(args):
        args.func(args)

if __name__ == "__main__":
    main()