
import smach
from fsm_waypoint.tasks import spin_once
from fsm_waypoint.tasks import network_error
from fsm_waypoint.tasks import wss_cancel_command
from fsm_waypoint.tasks import update_waypoint
from fsm_waypoint.utils import child_termination_cb, outcome_cb


def update(node):
    sm = smach.Concurrence(outcomes=['succeeded', 'preempted', 'aborted', 'cancel'],
                           default_outcome='aborted',
                           input_keys=['blackboard'],
                           output_keys=['blackboard'],
                           child_termination_cb=child_termination_cb,
                           outcome_cb=outcome_cb)

    with sm:
        smach.Concurrence.add('WSS_COMMAND', wss_cancel_command(node))
        smach.Concurrence.add('SPIN_ONCE', spin_once(node))
        smach.Concurrence.add('NETWORK_ERROR', network_error(node))
        smach.Concurrence.add('UPDATE_WAYPOINT', update_waypoint(node))
    return sm