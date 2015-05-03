#!/usr/bin/env python
from __future__ import division

import rospy

import rostest
import sys
from math import ceil

import random
from strands_executive_msgs.msg import Task, DurationList, DurationMatrix
from strands_executive_msgs.srv import GetSchedule
from strands_executive_msgs.srv import GetExpectedTravelTimesToWaypoint

class StandaloneSchedulerClient(object):

    def __init__(self):
        super(StandaloneSchedulerClient, self).__init__()
        self.create_srvs()
        

    def create_srvs(self):
        expected_time_srv_name = 'mdp_plan_exec/get_expected_travel_times_to_waypoint'
        rospy.loginfo('Waiting for %s' % expected_time_srv_name)
        rospy.wait_for_service(expected_time_srv_name)
        rospy.loginfo('... and got %s' % expected_time_srv_name)
        self.expected_time_srv = rospy.ServiceProxy(expected_time_srv_name, GetExpectedTravelTimesToWaypoint)        

        schedule_srv_name = 'get_schedule'
        rospy.loginfo("Waiting for scheduler service...")
        rospy.wait_for_service(schedule_srv_name)
        rospy.loginfo("Done")        
        self.schedule_srv = rospy.ServiceProxy(schedule_srv_name, GetSchedule)
        
    

    def get_navigation_duration(self, start, end, task = None):

        try:            
            et = self.mdp_expected_time(start, end, task)
        except Exception, e:
            rospy.logwarn('Caught exception when getting expected time: %s' % e)
            return rospy.Duration(10)



    def get_duration_matrix_mdp(self, tasks):
        """
        Creates the matrix of durations between waypoints needed as input to the scheuler.
        Output is a DurationMatrix encoding  duration[i][j] where this is the duration expected for travelling between the end of the ith task in tasks and the start of the jth element.
        """

        # mdp returns vector of times from ALL START nodes to a SINGLE TARGET for a SINGLE TIME 
        # therefore we can combine calls from any start node to the same target for the same time 


        # used to cache reseults of calls to mdp service
        travel_durations = dict()

        # thing we need to return
        dm = DurationMatrix()

        for first_task in tasks:
            dm.durations.append(DurationList())
            for second_task in tasks:

                start = first_task.end_node_id
                epoch = second_task.start_after
                target = second_task.start_node_id

                # if the task should happen where the robot is currently stood then the travel is free
                if target == '':
                    dm.durations[-1].durations.append(rospy.Duration(0))
                else:
                    # we need to get the travel_duration for this tuple if we haven't seen the time before or the target for this time before
                    if epoch not in travel_durations or target not in travel_durations[epoch]:
                        if epoch not in travel_durations:
                            travel_durations[epoch] = dict()
                        # call mdp duration service
                        resp = self.expected_time_srv(target, epoch)
                        travel_durations[epoch][target] = resp
                    else:
                        # rospy.loginfo('Saving a call: %s %s %s', start, epoch.secs, target)                    
                        resp = travel_durations[epoch][target]

                    dm.durations[-1].durations.append(resp.travel_times[resp.source_waypoints.index(start)])

        return dm

    def run_scheduler(self, tasks):    
        try:

            durations = self.get_duration_matrix_mdp(tasks)
            first_task = 0
            earliest_start = rospy.Time(0)
                    
            # Schedule the tasks
            resp = self.schedule_srv(tasks, earliest_start, first_task, durations)

            print resp

            if len(resp.task_order) > 0:

                # add start times to a dictionary for fast lookup
                task_times = {}
                for (task_id, start_time) in zip(resp.task_order, resp.execution_times):
                    task_times[task_id] = start_time

                # set start times inside of tasks
                for task in tasks:
                    task.execution_time = task_times[task.task_id]
                    print 'task %s   window from %s.%s to %s.%s' % (task.task_id, task.start_after.secs, task.start_after.nsecs, task.end_before.secs, task.end_before.nsecs)            
                    print 'task %s will start at %s.%s' % (task.task_id, task.execution_time.secs, task.execution_time.nsecs)            

            
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def create_tasks_in_single_window(self, task_count, waypoints=['Station', 'ChargingPoint', 'WayPoint3', 'WayPoint6', 'WayPoint2', 'WayPoint1']):           
        one_hour_secs = 60 * 60 * 60
        max_duration = rospy.Duration(one_hour_secs)
        start_of_window = rospy.get_rostime()
        end_of_window = start_of_window + rospy.Duration(one_hour_secs * (task_count * 2))

        print "creating %s tasks of length %s to fit into %s" % (task_count, rospy.Duration(max_duration.secs/2).secs, (end_of_window - start_of_window).secs)


        tasks = []
        for task_id in range(1, task_count+1):    
            # create the task from the description
            task = Task()
            task.task_id=task_id
            task.start_node_id=random.choice(waypoints)
            task.end_node_id=task.start_node_id
            task.start_after = start_of_window
            task.end_before = end_of_window
            task.max_duration = rospy.Duration(max_duration.secs/2)
            tasks.append(task)

        return tasks

        
    def get_tasks(self):
        """
        Provide some tasks
        """
        return self.create_tasks_in_single_window(5)


if __name__ == '__main__':
    rospy.init_node('standalone_scheduler_client')
    client = StandaloneSchedulerClient()
    client.run_scheduler(client.get_tasks())

    rospy.spin()


