#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client
import dynamic_reconfigure.server
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

class Multireconfigure:
    clients = list()
    ddynrec = None

    def usr_callback(self, config, level):
        for client in self.clients:
            rospy.loginfo("Updating client {}".format(client.name))
            client.update_configuration(config)
        return config

    def __init__(self, dynparam_topics):
        
        self.ddynrec = DDynamicReconfigure("multireconfigure")
        for topic_name in dynparam_topics:
            rospy.loginfo("Registering client at {}".format(topic_name))
            try:
                client = dynamic_reconfigure.client.Client(topic_name, timeout=5)
            except:
                rospy.logwarn("Couldn't register client (timed out). Skipping.")
                continue

            config = client.get_parameter_descriptions()
            for var in config:
                if self.ddynrec.get_variable_names().count(var['name']) == 0:
                    self.ddynrec.add_variable(var['name'], var['type'], var['default'], var['min'], var['max'])
            self.clients.append(client)

        # Start the server
        self.ddynrec.start(self.usr_callback)
        rospy.loginfo("DDynRec server started")


if __name__ == "__main__":
    rospy.init_node("multireconfigure")
    topics = rospy.get_param("~dynparam_topics")
    mrc = Multireconfigure(topics)
    rospy.spin()
