#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client
import dynamic_reconfigure.server
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

class Multireconfigure:
    clients = list()
    ddynrec = None
    first_callback = True


    def usr_callback(self, config, level):
        # ignore the first callback after initialization (that will contain the default values we don't care about)
        if self.first_callback:
            self.first_callback = False
            return config

        rospy.loginfo("Got new config: {}".format(config))
        for client in self.clients:
            rospy.loginfo("Updating client {}".format(client.name))
            client.update_configuration(config)
        return config


    def __init__(self, dynparam_topics):
        
        self.ddynrec = DDynamicReconfigure("multireconfigure")
        cur_cfg = dict()
        for topic_name in dynparam_topics:
            rospy.loginfo("Registering client at {}".format(topic_name))
            try:
                client = dynamic_reconfigure.client.Client(topic_name, timeout=1)
            except:
                rospy.logwarn("Couldn't register client (timed out). Skipping.")
                continue

            # save the current values of the dynparams
            config = client.get_configuration()
            for key in config:
                cur_cfg[key] = config[key]

            # create the ddynrec variables
            descr = client.get_parameter_descriptions()
            for var in descr:
                if self.ddynrec.get_variable_names().count(var['name']) == 0:
                    tp = var['type']
                    if tp == "int" or tp == "double":
                        self.ddynrec.add_variable(var['name'], var['description'], var['default'], var['min'], var['max'])
                    elif tp == "str" or tp == "bool":
                        self.ddynrec.add_variable(var['name'], var['description'], var['default'])
                    else:
                        rospy.logerr("Variable type {} not implemented. Feel free to fix this.".format(tp))
            self.clients.append(client)

        # Start the server
        self.ddynrec.start(self.usr_callback)
        # Update the server with the current values to override the defaults
        self.ddynrec.dyn_rec_srv.update_configuration(cur_cfg)
        rospy.loginfo("DDynRec server started")


if __name__ == "__main__":
    rospy.init_node("multireconfigure")
    topics = rospy.get_param("~dynparam_topics")
    mrc = Multireconfigure(topics)
    rospy.spin()
