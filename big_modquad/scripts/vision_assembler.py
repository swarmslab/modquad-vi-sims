#!/usr/bin/env python

import rospy
import numpy as np
import networkx as nx
from modquad.srv import *
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import PoseArray

class vision_assembler:
    image_updated = False
    def __init__(self):
        self.target_assembly = self.parse_2d(rospy.get_param('target_assembly'))
        self.G = nx.Graph()
        #self.state_pub = rospy.Publisher('/modquad_vision_assembler', VisionOdom, queue_size=10)
        rospy.Subscriber('/modquad', PoseArray, callback=self.image_detection_cb)
        self.waypoint_srvs = []
        self.dock_srvs = []
        self.track_srvs = []
        self.face_directions = {0 : 0.0, 1 : -np.pi/2, 2 : np.pi, 3 : np.pi/2}
        for i in range(self.target_assembly.size/5):
            self.waypoint_srvs.append(rospy.ServiceProxy('/modquad'+str(i)+'/send_waypoint', sendwaypoint))
            self.dock_srvs.append(rospy.ServiceProxy('/modquad'+str(i)+'/dock', dock))
            self.track_srvs.append(rospy.ServiceProxy('/modquad'+str(i)+'/track', track))

        rospy.init_node('modquad_vision_assembler')
        rate = rospy.Rate(2)

        self.calculate_positions(self.G, self.target_assembly)
        self.calculate_orientations(self.G, self.target_assembly)

        while not rospy.is_shutdown():
            #rospy.loginfo(self.target_assembly[2][0])
            rate.sleep()

#TODO: in order for filters to correctly initialize we must first calculate the intended docking orientation and then reset the filter. This can be done by sending a docking service message when the modquads arent looking at a tag. Hence it would be ideal at the beginning of the program before the quads take off.

    def parse_2d(self, unparsed):
        unparsed = np.asarray(unparsed)
        return np.reshape(unparsed, (-1, 5))

    def calculate_positions(self, G, target):
        IDs = target[:,0].tolist()
        G.add_nodes_from(IDs)

        for rows in target:
            for j in range(5):
                if rows[j] > -1:
                    if j == 1:  # up 1
                        G.add_edge(rows[0], rows[j], dock=[-1.0, 0])
                    elif j == 2:  # right 2
                        G.add_edge(rows[0], rows[j], dock=[0, 1.0])
                    elif j == 3:  # down 3
                        G.add_edge(rows[0], rows[j], dock=[1.0, 0])
                    elif j == 4:  # left 4
                        G.add_edge(rows[0], rows[j], dock=[0, -1.0])

    def calculate_orientations(self, G, target):
        #find quad with most connections, set that to back and then grow out from there, if multiple quads with most connections, just pick random one

        IDs = target[:,0].tolist()
        raw_connections = target[:,1:]
        num_connections = []
        for rows in raw_connections:
            rows = rows[(rows != -1)]
            num_connections.append(rows.size)
        num_connections = np.array(num_connections)
        max_conn_idxs = np.where(num_connections == np.amax(num_connections))[0]

        if max_conn_idxs.size % 4 != 0: #no loops in structure
            master = np.random.choice(max_conn_idxs)
            rospy.loginfo("Master robot ID is: " + str(master))
            positions = self.components_locations(G, len(IDs))
            #master has to be available for future dockings, so it has to face an unoccupied direction
            available_face = np.where(raw_connections[master] == -1)[0][0]
            #TODO: consider case where all 4 faces of master are occupied (due to symmetry, we can make all modules point at the same direction and the master would be the furthermost module)
            self.waypoint_srvs[master](positions[master][0],
                                        positions[master][1],
                                        1.0,
                                        self.face_directions[available_face])

            for it, srvs in zip(IDs, self.waypoint_srvs):
                direction = np.where(raw_connections[it] != -1)[0][0]
                if it == master:
                    continue
                srvs(positions[it][0], positions[it][1], 1.0, self.face_directions[direction])
            
        else: #looping case, e.g. rectangles, loops, etc.
            print("beep")

    #TODO: rework components_locations to work when IDs are a dictionary and not range(n)
    def components_locations(self, G, n):
        """
        Convert graph to locations.
        :param G:
        :param n:
        :return: location of each node in its component.
        """
        zero2d = np.array([0., 0.])
        # location for each node in the local frame
        loc = [zero2d for i in range(n)]

        # For each subgraph, compute its structure
        for C in nx.connected_components(G):
            # Minimum id in component
            i = min(C)
            # remove minimum from i
            C.remove(i)

            def move_neighbors(i, G, C, loc):
                for j in G.neighbors(i):
                    if j in C:
                        mov = G[i][j]['dock']

                        if j < i:
                            mov = -np.array(mov)

                        loc[j] = loc[i] + mov

                        C.remove(j)

                        # Expand recursively
                        move_neighbors(j, G, C, loc)
                return
            move_neighbors(i, G, C, loc)
        return np.array(loc)

    def image_detection_cb(self,msg):
        self.whycon_position = msg

if __name__ == "__main__":
    try:
        vision_assembler()
    except rospy.ROSInterruptException:
        pass
