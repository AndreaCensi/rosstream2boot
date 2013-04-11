#!/usr/bin/env python
import roslib; roslib.load_manifest('youbot_agent')
import sys
import numpy as np
from sensor_msgs.msg import PointCloud
import rospy  # @UnresolvedImport
import array_msgs.msg  # @UnresolvedImport
roslib.load_manifest('laser_assembler')
from laser_assembler.srv import *
from youbot_agent.msg import Safety
from rospy import ROSException
import yaml
from constraints import *

class ServoDemo():
    """ 
        First implementation of a servo demo; a big node does everything.
        
        Param:
        
        Input:
        
        Output:
        
    """

    def main(self, args, node_name='hokuyo_safety'):
        rospy.init_node(node_name)
        rospy.loginfo('Started.')

        self.pub_safe = rospy.Publisher('~out_safe', PointCloud)
        self.pub_unsafe = rospy.Publisher('~out_unsafe', PointCloud)
        self.pub_safety = rospy.Publisher('~out_safety', Safety)

        self.warn_distance = rospy.get_param('~warn_distance', 0.65)
        self.ignore_closer = rospy.get_param('~ignore_closer', 0.35)

        self.publish_clouds = rospy.get_param('~publish_clouds', True)

        rospy.loginfo('publish clouds: %r' % self.publish_clouds)

        assert 0 <= self.ignore_closer < self.warn_distance

        names = ['~assemble_scans%d' % i for i in range(2)]
        self.init_services(names)

        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            self.periodic()

    def periodic(self):

        clouds = self.get_clouds()            
        points = clouds[0].points + clouds[1].points
        frame_id = clouds[0].header.frame_id

        def norm(p):
            return np.hypot(np.hypot(p.x, p.y), p.z)


        p_safe = filter(lambda p: norm(p) > self.warn_distance, points)
        p_unsafe = filter(lambda p: (norm(p) > self.ignore_closer) and 
                                (norm(p) < self.warn_distance),
                          points)

        if self.publish_clouds:
            pc_safe = PointCloud()            
            pc_safe.header.frame_id = frame_id
            pc_safe.points = p_safe
            self.pub_safe.publish(pc_safe)

            pc_unsafe = PointCloud()            
            pc_unsafe.header.frame_id = frame_id
            pc_unsafe.points = p_unsafe
            self.pub_unsafe.publish(pc_unsafe)

        if len(p_unsafe) > 0:
            # find the closest unsafe point
            norms = map(norm, p_unsafe)
            closest = np.argmin(norms)
            closest_dist = norms[closest]
            assert closest_dist > self.ignore_closer
            assert closest_dist < self.warn_distance
            p0 = p_unsafe[closest]
            angle = np.rad2deg(np.arctan2(p0.y, p0.x))
            desc = ('Point at distance %.2fm angle %.1fdeg (x:%.2f, y:%.2f) ign: %s warn: %s' 
                    % (norms[closest], angle, p0.x, p0.y, self.ignore_closer, self.warn_distance))
            plane = [p0.x, p0.y]
            constraints = [constraint_plane(desc=desc, plane=plane)]
        else:
            constraints = []

        
        msg_safety = Safety()
        msg_safety.id_check = 'hokuyo_safety'
        msg_safety.header.stamp = rospy.Time.now()
        msg_safety.constraints = yaml.dump(constraints)
        
        if p_unsafe:
            msg_safety.safe = False
            msg_safety.desc = '%d unsafe points' % len(p_unsafe)
        else:
            msg_safety.safe = True
            msg_safety.desc = 'OK' 
        
        self.pub_safety.publish(msg_safety)

    def init_services(self, names , timeout=1):
        rospy.loginfo('Opening service proxies...')
        srv_assemble = []
        for n in names:
            rospy.wait_for_service(n, timeout=timeout)
            rospy.loginfo('Connecting to %s' % n)
            x = rospy.ServiceProxy(n, AssembleScans, persistent=True)
            rospy.loginfo('Connecting to %s [done]' % n)
            srv_assemble.append(x)
        self.services = srv_assemble

    def get_services(self):
        return self.services

    def get_clouds(self):
        clouds = []
        services = self.get_services()
        for i, srv in enumerate(services):
            resp = srv(rospy.Time(0, 0), rospy.get_rostime())
            # rospy.loginfo('Cloud %d: %s points' %(i,  len(resp.cloud.points)))
            clouds.append(resp.cloud)
        return clouds
    

def main(args):
    ServoDemo().main(args)
   

if __name__ == '__main__':
    main(sys.argv)
