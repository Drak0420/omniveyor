#!/usr/bin/python

from slam_toolbox_msgs.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import String
from spatio_temporal_voxel_layer.srv import SaveGrid
import rospy
import rospkg

if __name__ == "__main__":
    rospy.init_node("map_saving_utility")
    mapPath = rospy.get_param("map_save_path", rospkg.RosPack().get_path('omniveyor_mobility')+'/resources/maps/map')
    interval = rospy.get_param("map_save_interval", 60.0)
    if (interval < 0.):
        raise RuntimeError("ERROR: map saving interval must not be a negative number!")
    planarMapSaveSrvUp = False
    pointCloudSaveSrvUp = False
    save2dMap = None
    save3dMap = None

    f = rospy.Rate(1./interval)

    while(not rospy.is_shutdown()):
        if (not planarMapSaveSrvUp):
            try:
                rospy.wait_for_service("slam_toolbox/serialize_map", timeout=1.0)
                planarMapSaveSrvUp = True
                save2dMap = [rospy.ServiceProxy('slam_toolbox/serialize_map', SerializePoseGraph),
                            rospy.ServiceProxy('slam_toolbox/save_map', SaveMap)]
            except rospy.ROSException as ex:
                print ("WARNING: Service 'slam_toolbox/serialize_map' or 'slam_toolbox/save_map' is down")
    
        if (not pointCloudSaveSrvUp):
            try:
                rospy.wait_for_service("spatiotemporal_voxel_grid/save_grid", timeout=1.0)
                pointCloudSaveSrvUp = True
                save3dMap = rospy.ServiceProxy('spatiotemporal_voxel_grid/save_grid', SaveGrid)
            except rospy.ROSException as ex:
                print ("WARNING: Service 'spatiotemporal_voxel_grid/save_grid' is down")

        f.sleep()

        if (planarMapSaveSrvUp):
            try:
                resp = save2dMap[0](mapPath)
                print("PoseGraph saved with response: " + resp)
            except rospy.ServiceException as exc:
                print("WARNING: Service 'slam_toolbox/serialize_map' did not process request: " + str(exc))
            try:
                resp = save2dMap[1](String(mapPath))
                print("pgm saved with response: " + resp)
            except rospy.ServiceException as exc:
                print("WARNING: Service 'slam_toolbox/save_map' did not process request: " + str(exc))
        
        if (pointCloudSaveSrvUp):
            try:
                resp = save3dMap(String(mapPath))
                print("vdb saved with response: " + resp)
            except rospy.ServiceException as exc:
                print("WARNING: Service 'spatiotemporal_voxel_grid/save_grid' did not process request: " + str(exc))

    