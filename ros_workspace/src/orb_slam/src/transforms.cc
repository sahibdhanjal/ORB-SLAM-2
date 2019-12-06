#include <cstdio>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_broadcaster");
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    // handle static transforms
    geometry_msgs::TransformStamped world_to_map_static;
    world_to_map_static.header.stamp = ros::Time::now();
    world_to_map_static.header.frame_id = "world";
    world_to_map_static.child_frame_id = "map";
    world_to_map_static.transform.translation.x = 0;
    world_to_map_static.transform.translation.y = 0;
    world_to_map_static.transform.translation.z = 0;
    world_to_map_static.transform.rotation.x = 0;
    world_to_map_static.transform.rotation.y = 0;
    world_to_map_static.transform.rotation.z = 0;
    world_to_map_static.transform.rotation.w = 1;
    static_broadcaster.sendTransform(world_to_map_static);

    // TODO (anyone): handle dynamic transforms
    /**
     * get the transform between the frame of the car and the camera
     * and implement a transform broadcaster between map and camera.
     *
     * The final transform would look as follows:
     *                     car_frame (wheel odometry will be in this frame)
     *                   /
     *      world -> map
     *                  \
     *                   camera (and other sensors) (orb slam odometry is this frame)
     */

    ros::spin();
    return 0;
}
