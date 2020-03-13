#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <interactive_markers/menu_handler.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <sys/stat.h>

using namespace Eigen;

struct COLOR {
    COLOR(float r, float g, float b, float a) : r(r), g(g), b(b), a(a) {};
    void randColor(){
        r = 1.0f-(rand() / ((float)(RAND_MAX)));
        g = 1.0f-(rand() / ((float)(RAND_MAX)));
        b = 1.0f-(rand() / ((float)(RAND_MAX)));
    };
    float r, g, b, a;
};

using namespace visualization_msgs;
using std::string;

class rviz_visualization {
public:
    rviz_visualization();

    ~rviz_visualization();

    static void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    void initializeInteractiveMarkerServer();

    Marker makeBox(InteractiveMarker &msg);

    InteractiveMarkerControl &makeBoxControl(InteractiveMarker &msg);

    void make6DofMarker(bool fixed, unsigned int interaction_mode, const tf::Vector3 &position,
                        bool show_6dof, double scale = 1, const char *frame = "world",
                        const char *name = "interactive_marker",
                        const char *description = "for interaction and shit");

    Vector3d convertGeometryToEigen(const geometry_msgs::Vector3 &vector_in);

    geometry_msgs::Vector3 convertEigenToGeometry(const Vector3d &vector_in);

    void PoseMsgToTF(const geometry_msgs::Pose &msg, tf::Transform &bt);

    /**
     * Publishes a mesh visualization marker
     * @param package ros package in which this mesh is located
     * @param relative_path the relative path inside that ros packae
     * @param the mesh file name
     * @param pos at this position
     * @param orientation with this orientation
     * @param modelname name of the mesh.dae
     * @param frame in this frame
     * @param ns namespace
     * @param message_id unique id
     * @param duration in seconds
     * @param color of the mesh
     */
    void publishMesh(const char *package, const char *relative_path, const char *modelname, Vector3d &pos,
                     Quaterniond &orientation,
                     double scale, const char *frame, const char *ns, int message_id, double duration = 0,
                     COLOR color = COLOR(1, 1, 1, 1), bool update = false);

    /**
     * Publishes a mesh visualization marker
     * @param package ros package in which this mesh is located
     * @param relative_path the relative path inside that ros packae
     * @param the mesh file name
     * @param pose of mesh
     * @param modelname name of the mesh.dae
     * @param frame in this frame
     * @param ns namespace
     * @param message_id unique id
     * @param duration in seconds
     * @param color of the mesh
     */
    void publishMesh(const char *package, const char *relative_path, const char *modelname, geometry_msgs::Pose &pose,
                     double scale, const char *frame, const char *ns, int message_id, double duration = 0,
                     COLOR color = COLOR(1, 1, 1, 1), bool update = false);

    /**
     * Publishes a sphere visualization marker
     * @param pos at this positon
     * @param frame in this frame
     * @param ns namespace
     * @param message_id a unique id
     * @param rgda rgb color (0-1) plus transparancy
     * @param duration for this duration in seconds (0=forever)
     */
    void
    publishSphere(Vector3d &pos, const char *frame, const char *ns, int message_id, COLOR color, float radius = 0.01,
                  double duration = 0);

    /**
     * Publishes a sphere visualization marker
     * @param pose at this positon
     * @param frame in this frame
     * @param ns namespace
     * @param message_id a unique id
     * @param rgda rgb color (0-1) plus transparancy
     * @param duration for this duration in seconds (0=forever)
     */
    void publishSphere(geometry_msgs::Pose &pose, const char *frame, const char *ns, int message_id, COLOR color,
                       float radius = 0.01, double duration = 0);

    /**
     * Publishes a cube visualization marker
     * @param pos at this positon
     * @param quat with this orientation
     * @param frame in this frame
     * @param ns namespace
     * @param message_id a unique id
     * @param rgda rgb color (0-1) plus transparancy
     * @param duration for this duration in seconds (0=forever)
     */
    void publishCube(Vector3d &pos, Vector4d &quat, const char *frame, const char *ns, int message_id, COLOR color,
                     float radius = 0.01, double duration = 0);

    /**
     * Publishes a cube visualization marker
     * @param pose with this pose
     * @param frame in this frame
     * @param ns namespace
     * @param message_id a unique id
     * @param rgda rgb color (0-1) plus transparancy
     * @param duration for this duration in seconds (0=forever)
     */
    void publishCube(geometry_msgs::Pose &pose, const char *frame, const char *ns, int message_id, COLOR color,
                     float radius = 0.01, double duration = 0);

    /**
     * Publishes a cube visualization marker
     * @param pos position
     * @param quat quaternion
     * @param frame in this frame
     * @param ns namespace
     * @param message_id a unique id
     * @param rgda rgb color (0-1) plus transparancy
     * @param dx cube x dim
     * @param dy cube y dim
     * @param dz cube z dim
     * @param duration for this duration in seconds (0=forever)
     */
    void publishCube(Vector3d &pos, Quaternionf &quat, const char *frame, const char *ns, int message_id, COLOR color,
                     float dx = 0.01, float dy = 0.01, float dz = 0.01, double duration = 0);

    /**
     * Publishes a cylinder visualization marker
     * @param pos at this positon
     * @param frame in this frame
     * @param ns namespace
     * @param message_id a unique id
     * @param rgda rgb color (0-1) plus transparancy
     * @param duration for this duration in seconds (0=forever)
     */
    void
    publishCylinder(Vector3d &pos, const char *frame, const char *ns, int message_id, COLOR color, float radius = 0.01,
                    double duration = 0);

    /**
     * Publishes a ray visualization marker
     * @param pos at this positon
     * @param dir direction
     * @param frame in this frame
     * @param message_id a unique id
     * @param ns namespace
     * @param color rgb color (0-1) plus transparancy
     * @param duration for this duration in seconds (0=forever)
     */
    void publishRay(Vector3d &pos, Vector3d &dir, const char *frame, const char *ns, int message_id, COLOR color,
                    double duration = 0);

    /**
     * Publishes a text message marker
     * @param pos at this positon
     * @param text with this text
     * @param frame in this frame
     * @param ns namespace
     * @param message_id a unique id
     * @param color rgb color (0-1) plus transparancy
     * @param duration for this duration in seconds (0=forever)
     * @param height height of the text
     */
    void publishText(Vector3d &pos, const char *text, const char *frame, const char *ns, int message_id, COLOR color,
                     double duration, float height);

    /**
     * Clears a marker with this id
     */
    void clearMarker(int id);

    /**
     * Clears all markers in rviz
     */
    void clearAll();

    /**
     * Gets a tf transform
     * @param from source frame
     * @param to target frame
     * @param transform will be filled with the transform
     * @return success
     */
    bool getTransform(string from, string to, geometry_msgs::Pose &transform);

    /**
     * Gets a tf transform
     * @param from source frame
     * @param to target frame
     * @param transform will be filled with the transform
     * @return success
     */
    bool getTransform(string from, string to, Matrix4d &transform);

    /**
     * Queries the tf listener for the specified transform
     * @param lighthouse
     * @param to another frame
     * @param transform the transform if available
     * @return true if available
     */
    bool getLighthouseTransform(bool lighthouse, const char *to, Matrix4d &transform);

    /**
     * Queries the tf listener for the specified transform
     * @param lighthouse
     * @param from another frame
     * @param transform the transform if available
     * @return true if available
     */
    bool getLighthouseTransform(const char *from, bool lighthouse, Matrix4d &transform);

    /**
     * Queries the tf listener for the specified transform
     * @param to this frame
     * @param from another frame
     * @param transform the transform if available
     * @return true if available
     */
    bool getTransform(const char *from, const char *to, tf::Transform &transform);

    /**
     * Publishes a tf transform
     * @param from source frame
     * @param to target frame
     * @param transform
     * @return success
     */
    bool publishTransform(string from, string to, geometry_msgs::Pose &transform);

private:
    ros::NodeHandlePtr nh;
    static boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server;
    static interactive_markers::MenuHandler menu_handler;
    static bool first;
    visualization_msgs::MarkerArray marker_array;
    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;
public:
    ros::Publisher visualization_pub, visualization_array_pub;
    bool publish_as_marker_array = false;
    int number_of_markers_to_publish_at_once = 100;
};