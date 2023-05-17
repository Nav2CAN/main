#include "context_aware_navigation/social_layer.hpp"
#include <math.h>
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace context_aware_navigation
{

    SocialLayer::SocialLayer(){}

    // This method is called at the end of plugin initialization.
    // It contains ROS parameter(s) declaration and initialization
    // of need_recalculation_ variable.
    void
    SocialLayer::onInitialize()
    {
        auto node = node_.lock();
        if (!node) {
            throw std::runtime_error{"Failed to lock node"};
        }

        declareParameter("enabled", rclcpp::ParameterValue(true));
        node->get_parameter(name_ + "." + "enabled", enabled_);

        base_frame = layered_costmap_->getGlobalFrameID();

        tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        social_map_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
        "/social_map", 10, std::bind(&SocialLayer::imageCallback, this, std::placeholders::_1));
        current_ = true;

    }

    // The method is called to ask the plugin: which area of costmap it needs to update.
    // Inside this method window bounds are re-calculated if need_recalculation_ is true
    // and updated independently on its value.
    void
    SocialLayer::updateBounds(
        double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double *min_x,
        double *min_y, double *max_x, double *max_y)
    {
        //figure out the zone where we want to draw the ellipse
        
        if (social_map_rotated.data != NULL)
        {
            float minx=0;
            float maxx=social_map_rotated.size().width;
            float miny=0;
            float maxy=social_map_rotated.size().height;

            if (minx < *min_x)
            {
                *min_x =minx;
            }
            if (maxx > *max_x)
            {
                *max_x =maxx;
            }
            if (miny < *min_y)
            {
                *min_y =miny;
            }
            if (maxy > *max_y)
            {
                *max_y =maxy;
            }

        }
        auto node = node_.lock();
        if (!node) {
            throw std::runtime_error{"Failed to lock node"};
        }

        RCLCPP_DEBUG(
            node->get_logger(),
            "Update bounds: Min x: %f Min y: %f Max x: %f Max y: %f", *min_x,
            *min_y, *max_x, *max_y);
            

    }
    void
    SocialLayer::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr message)
    {
        auto node = node_.lock();
        if (!node) {
            throw std::runtime_error{"Failed to lock node"};
        }
        cv_bridge::CvImagePtr cv_ptr;
        // convert iamge message
        try
        {
            // convert and update pointer with new image
            //  TODO the rotation from this is not critcal through time but the translation is so consider saving the timestamp of the message
            cv_ptr = cv_bridge::toCvCopy(message);
            // cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2GRAY); //save grayscale image
            // cv::Mat test=cv_ptr->image;
            // std::string fromFrameRel = message->header.frame_id.c_str();
            // std::string toFrameRel = base_frame.c_str();
            // geometry_msgs::msg::TransformStamped t;
            // // get transform between robot and map
            // try
            // {
            //     t = tf_buffer->lookupTransform(
            //         toFrameRel, fromFrameRel,
            //         tf2::TimePointZero);
            // }
            // catch (const tf2::TransformException &ex)
            // {
            //     return;
            // }
            // // get rpy
            // tf2::Quaternion q(
            //     t.transform.rotation.x,
            //     t.transform.rotation.y,
            //     t.transform.rotation.z,
            //     t.transform.rotation.w);
            // tf2::Matrix3x3 m(q);
            // double roll, pitch, yaw;
            // m.getRPY(roll, pitch, yaw);
            // // https://stackoverflow.com/questions/22041699/rotate-an-image-without-cropping-in-opencv-in-c
            // //  get rotation matrix for rotating the image around its center in pixel coordinates
            // cv::Point2f center((cv_ptr->image.cols - 1) / 2.0, (cv_ptr->image.rows - 1) / 2.0);
            // cv::Mat rot = cv::getRotationMatrix2D(center, -yaw * (180 / M_PI), 1.0);
            // // determine bounding rectangle, center not relevant
            // cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), cv_ptr->image.size(), -yaw * (180 / M_PI)).boundingRect2f();
            // // adjust transformation matrix
            // rot.at<double>(0, 2) += bbox.width / 2.0 - cv_ptr->image.cols / 2.0;
            // rot.at<double>(1, 2) += bbox.height / 2.0 - cv_ptr->image.rows / 2.0;
            // cv::warpAffine(cv_ptr->image, social_map_rotated, rot, bbox.size());
            // cv::flip(cv_ptr->image,social_map_rotated,0);
            cv::Mat temp=cv_ptr->image;
            temp.convertTo(temp, CV_8UC1);
            cv::flip(temp,temp,0);
            social_map_rotated = temp;
        }
        catch (cv_bridge::Exception &e)
        {
            return;
        }
    }


    // The method is called when costmap recalculation is required.
    // It updates the costmap within its window bounds.
    // Inside this method the costmap gradient is generated and is writing directly
    // to the resulting costmap master_grid without any merging with previous layers.
    void
    SocialLayer::updateCosts(
        nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j,
        int max_i,
        int max_j)
    {
        if (!enabled_)
        {
            return;
        }
        auto node = node_.lock();
        if (!node)
        {
            throw std::runtime_error{"Failed to lock node"};
        }
        setDefaultValue(nav2_costmap_2d::NO_INFORMATION);
        matchSize();
        uint8_t *costmap_array = getCharMap();
        unsigned int size_x = getSizeInCellsX(), size_y = getSizeInCellsY();

        min_i = 0;
        min_j = 0;
        max_i = size_x;
        max_j = size_y;

        // if (social_map_rotated.data!=NULL){
                        // auto test = social_map_rotated.at<uchar>(center_image_j - (center_layer_j - j),center_image_i - (center_layer_i - i));

        if (social_map_rotated.data != NULL){
            for (int j = min_j; j < max_j; j++) {
                for (int i = min_i; i < max_i; i++) {
                    try{
                    int index = getIndex(i, j);

                    costmap_array[index] = static_cast<uint8_t>(social_map_rotated.at<uchar>(j,i));
            // This combin      es the master costmap with the current costmap by taking
            // the max across all costmaps.
                    }
                    catch(std::exception &e){return;}
                }
            }
        }
        updateWithMax(master_grid, min_i, min_j, max_i, max_j);
        current_ = true;

    }
} // namespace nav2_gradient_costmap_plugin

// This is the macro allowing a nav2_gradient_costmap_plugin::GradientLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(context_aware_navigation::SocialLayer, nav2_costmap_2d::Layer)