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

    SocialLayer::SocialLayer()
        : last_min_x_(-std::numeric_limits<float>::max()),
          last_min_y_(-std::numeric_limits<float>::max()),
          last_max_x_(std::numeric_limits<float>::max()),
          last_max_y_(std::numeric_limits<float>::max())
    {
    }

    // This method is called at the end of plugin initialization.
    // It contains ROS parameter(s) declaration and initialization
    // of need_recalculation_ variable.
    void
    SocialLayer::onInitialize()
    {
        auto node = node_.lock();
        declareParameter("enabled", rclcpp::ParameterValue(true));
        node->get_parameter(name_ + "." + "enabled", enabled_);

        need_recalculation_ = false;
        current_ = true;

        target_frame = node->declare_parameter<std::string>("target_frame", "map");
        base_frame = layered_costmap_->getGlobalFrameID();

        tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    }

    // The method is called to ask the plugin: which area of costmap it needs to update.
    // Inside this method window bounds are re-calculated if need_recalculation_ is true
    // and updated independently on its value.
    void
    SocialLayer::updateBounds(
        double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double *min_x,
        double *min_y, double *max_x, double *max_y)
    {
        if (need_recalculation_)
        {
            last_min_x_ = *min_x;
            last_min_y_ = *min_y;
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;
            // For some reason when I make these -<double>::max() it does not
            // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
            // -<float>::max() instead.
            *min_x = -std::numeric_limits<float>::max();
            *min_y = -std::numeric_limits<float>::max();
            *max_x = std::numeric_limits<float>::max();
            *max_y = std::numeric_limits<float>::max();
            need_recalculation_ = false;
        }
        else
        {
            double tmp_min_x = last_min_x_;
            double tmp_min_y = last_min_y_;
            double tmp_max_x = last_max_x_;
            double tmp_max_y = last_max_y_;
            last_min_x_ = *min_x;
            last_min_y_ = *min_y;
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;
            *min_x = std::min(tmp_min_x, *min_x);
            *min_y = std::min(tmp_min_y, *min_y);
            *max_x = std::max(tmp_max_x, *max_x);
            *max_y = std::max(tmp_max_y, *max_y);
        }
    }
    void
    SocialLayer::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr message)
    {
        //convert iamge message
        try
        {
            //convert and update pointer with new image
            // TODO the rotation from this is not critcal through time but the translation is so consider saving the timestamp of the message
            social_map = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            return;
        }
    }



    // The method is called when footprint was changed.
    // Here it just resets need_recalculation_ variable.
    void
    SocialLayer::onFootprintChanged()
    {
        need_recalculation_ = true;

        RCLCPP_DEBUG(rclcpp::get_logger(
                         "nav2_costmap_2d"),
                     "SocialLayer::onFootprintChanged(): num footprint points: %lu",
                     layered_costmap_->getFootprint().size());
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
          if (!node) {
            throw std::runtime_error{"Failed to lock node"};
        }
        setDefaultValue(nav2_costmap_2d::NO_INFORMATION);
        matchSize();
        uint8_t * costmap_array = getCharMap();
        unsigned int size_x = getSizeInCellsX(), size_y = getSizeInCellsY();

        min_i = std::max(0, min_i);
        min_j = std::max(0, min_j);
        max_i = std::min(static_cast<int>(size_x), max_i);
        max_j = std::min(static_cast<int>(size_y), max_j);

        // todo handle cropping down of arraysizes
        costmap_array = social_map_rotated->data;
        // This combines the master costmap with the current costmap by taking
        // the max across all costmaps.
        updateWithMax(master_grid, min_i, min_j, max_i, max_j);
        current_ = true;
    }

    void SocialLayer::rotateImage()
    {

        //rotate the map to the latest location of the robot and create pointer to rotated map

        std::string fromFrameRel = target_frame.c_str();
        std::string toFrameRel = base_frame.c_str();
        geometry_msgs::msg::TransformStamped t;
            //get transform between robot and map
        try {
        t = tf_buffer->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {

        return;
        }

        //get rpy
        tf2::Quaternion q(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        //https://stackoverflow.com/questions/22041699/rotate-an-image-without-cropping-in-opencv-in-c
        // get rotation matrix for rotating the image around its center in pixel coordinates
        cv::Point2f center((social_map->image.cols-1)/2.0, (social_map->image.rows-1)/2.0);
        cv::Mat rot = cv::getRotationMatrix2D(center, -yaw*(180/M_PI), 1.0);
        // determine bounding rectangle, center not relevant
        cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), social_map->image.size(),-yaw*(180/M_PI)).boundingRect2f();
        // adjust transformation matrix
        rot.at<double>(0,2) += bbox.width/2.0 - social_map->image.cols/2.0;
        rot.at<double>(1,2) += bbox.height/2.0 - social_map->image.rows/2.0;
        cv::Mat dst;
        cv::warpAffine(social_map->image, dst, rot, bbox.size());
        social_map_rotated = std::make_shared<cv::Mat>(dst);
    }
} // namespace nav2_gradient_costmap_plugin




// This is the macro allowing a nav2_gradient_costmap_plugin::GradientLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(context_aware_navigation::SocialLayer, nav2_costmap_2d::Layer)