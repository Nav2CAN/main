#include "context_aware_navigation/interaction_layer.hpp"
#include <math.h>
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace context_aware_navigation
{

    InteractionLayer::InteractionLayer()
    {
    }

    // This method is called at the end of plugin initialization.
    // It contains ROS parameter(s) declaration and initialization
    // of need_recalculation_ variable.
    void
    InteractionLayer::onInitialize()
    {
        auto node = node_.lock();
        declareParameter("enabled", rclcpp::ParameterValue(true));
        node->get_parameter(name_ + "." + "enabled", enabled_);



        interaction_cost = node->declare_parameter<float>("interaction_map_size", 125.0);

        tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        current_ = true;
    }

    // The method is called to ask the plugin: which area of costmap it needs to update.
    // Inside this method window bounds are re-calculated if need_recalculation_ is true
    // and updated independently on its value.
    void
    InteractionLayer::updateBounds(
        double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double *min_x,
        double *min_y, double *max_x, double *max_y)
    {
        if (BoundingBox != nullptr)
        {
            if (BoundingBox->center_x-BoundingBox->width/2< *min_x)
            {
                *min_x =BoundingBox->center_x-BoundingBox->width/2;
            }
            if (BoundingBox->center_x+BoundingBox->width/2> *max_x)
            {
                *max_x =BoundingBox->center_x+BoundingBox->width/2;
            }
            if (BoundingBox->center_y-BoundingBox->height/2< *min_y)
            {
                *min_y =BoundingBox->center_x-BoundingBox->height/2;
            }
            if (BoundingBox->center_y+BoundingBox->height/2> *max_y)
            {
                *max_y =BoundingBox->center_y+BoundingBox->height/2;
            }

        }
    }
    void
    InteractionLayer::interactionCallback(
      multi_person_tracker_interfaces::msg::BoundingBox::ConstSharedPtr message)
    {
        BoundingBox=message;
    }

    // The method is called when footprint was changed.
    // Here it just resets need_recalculation_ variable.
    void
    InteractionLayer::onFootprintChanged()
    {
        need_recalculation_ = true;

        RCLCPP_DEBUG(rclcpp::get_logger(
                         "nav2_costmap_2d"),
                     "InteractionLayer::onFootprintChanged(): num footprint points: %lu",
                     layered_costmap_->getFootprint().size());
    }

    // The method is called when costmap recalculation is required.
    // It updates the costmap within its window bounds.
    // Inside this method the costmap gradient is generated and is writing directly
    // to the resulting costmap master_grid without any merging with previous layers.
    void
    InteractionLayer::updateCosts(
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

        min_i = std::max(0, min_i);
        min_j = std::max(0, min_j);
        max_i = std::min(static_cast<int>(size_x), max_i);
        max_j = std::min(static_cast<int>(size_y), max_j);

        // center of costmap layer
        
        uint x1,y1;

        //get coordinates of interaction in the costmap
        bool valid=worldToMap(BoundingBox->center_x,BoundingBox->center_y,x1,y1);

        if (valid==true){
            cv::Mat cv_costmap = cv::Mat(size_y,size_x,CV_8UC1, costmap_array);//make a cv::Mat from the current costmap
            cv::Point center = cv::Point(x1,y1);
            cv::Point size = cv::Size(BoundingBox->width,BoundingBox->height);
            cv::ellipse(cv_costmap,center,size,0,0,360,interaction_cost,-1);//draw interaction as ellipse

            costmap_array=cv_costmap.data;//write the array to the costmap
            updateWithMax(master_grid, min_i, min_j, max_i, max_j); //update with max
            current_ = true;
        }

    }
} // namespace nav2_gradient_costmap_plugin

// This is the macro allowing a nav2_gradient_costmap_plugin::GradientLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(context_aware_navigation::InteractionLayer, nav2_costmap_2d::Layer)