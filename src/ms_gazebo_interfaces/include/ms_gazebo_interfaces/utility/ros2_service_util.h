#ifndef ROS2_SERVICE_ULIL_H_
#define ROS2_SERVICE_ULIL_H_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <list>
using namespace std;

// *** ROS include
#include <rclcpp/rclcpp.hpp>

// a convient function to send request using ros_service
template <typename srv_client, typename srv_request>
bool ros_service_send_request(const std::shared_ptr<rclcpp::Node>& node, const srv_client& client, const srv_request& request)
{
    // *** Wait the service
    string srv_name = client->get_service_name();
    if (!client->wait_for_service(200ms)) {
        RCLCPP_ERROR(node->get_logger(), "Service %s does not exit", srv_name.c_str());
        return false;
    }
    // Send request.
    auto result = client->async_send_request(request);
    // Wait for the result.
    double time = 0;
    while(rclcpp::ok()){
        auto state = result.wait_for(std::chrono::seconds(0));
        if(state == std::future_status::ready){
            RCLCPP_INFO(node->get_logger(), "Request to %s is Successfully respond", srv_name.c_str());
            if(result.get()->success){
                RCLCPP_INFO(node->get_logger(), "Result from %s id Success", srv_name.c_str());
                return true;
            }else{
                RCLCPP_ERROR(node->get_logger(), "Result from %s is Failure", srv_name.c_str());
                return true;
            }
            
        }
        else if(state == std::future_status::timeout) {
            RCLCPP_INFO(node->get_logger(), "Request to %s  Waiting", srv_name.c_str());
        }
        else if(state == std::future_status::deferred) {
            RCLCPP_ERROR(node->get_logger(), "Request to %s  Deferred", srv_name.c_str());
            return false;
        }
        if(time > 0.5){
            RCLCPP_ERROR(node->get_logger(), "Request to %s Time out", srv_name.c_str());
            return false;
        }
        rclcpp::sleep_for(100ms);
        time += 0.1;
    }
    return false;
}

# endif //ROS2_SERVICE_ULIL_H_
