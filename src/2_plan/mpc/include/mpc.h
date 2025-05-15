#ifndef ACADOS_OCP_NODE_HPP
#define ACADOS_OCP_NODE_HPP


#include <memory>
#include <array>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>

// Include Massages
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "utility/msg/trajectory.hpp"
#include "utility/msg/control.hpp"
#include "utility/msg/filtered_hall.hpp"
#include "utility/msg/fused_sensor.hpp"

// Include project headers
#include "point_msg_helper.hpp"
#include "marker_publisher.hpp"
#include "cubic_bezier.hpp"
#include "timer.hpp"

// Include acados headers
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados/ocp_nlp/ocp_nlp_sqp_rti.h"
#include "acados/ocp_nlp/ocp_nlp_common.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_bicycle_model.h"
#include "blasfeo_d_aux_ext_dep.h"

#define TS     0.021

struct Input {
    double steering_angle;
    double acceleration;
};

struct State {
    double s;
    double e_y;
    double e_theta;
    double v;
    double delta;
    double x;
    double y;
};

class AcadosOcpNode : public rclcpp::Node {
private:
    // Membervariable, um den acados OCP Solver-Handle zu speichern.
    // Der konkrete Typ hängt von deiner acados-Integration ab; hier als void* exemplarisch.
    bicycle_model_solver_capsule* ocp_capsule_;
    ocp_nlp_solver* ocp_nlp_solver_;
    ocp_nlp_out* ocp_nlp_out_;
    ocp_nlp_in* ocp_nlp_in_; 
    ocp_nlp_dims* ocp_nlp_dims_;
    ocp_nlp_config* ocp_nlp_config_;
    void* ocp_nlp_opts_;

    // Sensorwerte für v und delta, z. B. aus Parametern oder einem anderen Topic
    double sensor_v_;
    double sensor_delta_;

    std::vector<Input> inputs_;
    int pub_input_count_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool first_run_;

    // Subscriber
    rclcpp::Subscription<utility::msg::Trajectory>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<utility::msg::FusedSensor>::SharedPtr fused_sensor_sub_;
    rclcpp::Subscription<utility::msg::FilteredHall>::SharedPtr filtered_hall_sub_;

    // Publisher
    rclcpp::Publisher<utility::msg::Control>::SharedPtr input_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr control_points_marker_pub_;

public:
    AcadosOcpNode();
    ~AcadosOcpNode();
  
private:
    // Callbacks
    void trajectory_callback(const utility::msg::Trajectory::SharedPtr msg);
    void fused_sensor_callback(const utility::msg::FusedSensor::SharedPtr hall_ptr);
    void hall_callback(const utility::msg::FilteredHall::SharedPtr hall_ptr);

    // Hilfsfunktion: Setzt die Kontrollpunkte als Parameter im acados OCP Solver für alle Shooting-Nodes
    void set_ocp_parameters(const std::vector<geometry_msgs::msg::Point>& ctrl_points);

    // Initialisiert den OCP-Solver (z.B. durch einen Aufruf einer externen API)
    void initialize_ocp_solver();

    void prepare_ocp_solver();

    // Löst das OCP und gibt ggf. den neuen Zustandsvektor zurück
    void solve_ocp();

    void publish_input();

    Input get_input(int stage);
    State get_state(int stage);

    std::vector<Input> get_all_inputs();
    std::vector<State> get_all_states();
    std::vector<geometry_msgs::msg::Point> get_all_state_points();
    std::vector<geometry_msgs::msg::Point> get_ocp_parameters(std::vector<geometry_msgs::msg::Point>& trajectory);
    
    void reset_timer();

    // Prints the OCP solution
    void print_array(const double* values, int size, const char* description);

    std::string get_array_string(const double* values, int size, const char* description);

    std::vector<geometry_msgs::msg::Point> state2point(std::vector<State>& states);
};

#endif  // ACADOS_OCP_NODE_HPP