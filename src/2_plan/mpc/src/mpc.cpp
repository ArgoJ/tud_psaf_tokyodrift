#include "mpc.h"


AcadosOcpNode::AcadosOcpNode()
: Node("mpc"), ocp_capsule_(nullptr) {
    this->initialize_ocp_solver();

    // Subscriber
    trajectory_sub_ = this->create_subscription<utility::msg::Trajectory>(
        "tokyodrift/plan/transformed_lane", 10,
        std::bind(&AcadosOcpNode::trajectory_callback, this, std::placeholders::_1)
    );

    // Publisher
    this->trajectory_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "tokyodrift/plan/mpc/trajectory_marker", 3
    ); 
    this->control_points_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "tokyodrift/plan/mpc/control_points_marker", 3
    );
}

AcadosOcpNode::~AcadosOcpNode() {
    // free solver
    int status;
    status = bicycle_model_acados_free(this->ocp_capsule_);
    if (status) {
        RCLCPP_ERROR(this->get_logger(), "bicycle_model_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = bicycle_model_acados_free_capsule(this->ocp_capsule_);
    if (status) {
        RCLCPP_ERROR(this->get_logger(), "bicycle_model_acados_free_capsule() returned status %d. \n", status);
    }
}

void AcadosOcpNode::trajectory_callback(const utility::msg::Trajectory::SharedPtr msg) {
    if (this->ocp_capsule_) {
        // std::vector<geometry_msgs::msg::Point> trajectory = msg->points;
        auto control_points = this->get_ocp_parameters(msg->points);
    
        if (!control_points.empty()) {
            this->set_ocp_parameters(control_points);
            this->solve_ocp();

            publish_marker_points(
                control_points,
                this->control_points_marker_pub_,
                "map",
                this->get_clock(),
                Color{1.0, 0.2, 0.6},
                "mpc_parameter_points",
                1,
                0.03,
                1.0,
                visualization_msgs::msg::Marker::SPHERE_LIST
            );

            auto states = this->get_all_states();
            publish_marker_points(
                states,
                this->trajectory_marker_pub_,
                "map",
                this->get_clock(),
                Color{0.3, 1.0, 0.2},
                "mpc_states",
                1,
                0.03,
                1.0,
                visualization_msgs::msg::Marker::LINE_STRIP
            );
        } else {
            RCLCPP_ERROR(this->get_logger(), "No control points found!"); 
        }
        
    } else {
        RCLCPP_ERROR(this->get_logger(), "Solver not initialisied");    
        this->initialize_ocp_solver();
    }
}

std::vector<geometry_msgs::msg::Point> AcadosOcpNode::get_ocp_parameters(std::vector<geometry_msgs::msg::Point> &trajectory) {
    double distance_fraction = 0.3;
    geometry_msgs::msg::Point zero_point = make_point(0.0, 0.0);
    geometry_msgs::msg::Point zero_unit_grad = make_point(1.0, 0.0);

    auto [target, idx] = select_optimal_target(
        trajectory, zero_point, zero_unit_grad, distance_fraction, 3.0, 1.3, 3.0, 1.0
    );

    geometry_msgs::msg::Point negative_unit_gradient;
    if (idx < trajectory.size() - 1) {
        negative_unit_gradient = get_unit_gradient(trajectory[idx + 1], target);
    } else if (idx == trajectory.size() - 1) {
        negative_unit_gradient = get_unit_gradient(target, trajectory[idx - 1]);
    }

    return get_control_points(
        zero_point, zero_unit_grad, target, negative_unit_gradient, distance_fraction
    );
}

void AcadosOcpNode::set_ocp_parameters(const std::vector<geometry_msgs::msg::Point>& ctrl_points) {
    std::array<double, NP> ctrl_points_arr = {
        ctrl_points[0].x, ctrl_points[0].y, 
        ctrl_points[1].x, ctrl_points[1].y,
        ctrl_points[2].x, ctrl_points[2].y,
        ctrl_points[3].x, ctrl_points[3].y,
    };
    // std::array<double, NP> ctrl_points_arr = {
    //     0.0,0.0, 
    //     0.6,0.0, 
    //     1.0,0.5, 
    //     1.8,-0.4
    // };

    // Für jeden Zeitschritt k die Parameter updaten
    int status = 0;
    for (int k = 0; k < N; ++k) {
        status = bicycle_model_acados_update_params(
            this->ocp_capsule_, 
            k, 
            const_cast<double*>(ctrl_points_arr.data()), 
            static_cast<int>(ctrl_points_arr.size())
        );
    }
}

void AcadosOcpNode::initialize_ocp_solver() {
    // Capsule allozieren
    this->ocp_capsule_ = bicycle_model_acados_create_capsule();
    if (!ocp_capsule_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to allocate solver capsule");
        rclcpp::shutdown();
        return;
    }
    // Solver initialisieren (JSON muss im Arbeitsverzeichnis liegen)
    int status = bicycle_model_acados_create(ocp_capsule_, "src/2_plan/mpc/bicycle_model.json");
    if (status) {
        RCLCPP_ERROR(this->get_logger(), "Creating model failed with status %d", status);
        rclcpp::shutdown();
    } else {
        RCLCPP_INFO(this->get_logger(), "Acados OCP Solver initialisiert");
    }

    this->ocp_nlp_config_ = bicycle_model_acados_get_nlp_config(this->ocp_capsule_);
    this->ocp_nlp_dims_ = bicycle_model_acados_get_nlp_dims(this->ocp_capsule_);
    this->ocp_nlp_in_ = bicycle_model_acados_get_nlp_in(this->ocp_capsule_);
    this->ocp_nlp_out_ = bicycle_model_acados_get_nlp_out(this->ocp_capsule_);
    this->ocp_nlp_solver_ = bicycle_model_acados_get_nlp_solver(this->ocp_capsule_);
}

void AcadosOcpNode::solve_ocp() {
    // Initial OCP in- and outputs
    double x_init[NX] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double u0[NU] = {0.0, 0.0};
    for (int i = 0; i < N; i++) {
        ocp_nlp_out_set(this->ocp_nlp_config_, this->ocp_nlp_dims_, this->ocp_nlp_out_, i, "x", x_init);
        ocp_nlp_out_set(this->ocp_nlp_config_, this->ocp_nlp_dims_, this->ocp_nlp_out_, i, "u", u0);
    }

    // Initial OCP-Bounds
    double bx0[NX] = {0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0};
    ocp_nlp_constraints_model_set(this->ocp_nlp_config_, this->ocp_nlp_dims_, this->ocp_nlp_in_, 0, "lbx", bx0);
    ocp_nlp_constraints_model_set(this->ocp_nlp_config_, this->ocp_nlp_dims_, this->ocp_nlp_in_, 0, "ubx", bx0);

    // Solve OCP
    int status = bicycle_model_acados_solve(ocp_capsule_);
    if (status != 0) {
        RCLCPP_ERROR(this->get_logger(), "acados_solve failed: %d", status);
        return;
    }

    // get optimal input
    this->get_input(0, u0);
    this->print_array(u0, NU, "Optimal input");
}

void AcadosOcpNode::get_input(int stage, void* values) {
    ocp_nlp_out_get(this->ocp_nlp_config_, this->ocp_nlp_dims_, this->ocp_nlp_out_, stage, "u", values);
}

void AcadosOcpNode::get_state(int stage, void* values) {
    ocp_nlp_out_get(this->ocp_nlp_config_, this->ocp_nlp_dims_, this->ocp_nlp_out_, stage, "x", values);
}

std::vector<geometry_msgs::msg::Point> AcadosOcpNode::get_all_states() {
    std::vector<geometry_msgs::msg::Point> states;
    states.reserve(N);
    for (int i = 0; i < N; i++) {
        double values[NX];
        this->get_state(i, values);
        geometry_msgs::msg::Point point = make_point(values[5], values[6]);
        states.push_back(point);

        std::string str = "State " + std::to_string(i);
        this->print_array(values, NX, str.c_str());
    }
    return states;
}

void AcadosOcpNode::print_array(const double* values, int size, const char* description) {
    std::string str = this->get_array_string(values, size, description);
    RCLCPP_DEBUG(this->get_logger(), "%s", str.c_str());
}

std::string AcadosOcpNode::get_array_string(const double* values, int size, const char* description) {
    std::ostringstream oss;
    oss << description << ": [";
    for (int i = 0; i < size; ++i) {
        oss << values[i];
        if (i < size - 1) oss << ", ";
    }
    oss << "]";
    return oss.str();
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AcadosOcpNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}