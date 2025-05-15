#include "mpc.h"


AcadosOcpNode::AcadosOcpNode()
: Node("mpc"), ocp_capsule_(nullptr) {
    this->initialize_ocp_solver();

    // Subscriber
    trajectory_sub_ = this->create_subscription<utility::msg::Trajectory>(
        "tokyodrift/plan/transformed_lane", 1,
        std::bind(&AcadosOcpNode::trajectory_callback, this, std::placeholders::_1)
    );
    this->fused_sensor_sub_ = this->create_subscription<utility::msg::FusedSensor>(
        "fused_sensor", 1,
        std::bind(&AcadosOcpNode::fused_sensor_callback, this, std::placeholders::_1)
    );
    this->filtered_hall_sub_ = this->create_subscription<utility::msg::FilteredHall>(
        "filtered_hall", 1,
        std::bind(&AcadosOcpNode::hall_callback, this, std::placeholders::_1)
    );

    // Publisher
    this->input_pub_ = this->create_publisher<utility::msg::Control>(
        "tokyodrift/plan/control", 3
    ); 
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
        START_TIMER("Get Parameters")
        auto control_points = this->get_ocp_parameters(msg->points);
        STOP_TIMER("Get Parameters")
    
        if (!control_points.empty()) {
            START_TIMER("Set Parameter")
            this->set_ocp_parameters(control_points);
            STOP_TIMER("Set Parameter")

            // Prepare OCP solution
            if (this->first_run_) {
                START_TIMER("Preperation")
                this->prepare_ocp_solver();
                STOP_TIMER("Preperation")
                this->first_run_ = false;
            }

            START_TIMER("Solve OCP")
            this->solve_ocp();
            STOP_TIMER("Solve OCP")

            // Publish control input
            this->pub_input_count_ = 0;
            this->publish_input();

            START_TIMER("Reset Timer")
            this->reset_timer();
            STOP_TIMER("Reset Timer")

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

            std::vector<geometry_msgs::msg::Point> state_points = this->get_all_state_points();
            publish_marker_points(
                state_points,
                this->trajectory_marker_pub_,
                "map",
                this->get_clock(),
                Color{0.3, 1.0, 0.2},
                "mpc_states",
                4,
                0.03,
                0.7,
                visualization_msgs::msg::Marker::LINE_STRIP
            );

            // Prepare the next OCP solution
            START_TIMER("Preperation")
            this->prepare_ocp_solver();
            STOP_TIMER("Preperation")
        } else {
            RCLCPP_ERROR(this->get_logger(), "No control points found!"); 
        }
        
    } else {
        RCLCPP_ERROR(this->get_logger(), "Solver not initialisied");    
        this->initialize_ocp_solver();
    }
}

void AcadosOcpNode::fused_sensor_callback(const utility::msg::FusedSensor::SharedPtr fused_sensor_ptr) {
    this->sensor_delta_ = fused_sensor_ptr->delta;
}

void AcadosOcpNode::hall_callback(const utility::msg::FilteredHall::SharedPtr hall_ptr) {
    this->sensor_v_ = hall_ptr->longitudinal_velocity;
}

void AcadosOcpNode::set_ocp_parameters(const std::vector<geometry_msgs::msg::Point>& ctrl_points) {
    std::array<double, BICYCLE_MODEL_NP> ctrl_points_arr = {
        ctrl_points[0].x, ctrl_points[0].y, 
        ctrl_points[1].x, ctrl_points[1].y,
        ctrl_points[2].x, ctrl_points[2].y,
        ctrl_points[3].x, ctrl_points[3].y,
    };

    // Für jeden Zeitschritt k die Parameter updaten
    int status = 0;
    for (int k = 0; k < BICYCLE_MODEL_N; ++k) {
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
    this->first_run_ = true;
    this->ocp_capsule_ = bicycle_model_acados_create_capsule();
    if (!ocp_capsule_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to allocate solver capsule");
        rclcpp::shutdown();
        return;
    }
    // Solver initialisieren (JSON muss im Arbeitsverzeichnis liegen)
    int status = bicycle_model_acados_create(ocp_capsule_);
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
    this->ocp_nlp_opts_ = bicycle_model_acados_get_nlp_opts(this->ocp_capsule_);
}

void AcadosOcpNode::prepare_ocp_solver() {
    const char* field = "rti_phase";
    int prep_value = PREPARATION;
    ocp_nlp_sqp_rti_opts_set(this->ocp_nlp_config_, this->ocp_nlp_opts_, field, &prep_value);
    // Solve OCP
    int status = bicycle_model_acados_solve(ocp_capsule_);
    if (status != 0  && status != 5) {
        RCLCPP_ERROR(this->get_logger(), "Solver failed at preperation phase: %d", status);
        return;
    }
}

void AcadosOcpNode::solve_ocp() {    
    double bx0[BICYCLE_MODEL_NX] = {
        0.0, 
        0.0, 
        0.0, 
        this->sensor_v_, 
        this->sensor_delta_, 
        0.0, 
        0.0
    };
    ocp_nlp_constraints_model_set(this->ocp_nlp_config_, this->ocp_nlp_dims_, this->ocp_nlp_in_, this->ocp_nlp_out_, 0, "lbx", bx0);
    ocp_nlp_constraints_model_set(this->ocp_nlp_config_, this->ocp_nlp_dims_, this->ocp_nlp_in_, this->ocp_nlp_out_, 0, "ubx", bx0);
    
    // Solve OCP
    const char* field = "rti_phase";
    int feedb_value = FEEDBACK;
    ocp_nlp_sqp_rti_opts_set(this->ocp_nlp_config_, this->ocp_nlp_opts_, field, &feedb_value);
    int status = bicycle_model_acados_solve(ocp_capsule_);
    if (status != 0) {
        RCLCPP_ERROR(this->get_logger(), "Solver failed in feedback phase: %d", status);
        return;
    }
}

void AcadosOcpNode::publish_input() {
    if (this->pub_input_count_ >= BICYCLE_MODEL_N) {
        this->timer_->cancel();
        RCLCPP_ERROR(this->get_logger(), "No inputs available to publish");
        return;
    }
    Input input = this->get_input(this->pub_input_count_);

    utility::msg::Control control_msg;
    control_msg.delta = input.steering_angle;
    control_msg.longitudinal_control = input.acceleration;
    control_msg.header.stamp = this->get_clock()->now();

    this->input_pub_->publish(control_msg);
    RCLCPP_DEBUG(this->get_logger(), "Published input: {delta=%f, a=%f}", control_msg.delta, control_msg.longitudinal_control);
    this->pub_input_count_++;
}

Input AcadosOcpNode::get_input(int stage) {
    double values[BICYCLE_MODEL_NU];
    ocp_nlp_out_get(this->ocp_nlp_config_, this->ocp_nlp_dims_, this->ocp_nlp_out_, stage, "u", values);
    return {values[0], values[1]};
}

State AcadosOcpNode::get_state(int stage) {
    double values[BICYCLE_MODEL_NX];
    ocp_nlp_out_get(this->ocp_nlp_config_, this->ocp_nlp_dims_, this->ocp_nlp_out_, stage, "x", values);
    return {values[0], values[1], values[2], values[3], values[4], values[5], values[6]};
}

std::vector<Input> AcadosOcpNode::get_all_inputs() {
    std::vector<Input> inputs;
    for (int i = 0; i < BICYCLE_MODEL_N; i++) {
        Input input = this->get_input(i);
        inputs.push_back(input);
    }
    return inputs;
}

std::vector<State> AcadosOcpNode::get_all_states() {
    std::vector<State> states;
    states.reserve(BICYCLE_MODEL_N+1);
    for (int i = 0; i <= BICYCLE_MODEL_N; i++) {
        State state = this->get_state(i);
        states.push_back(state);
    }
    return states;
}

std::vector<geometry_msgs::msg::Point> AcadosOcpNode::get_all_state_points() {
    std::vector<geometry_msgs::msg::Point> state_points;
    state_points.reserve(BICYCLE_MODEL_N+1);
    for (int i = 0; i <= BICYCLE_MODEL_N; i++) {
        State state = this->get_state(i);
        state_points.push_back(make_point(state.x, state.y));
    }
    return state_points;
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

void AcadosOcpNode::reset_timer() {
    if (this->timer_) {
        this->timer_->cancel();
    }
    auto steady_clock = this->get_clock();
    this->timer_ = rclcpp::create_timer(
        this,
        steady_clock,
        std::chrono::duration<double>(TS),
        std::bind(&AcadosOcpNode::publish_input, this)
    );
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

std::vector<geometry_msgs::msg::Point> AcadosOcpNode::state2point(std::vector<State>& states) {
    std::vector<geometry_msgs::msg::Point> points;
    points.reserve(states.size());
    for (const auto& state : states) {
        points.push_back(make_point(state.x, state.y));
    }
    return points;
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AcadosOcpNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}