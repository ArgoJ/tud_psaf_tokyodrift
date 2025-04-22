#include "mpc.h"


AcadosOcpNode::AcadosOcpNode()
: Node("mpc"), ocp_capsule_(nullptr) {
    this->initialize_ocp_solver();

    // Abonniere die Trajektorie
    trajectory_sub_ = this->create_subscription<utility::msg::Trajectory>(
        "tokyodrift/plan/transformed_lane", 10,
        std::bind(&AcadosOcpNode::trajectory_callback, this, std::placeholders::_1)
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
    std::array<double, NP> ctrl_points = {
        0.0,0.0, 
        0.6,0.0, 
        1.0,0.5, 
        1.8,-0.4
    };

    if (this->ocp_capsule_) {
        this->set_ocp_parameters(ctrl_points);
        this->solve_ocp();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Solver not initialisied");    
        this->initialize_ocp_solver();
    }
}

void AcadosOcpNode::set_ocp_parameters(const std::array<double, NP> & ctrl_points) {
    // Für jeden Zeitschritt k die Parameter updaten
    int status = 0;
    for (int k = 0; k < N; ++k) {
        status = bicycle_model_acados_update_params(
            this->ocp_capsule_, 
            k, 
            const_cast<double*>(ctrl_points.data()), 
            static_cast<int>(ctrl_points.size())
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
    this->print_input(0);
    for (int i = 0; i < N; i++) {
        this->print_state(i);
    }
}

void AcadosOcpNode::print_state(int stage) {
    double values[NX];
    ocp_nlp_out_get(this->ocp_nlp_config_, this->ocp_nlp_dims_, this->ocp_nlp_out_, stage, "x", values);
    std::string str = this->array_stream(values, NX, stage, "x");
    RCLCPP_DEBUG(this->get_logger(), "%s", str.c_str());
}

void AcadosOcpNode::print_input(int stage) {
    double values[NU];
    ocp_nlp_out_get(this->ocp_nlp_config_, this->ocp_nlp_dims_, this->ocp_nlp_out_, stage, "u", values);
    std::string str = this->array_stream(values, NU, stage, "u");
    RCLCPP_DEBUG(this->get_logger(), "%s", str.c_str());
}

std::string AcadosOcpNode::array_stream(const double* values, int size, int stage, const char* field) {
    std::ostringstream oss;
    oss << field << "_" << stage << ": [";
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