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
    if (this->ocp_capsule_) {
        bicycle_model_acados_free(ocp_capsule_);
        free(ocp_capsule_);
    }
}

void AcadosOcpNode::trajectory_callback(const utility::msg::Trajectory::SharedPtr msg) {
    // Beispiel-Kontrollpunkte aus Nachricht oder Sensor
    std::array<double,8> ctrl_points = {0.0,0.0,1.0,0.0,2.0,1.0,3.0,1.0};
    this->set_ocp_parameters(ctrl_points);
    this->solve_ocp();
}

void AcadosOcpNode::set_ocp_parameters(const std::array<double, 8> & ctrl_points) {
    // Für jeden Zeitschritt k die Parameter updaten
    for (int k = 0; k < this->NX; ++k) {
        bicycle_model_acados_update_params(
            this->ocp_capsule_, 
            k, 
            const_cast<double*>(ctrl_points.data()), 
            static_cast<int>(ctrl_points.size())
        );
    }
    RCLCPP_DEBUG(this->get_logger(), "OCP-Parameter an Solver übergeben");
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
    int status = bicycle_model_acados_create(ocp_capsule_, "bicycle_model.json");
    if (status) {
        RCLCPP_ERROR(this->get_logger(), "acados_create failed with status %d", status);
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
    int status = bicycle_model_acados_solve(ocp_capsule_);
    if (status != 0) {
        RCLCPP_ERROR(this->get_logger(), "acados_solve failed: %d", status);
        return;
    }
    double u0[2] = {0.0, 0.0}; // Passe die Größe ggf. an NU an
    ocp_nlp_out_get(this->ocp_nlp_config_, this->ocp_nlp_dims_, this->ocp_nlp_out_, 0, "u", u0);
    RCLCPP_INFO(this->get_logger(), "u0[0] = %f, u0[1] = %f", u0[0], u0[1]);
}

// Main-Funktion als Einstiegspunkt
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AcadosOcpNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}