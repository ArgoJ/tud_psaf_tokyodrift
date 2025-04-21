#ifndef ACADOS_OCP_NODE_HPP
#define ACADOS_OCP_NODE_HPP

#include <memory>
#include <array>
#include <rclcpp/rclcpp.hpp>
#include "utility/msg/trajectory.hpp" // Annahme: Diese Message ist im Package "utility"

extern "C" {
    typedef struct ocp_nlp_config ocp_nlp_config;
    typedef struct ocp_nlp_dims ocp_nlp_dims;
    typedef struct ocp_nlp_out ocp_nlp_out;
    typedef struct ocp_nlp_in ocp_nlp_in;

    // Funktionsprototyp für ocp_nlp_out_get
    int ocp_nlp_out_get(
        ocp_nlp_config* config,
        ocp_nlp_dims* dims,
        ocp_nlp_out* out,
        int stage,
        const char* field,
        void* value
    );

    void*   bicycle_model_acados_create_capsule();
    int     bicycle_model_acados_create(void* capsule, const char* json_file);
    ocp_nlp_out*    bicycle_model_acados_get_nlp_out(void* capsule);
    ocp_nlp_in*     bicycle_model_acados_get_nlp_in(void* capsule);
    ocp_nlp_dims*   bicycle_model_acados_get_nlp_dims(void* capsule);
    ocp_nlp_config* bicycle_model_acados_get_nlp_config(void* capsule);
    int     bicycle_model_acados_solve(void* capsule);
    void*   bicycle_model_acados_get_nlp_solver(void* capsule);
    int     bicycle_model_acados_update_params(void* capsule, int stage, double* p, int np);
    int     bicycle_model_acados_free(void* capsule);
}

class AcadosOcpNode : public rclcpp::Node {
private:
    // Membervariable, um den acados OCP Solver-Handle zu speichern.
    // Der konkrete Typ hängt von deiner acados-Integration ab; hier als void* exemplarisch.
    void * ocp_capsule_, * ocp_nlp_solver_;
    ocp_nlp_out* ocp_nlp_out_;
    ocp_nlp_in* ocp_nlp_in_; 
    ocp_nlp_dims* ocp_nlp_dims_;
    ocp_nlp_config* ocp_nlp_config_;
    static const int NX = 7, NU = 2, NP = 8, NH = 1, N = 30;

    // Sensorwerte für v und delta, z. B. aus Parametern oder einem anderen Topic
    double sensor_v_;
    double sensor_delta_;

    // Subscriber für die Trajektorie
    rclcpp::Subscription<utility::msg::Trajectory>::SharedPtr trajectory_sub_;

public:
    AcadosOcpNode();
    ~AcadosOcpNode();
  
private:
    // Callback für Trajektorien-Updates
    void trajectory_callback(const utility::msg::Trajectory::SharedPtr msg);

    // Hilfsfunktion: Setzt die Kontrollpunkte als Parameter im acados OCP Solver für alle Shooting-Nodes
    void set_ocp_parameters(const std::array<double, 8> & ctrl_points);

    // Initialisiert den OCP-Solver (z.B. durch einen Aufruf einer externen API)
    void initialize_ocp_solver();

    // Löst das OCP und gibt ggf. den neuen Zustandsvektor zurück
    void solve_ocp();
};

#endif  // ACADOS_OCP_NODE_HPP