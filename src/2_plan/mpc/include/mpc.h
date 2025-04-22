#ifndef ACADOS_OCP_NODE_HPP
#define ACADOS_OCP_NODE_HPP


#include <memory>
#include <array>
#include <rclcpp/rclcpp.hpp>
#include "utility/msg/trajectory.hpp"

#define NX     7
#define NZ     0
#define NU     2
#define NP     8
#define NP_GLOBAL     0
#define NBX    7
#define NBX0   7
#define NBU    2
#define NSBX   0
#define NSBU   0
#define NSH    0
#define NSH0   0
#define NSG    0
#define NSPHI  0
#define NSHN   0
#define NSGN   0
#define NSPHIN 0
#define NSPHI0 0
#define NSBXN  0
#define NS     0
#define NS0    0
#define NSN    0
#define NG     0
#define NBXN   7
#define NGN    0
#define NY0    7
#define NY     7
#define NYN    5
#define N      30
#define NH     1
#define NHN    0
#define NH0    0
#define NPHI0  0
#define NPHI   0
#define NPHIN  0
#define NR     0

extern "C" {
    typedef struct ocp_nlp_config ocp_nlp_config;
    typedef struct ocp_nlp_dims ocp_nlp_dims;
    typedef struct ocp_nlp_out ocp_nlp_out;
    typedef struct ocp_nlp_in ocp_nlp_in;
    typedef struct ocp_nlp_solver ocp_nlp_solver;
    typedef struct ocp_nlp_opts ocp_nlp_opts;
    typedef struct bicycle_model_solver_capsule bicycle_model_solver_capsule;

    // Funktionsprototyp für ocp_nlp_out_get
    int ocp_nlp_out_get(
        ocp_nlp_config* config,
        ocp_nlp_dims* dims,
        ocp_nlp_out* out,
        int stage,
        const char* field,
        void* value
    );
    int ocp_nlp_out_set(
        ocp_nlp_config* config,
        ocp_nlp_dims* dims,
        ocp_nlp_out* out,
        int stage,
        const char* field,
        void* value
    );
    int ocp_nlp_constraints_model_set(
        ocp_nlp_config* config,
        ocp_nlp_dims* dims,
        ocp_nlp_in* in,
        int stage,
        const char* field,
        void* value
    );
    int ocp_nlp_get(
        ocp_nlp_solver* solver,
        const char* field,
        void* value
    );

    ocp_nlp_out*    bicycle_model_acados_get_nlp_out(bicycle_model_solver_capsule* capsule);
    ocp_nlp_in*     bicycle_model_acados_get_nlp_in(bicycle_model_solver_capsule* capsule);
    ocp_nlp_dims*   bicycle_model_acados_get_nlp_dims(bicycle_model_solver_capsule* capsule);
    ocp_nlp_config* bicycle_model_acados_get_nlp_config(bicycle_model_solver_capsule* capsule);
    ocp_nlp_solver* bicycle_model_acados_get_nlp_solver(bicycle_model_solver_capsule* capsule);
    bicycle_model_solver_capsule*   bicycle_model_acados_create_capsule();
    int     bicycle_model_acados_create(bicycle_model_solver_capsule* capsule, const char* json_file);
    int     bicycle_model_acados_solve(bicycle_model_solver_capsule* capsule);
    int     bicycle_model_acados_update_params(bicycle_model_solver_capsule* capsule, int stage, double* p, int np);
    int     bicycle_model_acados_free(bicycle_model_solver_capsule* capsule);
    int     bicycle_model_acados_free_capsule(bicycle_model_solver_capsule* capsule);
}

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

    // Prints the OCP solution
    void print_state(int stage);
    void print_input(int stage);

    std::string array_stream(const double* values, int size, int stage, const char * field);
};

#endif  // ACADOS_OCP_NODE_HPP