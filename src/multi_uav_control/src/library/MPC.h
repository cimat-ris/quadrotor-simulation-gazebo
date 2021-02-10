#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "osqp.h"

class MPC
{
public:
	int dim;
	int Hp;
	int Hu;
	double T, cost_x, cost_u, min_u, max_u;
	Eigen::MatrixXd H;
	Eigen::VectorXd f;
	Eigen::VectorXd r;
	Eigen::MatrixXd G;
	Eigen::VectorXd costs_x, costs_u;
	Eigen::VectorXd x;
	Eigen::MatrixXd Pu, Px;
	Eigen::VectorXd lVec, uVec;
	//Distributed control
	Eigen::MatrixXd B;
	Eigen::VectorXd A;
	OSQPWorkspace *work;
	OSQPSettings  *settings;
	OSQPData      *data;
	MPC(int dim, int Hp, int Hu, int Hw, double T, double c_x, double c_u, double minu, double maxu);
	~MPC();
	void adaptative_Cu_linear(double e, double e0, double min_cu, double max_cu);
	void adaptative_Cu_sigmoid(double e, double e0, double min_cu, double max_cu);
	void adaptative_Cx_linear(double e, double e0, double min_cx, double max_cx);
	int compute_control(Eigen::VectorXd &solMPC);
	void set_reference(Eigen::MatrixXd &pos, Eigen::VectorXd &ref, int i);
	void set_reference(Eigen::MatrixXd &pos, Eigen::VectorXd Nk, int &n, int &i);
	void set_reference(Eigen::MatrixXd &pos, Eigen::VectorXd Nk, int &n, int &i, Eigen::MatrixXd &u);
	void set_actual_state(Eigen::VectorXd &x_act);
	void update_f(Eigen::VectorXd x_act);
	void add_obstacle_constraint(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, double D);
	void add_obstacle_constraint(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, Eigen::VectorXd u, double D);
	void add_obstacle_constraint_column(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, double D);
	void add_obstacle_constraint_column(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, Eigen::VectorXd u, double D);
	void add_obstacle_penalty(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, double D, double k_d);
	void add_obstacle_penalty(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, Eigen::VectorXd u, double D, double k_d);
	void add_obstacle_penalty_column(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, double D, double k_d);
	void add_obstacle_penalty_column(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, Eigen::VectorXd u, double D, double k_d);
	void add_connectivity_constraint(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, double D);
	void add_connectivity_constraint(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, Eigen::VectorXd u, double D);
	void add_connectivity_penalty(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, double D, double k_d);
	void add_connectivity_penalty(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, Eigen::VectorXd u, double D, double k_d);
	void set_lower_constraint(Eigen::VectorXd x_act, double minz);
	void set_box_constraints(Eigen::VectorXd x_act, double minx, double maxx, double miny, double maxy, double minz, double maxz);
	void set_box_constraints_penalty(Eigen::VectorXd x_act, double minx, double maxx, double miny, double maxy, double D, double k_d);
private:
	void dense_to_sparse_csc(Eigen::MatrixXd& mat, Eigen::VectorXd& s_x, Eigen::VectorXi& s_i, Eigen::VectorXi& s_p, int& s_nnz, int& s_np);
	void init_costs(int dim, int Hp, int Hu);
	void init_costs();
	void init_H();
	void init_Px();
	void init_Pu();
	void init_constraints();
};
