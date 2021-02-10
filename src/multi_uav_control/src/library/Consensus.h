#include <Eigen/Dense>
#include <Eigen/Sparse>

class Consensus
{
public:
	static Eigen::MatrixXd laplacian(Eigen::MatrixXd &A);
        static Eigen::VectorXd consensus_error(Eigen::MatrixXd &L, Eigen::MatrixXd &qz);
        static Eigen::VectorXd norm_consensus_error(Eigen::VectorXd &e, int n_agents); 
        static Eigen::MatrixXd vector2matrix(Eigen::VectorXd v, int dim = 3);
        static Eigen::VectorXd matrix2vector(Eigen::MatrixXd &M);
    static Eigen::MatrixXd full_adjacency_matrix(Eigen::MatrixXd &A, int dim, int n_agents);
	static Eigen::MatrixXd reference_trajectory(Eigen::MatrixXd &L, Eigen::MatrixXd &x0, double tFin, double T, double limitu, int agent);
};
