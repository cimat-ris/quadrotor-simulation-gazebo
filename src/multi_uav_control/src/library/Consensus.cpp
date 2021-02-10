#include "Consensus.h"
#include <iostream>

Eigen::MatrixXd Consensus::laplacian(Eigen::MatrixXd &A)
{
    int rows = A.rows();
    Eigen::VectorXd Ones = Eigen::VectorXd::Ones(rows);
    Eigen::MatrixXd D = (A * Ones).asDiagonal();
    Eigen::MatrixXd L = D - A;

    return L;
}

Eigen::VectorXd Consensus::consensus_error(Eigen::MatrixXd &L, Eigen::MatrixXd &qz)
{
    int rows, dim;
    rows = L.rows(); // number of agents
    dim = qz.rows(); // dimension of the system
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim,dim);
    Eigen::VectorXd qz_vector = Consensus::matrix2vector(qz);
    Eigen::MatrixXd ext_L = Eigen::MatrixXd::Identity(dim*rows, dim*rows);
    for (int i=0; i < dim; i++)
        ext_L.block(i*rows, i*rows, rows, rows) = L;
    Eigen::VectorXd ez = -ext_L*qz_vector;

    return ez;
}

Eigen::VectorXd Consensus::norm_consensus_error(Eigen::VectorXd &e, int n_agents)
{
    int dim = e.size()/n_agents;
    Eigen::VectorXd e_L2(dim);
    for(int i=0; i<dim;i++){
        e_L2(i) = e.segment(i*n_agents, n_agents).norm();
    }

    return e_L2;
}

Eigen::VectorXd Consensus::matrix2vector(Eigen::MatrixXd &M)
{
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> M2(M);
    Eigen::Map<Eigen::VectorXd> v(M2.data(), M2.size());

    return v;
}

Eigen::MatrixXd Consensus::vector2matrix(Eigen::VectorXd v, int dim)
{
    int n_robots = v.size()/dim;
    Eigen::Map<Eigen::MatrixXd> MT(v.data(),n_robots, dim);
    Eigen::MatrixXd M =  MT.transpose().eval();

    return M;
}

Eigen::MatrixXd Consensus::full_adjacency_matrix(Eigen::MatrixXd &A, int dim, int n_agents)
{
    Eigen::MatrixXd A_full = Eigen::MatrixXd::Zero(dim*n_agents, dim*n_agents);

    for (int i=0; i < n_agents; i++)
        for (int j=0; j < n_agents; j++)
            if (A(i,j) == 1)
                A_full.block(i*dim, j*dim, dim, dim) = Eigen::MatrixXd::Identity(dim, dim);

    return A_full;
}

Eigen::MatrixXd Consensus::reference_trajectory(Eigen::MatrixXd &L, Eigen::MatrixXd &x0, double tFin, double T, double limitu, int agent)
{
    int steps = int(tFin/T);
    int dim, n_agents;
    dim = x0.rows(); n_agents = x0.cols();
    Eigen::MatrixXd ref(dim,steps+1);
    Eigen::VectorXd u(dim*n_agents), u_test(dim*n_agents), maxabs;
    Eigen::VectorXd x(dim*n_agents);
    for (int i=0; i < n_agents; i++)
        x.segment(i*dim, dim) = x0.col(i);
    double t = 0, k = 1.0;

    u_test = -k * L * x;
    maxabs = u_test.cwiseAbs();
    k = 10.0 * limitu / maxabs.maxCoeff();
    ref.col(0) = x.segment(agent*dim, dim);
    for (int i=1; i <= steps; i++)
    {
        t = t + T;
        u = -k * L * x;
        for (int j=0; j < dim*n_agents; j++)
        {
            if (u(j) > limitu)
                u(j) = limitu;
            if (u(j) < -limitu)
                u(j) = -limitu;
        }
        x = x + T*u;
        ref.col(i) = x.segment(agent*dim, dim);
    }
    return ref;
}
