#include "MPC.h"
#include <iostream>
#include <cmath>

MPC::MPC(int n_dim, int hp, int hu, int hw, double dt, double c_x, double c_u, double minu, double maxu)
{
	//init
	dim = n_dim;
	Hp = hp;
	Hu = hu;
	T = dt;
	cost_x = c_x;
	cost_u = c_u;
	min_u = minu;
	max_u = maxu;
	settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
	data     = (OSQPData *)c_malloc(sizeof(OSQPData));
	// Define solver settings as default
	if (settings) { osqp_set_default_settings(settings); settings->verbose=0;}
	MPC::init_costs();
	//r.resize(dim*Hp);
	f.resize(dim*Hu);
	MPC::init_Px();
	MPC::init_Pu();
	MPC::init_H();
	r = Eigen::VectorXd::Zero(dim*Hp);
	MPC::init_constraints();
}

void MPC::init_costs()
{
	costs_x.resize(dim*Hp);
	for (int i=0; i < dim*Hp; i++)
		costs_x(i) = cost_x;
	costs_u.resize(dim*Hu);
	for (int i=0; i < dim*Hu; i++)
		costs_u(i) = cost_u;
}

void MPC::init_Px()
{
	Px.resize(dim*Hp, dim);

	for (int i = 0; i < Hp; i++)
		Px.block(i*dim,0, dim, dim) = Eigen::MatrixXd::Identity(dim, dim);
}

void MPC::init_Pu()
{
	Pu = Eigen::MatrixXd::Zero(dim*Hp, dim*Hu);

	for (int i = 0; i < Hp; i++)
		for (int j=0; j <= i; j++)
			if (j < Hu)
				Pu.block(i*dim,j*dim, dim, dim) = T*Eigen::MatrixXd::Identity(dim, dim);
}

void MPC::init_H()
{
	Eigen::MatrixXd Q(dim*Hp, dim*Hp);
	Eigen::MatrixXd C(dim*Hu, dim*Hu);

	Q = costs_x.asDiagonal();
	C = costs_u.asDiagonal();

	//Eigen::MatrixXd aux; std::cout << "Q" << std::endl << Q << std::endl << "Pu" << std::endl << Pu << std::endl;
	//aux = Q*Pu; std::cout << "Q Pu" << std::endl << aux << std::endl << "Pu.T" << std::endl << Pu.transpose() << std::endl;
	//aux = Pu.transpose() * aux; std::cout << "Pu.T Q Pu" << std::endl << aux << std::endl << "C" << std::endl << C << std::endl;
	//aux = aux + C;
	//aux = 2.0*aux; std::cout << "2(Pu.T Q Pu + C)" << std::endl << aux << std::endl;

	H = 2.0*(Pu.transpose() * Q * Pu + C ); //std::cout << "H" << std::endl << H << std::endl;
}

void MPC::init_constraints()
{
	G = Eigen::MatrixXd::Identity(dim*Hu, dim*Hu);
	lVec = Eigen::VectorXd::Constant(dim*Hu, min_u);
	uVec = Eigen::VectorXd::Constant(dim*Hu, max_u);
}

void MPC::set_reference(Eigen::MatrixXd &pos, Eigen::VectorXd &ref, int i)
{
	Eigen::MatrixXd Q(dim*Hp, dim*Hp);

	// Fixed reference
	if (ref.size() == dim)
	{
		for (int i=0; i < Hp; i++)
		{
			r.segment(i*dim, dim) = ref;
		}
	}
	else
		r = ref;

	//Quadratic problem
	Q = costs_x.asDiagonal();
	f = 2.0*Pu.transpose()*Q*(Px*pos.col(i) - r);
}

void MPC::set_reference(Eigen::MatrixXd &pos, Eigen::VectorXd Nk, int &n, int &i)
{
	// Consensus error reference
	Eigen::MatrixXd Q(dim*Hp, dim*Hp);
	Eigen::MatrixXd C(dim*Hu, dim*Hu);
	Eigen::VectorXd ref(dim);
    int n_vec = 0;
    ref = pos.col(i);
    for (int j = 0; j < n; j++)
    {
        if (Nk(j) == 1)
        {
            ref = ref + pos.col(j);
            n_vec = n_vec + 1;
        }
    }
    ref = (1.0/(n_vec+1))*ref;// std::cout << "Goal" << std::endl << ref << std::endl;
	
	for (int k=0; k < Hp; k++)
	{
		r.segment(k*dim, dim) = ref;
	}

	//Quadratic problem
	Q = costs_x.asDiagonal();
	f = 2.0*Pu.transpose()*Q*(Px*pos.col(i) - r);
}

void MPC::set_reference(Eigen::MatrixXd &pos, Eigen::VectorXd Nk, int &n, int &i, Eigen::MatrixXd &u)
{
	// Distributed consensus error
	Eigen::MatrixXd Q(dim*Hp, dim*Hp);
	Eigen::MatrixXd C(dim*Hu, dim*Hu);
	Eigen::MatrixXd Pu_B;
	Eigen::VectorXd u_j(dim*Hu);
	Eigen::VectorXd S = Eigen::VectorXd::Zero(dim*Hp);
	int n_vec = 0;
	for (int j=0; j < n; j++)
	{
		if (Nk(j) == 1)
		{
			for (int k=0; k < Hu; k++)
			{
				u_j.segment(k*dim, dim) = u.col(j);
			}
			S = S + (Px*pos.col(j) + Pu*u_j);
			n_vec = n_vec + 1;
		}
	}
	S = (1.0/(n_vec+1))*S;
	A = S + (1.0/(n_vec+1))*Px*pos.col(i);

	B = (1.0/(n_vec+1))*Pu;

	//Quadratic problem
	Q = costs_x.asDiagonal();
	C = costs_u.asDiagonal();
	Pu_B = Pu - B;
	H = 2.0*(Pu_B.transpose() * Q * Pu_B + C );
	f = 2.0*Pu_B.transpose()*Q*(Px*pos.col(i) - A);
}

void MPC::set_actual_state(Eigen::VectorXd &x_act)
{
	x = x_act;
}

void MPC::update_f(Eigen::VectorXd x_act)
{
	Eigen::MatrixXd Q(dim*Hp, dim*Hp);

	Q = costs_x.asDiagonal();

	f = 2.0*Pu.transpose()*Q*(Px*x_act - r); //std::cout << "R" << std::endl << r << std::endl << "Pu" << std::endl << Pu << std::endl << "Q" << std::endl << Q << std::endl << "Px" << std::endl << Px << std::endl << "x" << std::endl << x_act << std::endl << "f" << std::endl << f << std::endl;
}

void MPC::add_obstacle_constraint(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, double D)
{
	Eigen::VectorXd Y(dim*Hp), diff;
	for (int i=0; i < Hp; i++)
	{
		Y.segment(i*dim, dim) = x_obs;
	}
	diff = Px*x_act - Y;
	G.conservativeResize(G.rows()+1, Eigen::NoChange);
	G.row(G.rows()-1) = (2.0/Hp)*diff.transpose()*Pu;
	lVec.conservativeResize(lVec.size()+1);
	lVec(lVec.size()-1) = D*D - diff.squaredNorm()/Hp;
	uVec.conservativeResize(uVec.size()+1);
	uVec(uVec.size()-1) = OSQP_INFTY;
}

void MPC::add_obstacle_constraint(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, Eigen::VectorXd u, double D)
{
	Eigen::VectorXd u_obs(dim*Hu), Y, diff;
	for (int i=0; i < Hu; i++)
	{
		u_obs.segment(i*dim, dim) = u;
	}
	Y = Px*x_obs + Pu*u_obs;
	diff = Px*x_act - Y;
	G.conservativeResize(G.rows()+1, Eigen::NoChange);
	G.row(G.rows()-1) = (2.0/Hp)*diff.transpose()*Pu;
	lVec.conservativeResize(lVec.size()+1);
	lVec(lVec.size()-1) = D*D - diff.squaredNorm()/Hp;
	uVec.conservativeResize(uVec.size()+1);
	uVec(uVec.size()-1) = OSQP_INFTY;
}

void MPC::add_obstacle_constraint_column(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, double D)
{
	Eigen::VectorXd Y(dim*Hp), diff;
	for (int i=0; i < Hp; i++)
	{
		Y.segment(i*dim, dim) = x_obs;
	}
	diff = Px*x_act - Y;
	for (int i=0; i < Hp; i++)
	{
		diff(i*dim + 2) = 0.0;
	}
	G.conservativeResize(G.rows()+1, Eigen::NoChange);
	G.row(G.rows()-1) = (2.0/Hp)*diff.transpose()*Pu;
	lVec.conservativeResize(lVec.size()+1);
	lVec(lVec.size()-1) = D*D - diff.squaredNorm()/Hp;
	uVec.conservativeResize(uVec.size()+1);
	uVec(uVec.size()-1) = OSQP_INFTY;
}

void MPC::add_obstacle_constraint_column(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, Eigen::VectorXd u, double D)
{
	Eigen::VectorXd u_obs(dim*Hu), Y, diff;
	for (int i=0; i < Hu; i++)
	{
		u_obs.segment(i*dim, dim) = u;
	}
	Y = Px*x_obs + Pu*u_obs;
	diff = Px*x_act - Y;
	for (int i=0; i < Hp; i++)
	{
		diff(i*dim + 2) = 0.0;
	}
	G.conservativeResize(G.rows()+1, Eigen::NoChange);
	G.row(G.rows()-1) = (2.0/Hp)*diff.transpose()*Pu;
	lVec.conservativeResize(lVec.size()+1);
	lVec(lVec.size()-1) = D*D - diff.squaredNorm()/Hp;
	uVec.conservativeResize(uVec.size()+1);
	uVec(uVec.size()-1) = OSQP_INFTY;
}

void MPC::add_obstacle_penalty(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, double D, double k_d)
{
	Eigen::VectorXd Y(dim*Hp), diff(dim*Hp);
	double d_0;
	for (int i=0; i < Hp; i++)
	{
		Y.segment(i*dim, dim) = x_obs;
	}
	diff = Px*x_act - Y;
	d_0 = std::sqrt( diff.squaredNorm() / Hp );
	f = f + ( -2.0*k_d/(Hp*d_0*std::pow(d_0 - D, 3)) )*Pu.transpose()*diff;
}

void MPC::add_obstacle_penalty(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, Eigen::VectorXd u, double D, double k_d)
{
	Eigen::VectorXd u_obs(dim*Hu), Y, diff;
	double d_0;
	for (int i=0; i < Hu; i++)
	{
		u_obs.segment(i*dim, dim) = u;
	}
	Y = Px*x_obs + Pu*u_obs;
	diff = Px*x_act - Y;
	d_0 = std::sqrt( diff.squaredNorm() / Hp );
	f = f + ( -2.0*k_d/(Hp*d_0*std::pow(d_0 - D, 3)) )*Pu.transpose()*diff;
}

void MPC::add_obstacle_penalty_column(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, double D, double k_d)
{
	Eigen::VectorXd Y(dim*Hp), diff;
	double d_0;
	for (int i=0; i < Hp; i++)
	{
		Y.segment(i*dim, dim) = x_obs;
	}
	diff = Px*x_act - Y;
	for (int i=0; i < Hp; i++)
	{
		diff(i*dim + 2) = 0.0;
	}
	d_0 = std::sqrt( diff.squaredNorm() / Hp );
	f = f + ( -2.0*k_d/(Hp*d_0*std::pow(d_0 - D, 3)) )*Pu.transpose()*diff;
}

void MPC::add_obstacle_penalty_column(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, Eigen::VectorXd u, double D, double k_d)
{
	Eigen::VectorXd u_obs(dim*Hu), Y, diff;
	double d_0;
	for (int i=0; i < Hu; i++)
	{
		u_obs.segment(i*dim, dim) = u;
	}
	Y = Px*x_obs + Pu*u_obs;
	diff = Px*x_act - Y;
	for (int i=0; i < Hp; i++)
	{
		diff(i*dim + 2) = 0.0;
	}
	d_0 = std::sqrt( diff.squaredNorm() / Hp );
	f = f + ( -2.0*k_d/(Hp*d_0*std::pow(d_0 - D, 3)) )*Pu.transpose()*diff;
}

void MPC::add_connectivity_constraint(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, double E)
{
	Eigen::VectorXd Y(dim*Hp), diff;
	for (int i=0; i < Hp; i++)
	{
		Y.segment(i*dim, dim) = x_obs;
	}
	diff = Px*x_act - Y;
	G.conservativeResize(G.rows()+1, Eigen::NoChange);
	G.row(G.rows()-1) = (2.0/Hp)*diff.transpose()*Pu;
	lVec.conservativeResize(lVec.size()+1);
	lVec(lVec.size()-1) = -OSQP_INFTY;
	uVec.conservativeResize(uVec.size()+1);
	uVec(uVec.size()-1) = E*E - diff.squaredNorm()/Hp;
}

void MPC::add_connectivity_constraint(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, Eigen::VectorXd u, double E)
{
	Eigen::VectorXd u_obs(dim*Hu), Y, diff;
	for (int i=0; i < Hu; i++)
	{
		u_obs.segment(i*dim, dim) = u;
	}
	Y = Px*x_obs + Pu*u_obs;
	diff = Px*x_act - Y;
	G.conservativeResize(G.rows()+1, Eigen::NoChange);
	G.row(G.rows()-1) = (2.0/Hp)*diff.transpose()*Pu;
	lVec.conservativeResize(lVec.size()+1);
	lVec(lVec.size()-1) = -OSQP_INFTY;
	uVec.conservativeResize(uVec.size()+1);
	uVec(uVec.size()-1) = E*E - diff.squaredNorm()/Hp;
}

void MPC::add_connectivity_penalty(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, double E, double k_e)
{
	Eigen::VectorXd Y(dim*Hp), diff(dim*Hp);
	double d_0;
	for (int i=0; i < Hp; i++)
	{
		Y.segment(i*dim, dim) = x_obs;
	}
	diff = Px*x_act - Y;
	d_0 = std::sqrt( diff.squaredNorm() / Hp );
	f = f + ( -2.0*k_e/(Hp*d_0*std::pow(d_0 - E, 3)) )*Pu.transpose()*diff;
}

void MPC::add_connectivity_penalty(Eigen::VectorXd x_obs, Eigen::VectorXd x_act, Eigen::VectorXd u, double E, double k_e)
{
	Eigen::VectorXd u_obs(dim*Hu), Y, diff;
	double d_0;
	for (int i=0; i < Hu; i++)
	{
		u_obs.segment(i*dim, dim) = u;
	}
	Y = Px*x_obs + Pu*u_obs;
	diff = Px*x_act - Y;
	d_0 = std::sqrt( diff.squaredNorm() / Hp );
	f = f + ( -2.0*k_e/(Hp*d_0*std::pow(d_0 - E, 3)) )*Pu.transpose()*diff;
}

void MPC::set_lower_constraint(Eigen::VectorXd x_act, double minz)
{
	Eigen::VectorXd Pxx;
	Pxx = Px*x_act;
	for (int i=0; i < Hp; i++)
	{
		G.conservativeResize(G.rows()+1, Eigen::NoChange);
		G.row(G.rows()-1) = Pu.row(i*dim + 2);
		lVec.conservativeResize(lVec.size()+1);
		lVec(lVec.size()-1) = minz - Pxx(i*dim + 2);
		uVec.conservativeResize(uVec.size()+1);
		uVec(uVec.size()-1) = OSQP_INFTY;
	}
}

void MPC::set_box_constraints(Eigen::VectorXd x_act, double minx, double maxx, double miny, double maxy, double minz, double maxz)
{
	Eigen::VectorXd Pxx;
	Pxx = Px*x_act;
	for (int i=0; i < Hp; i++)
	{
		// x
		G.conservativeResize(G.rows()+1, Eigen::NoChange);
		G.row(G.rows()-1) = Pu.row(i*dim);
		lVec.conservativeResize(lVec.size()+1);
		lVec(lVec.size()-1) = minx - Pxx(i*dim);
		uVec.conservativeResize(uVec.size()+1);
		uVec(uVec.size()-1) = maxx - Pxx(i*dim);
		// y
		G.conservativeResize(G.rows()+1, Eigen::NoChange);
		G.row(G.rows()-1) = Pu.row(i*dim + 1);
		lVec.conservativeResize(lVec.size()+1);
		lVec(lVec.size()-1) = miny - Pxx(i*dim + 1);
		uVec.conservativeResize(uVec.size()+1);
		uVec(uVec.size()-1) = maxy - Pxx(i*dim + 1);
		// z
		G.conservativeResize(G.rows()+1, Eigen::NoChange);
		G.row(G.rows()-1) = Pu.row(i*dim + 2);
		lVec.conservativeResize(lVec.size()+1);
		lVec(lVec.size()-1) = minz - Pxx(i*dim + 2);
		uVec.conservativeResize(uVec.size()+1);
		uVec(uVec.size()-1) = maxz - Pxx(i*dim + 2);
	}
}

void MPC::set_box_constraints_penalty(Eigen::VectorXd x_act, double minx, double maxx, double miny, double maxy, double D, double k_d)
{
	Eigen::VectorXd Y(dim*Hp), diff, auxm(dim), auxM(dim);
	double d_0;
	//min x
	auxm << minx, miny, 0;
	for (int i=0; i < Hp; i++)
	{
		Y.segment(i*dim, dim) = auxm;
	}
	diff = Px*x_act - Y;
	for (int i=0; i < Hp; i++)
	{
		diff(i*dim + 1) = 0.0;
		diff(i*dim + 2) = 0.0;
	}
	d_0 = std::sqrt( diff.squaredNorm() / Hp );
	f = f + ( -2.0*k_d/(Hp*d_0*std::pow(d_0 - D, 3)) )*Pu.transpose()*diff;
	//min y
	diff = Px*x_act - Y;
	for (int i=0; i < Hp; i++)
	{
		diff(i*dim + 0) = 0.0;
		diff(i*dim + 2) = 0.0;
	}
	d_0 = std::sqrt( diff.squaredNorm() / Hp );
	f = f + ( -2.0*k_d/(Hp*d_0*std::pow(d_0 - D, 3)) )*Pu.transpose()*diff;
	//max x
	auxM << maxx, maxy, 0;
	for (int i=0; i < Hp; i++)
	{
		Y.segment(i*dim, dim) = auxM;
	}
	diff = Px*x_act - Y;
	for (int i=0; i < Hp; i++)
	{
		diff(i*dim + 1) = 0.0;
		diff(i*dim + 2) = 0.0;
	}
	d_0 = std::sqrt( diff.squaredNorm() / Hp );
	f = f + ( -2.0*k_d/(Hp*d_0*std::pow(d_0 - D, 3)) )*Pu.transpose()*diff;
	//max y
	diff = Px*x_act - Y;
	for (int i=0; i < Hp; i++)
	{
		diff(i*dim + 0) = 0.0;
		diff(i*dim + 2) = 0.0;
	}
	d_0 = std::sqrt( diff.squaredNorm() / Hp );
	f = f + ( -2.0*k_d/(Hp*d_0*std::pow(d_0 - D, 3)) )*Pu.transpose()*diff;
}

MPC::~MPC()
{
	//destroy
}

int MPC::compute_control(Eigen::VectorXd &solMPC)
{
	Eigen::MatrixXd P;
	Eigen::VectorXd p_x;
	Eigen::VectorXi p_i, p_p;
	int p_nnz, p_np;
	P = H.triangularView<Eigen::Upper>();
	MPC::dense_to_sparse_csc(P, p_x, p_i, p_p, p_nnz, p_np);
	c_float P_x[p_nnz];
	c_int   P_nnz;
	c_int   P_i[p_nnz];
	c_int   P_p[p_np];// std::cout << "H" << std::endl << H << std::endl << "f" << std::endl << f << std::endl << "G" << std::endl << G << std::endl << "l" << std::endl << lVec << std::endl << "u" << std::endl << uVec << std::endl;

	for (int i=0; i < p_nnz; i++)
	{
		P_x[i] = p_x(i);
		P_i[i] = p_i(i);
	}
	for (int i=0; i < p_np; i++)
	{
		P_p[i] = p_p(i);
	}


	c_float q[dim*Hu];//   = { 1.0, 1.0, };
	for (int i=0; i < dim*Hu; i++)
	{
		q[i] = f(i);
	}

	Eigen::VectorXd a_x;
	Eigen::VectorXi a_i, a_p;
	int a_nnz, a_np;
	MPC::dense_to_sparse_csc(G, a_x, a_i, a_p, a_nnz, a_np);
	c_float A_x[a_nnz];
	c_int   A_nnz;
	c_int   A_i[a_nnz];
	c_int   A_p[a_np];

	for (int i=0; i < a_nnz; i++)
	{
		A_x[i] = a_x(i);
		A_i[i] = a_i(i);
	}
	for (int i=0; i < a_np; i++)
	{
		A_p[i] = a_p(i);
	}

	c_float l[lVec.size()];
	c_float u[uVec.size()];

	for (int i=0; i <lVec.size(); i++)
	{
		l[i] = lVec(i);
		u[i] = uVec(i);
	}

	//c_float A_x[4] = { 1.0, 1.0, 1.0, 1.0, };
	//c_int   A_nnz  = 4;
	//c_int   A_i[4] = { 0, 1, 0, 2, };
	//c_int   A_p[3] = { 0, 2, 4, };
	//c_float l[3]   = { 1.0, 0.0, 0.0, };
	//c_float u[3]   = { 1.0, 0.7, 0.7, };
	c_int n_data = dim*Hu;
	c_int m = lVec.size();

	// Exitflag
	c_int exitflag = 0;

	// Workspace structures
	//OSQPWorkspace *work;
	//OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
	//OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

	// Populate data
	if (data) {
	data->n = n_data;
	data->m = m;
	data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
	data->q = q;
	data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
	data->l = l;
	data->u = u;
	}

	// Setup workspace
	exitflag = osqp_setup(&work, data, settings);

	// Solve Problem
	osqp_solve(work);

	//for (int i = 0; i < n_data; i++)
	//{
	//	sol[i] = &work->solution->x[i];
	//}

	c_float* solution = work->solution->x;
    solMPC = Eigen::Map<Eigen::VectorXd>(solution, work->data->n, 1);
    for (uint i = 0; i < dim*Hu; i++)
    {
    	if (solMPC(i) < min_u)
    		if (solMPC(i) < 2*min_u)
    		{
    			solMPC(i) = -0.001;
    			exitflag = -19;
    		}
    		else
    			solMPC(i) = min_u;
    	if (solMPC(i) > max_u)
    		if (solMPC(i) > 2*max_u)
    		{
    			solMPC(i) = 0.001;
    			exitflag = -19;
    		}
    		else
    			solMPC(i) = max_u;
    }
    //std::cout << solMPC << std::endl;

    //std::cout << "min = " << 0.5*solMPC.transpose()*H*solMPC + f.transpose()*solMPC << std::endl;

	// Clean workspace
	osqp_cleanup(work);
	if (data) {
	if (data->A) c_free(data->A);
	if (data->P) c_free(data->P);
	c_free(data);
	}
	if (settings)  c_free(settings);

	//vel(0) = work->x[0]; vel(1) = work->x[1];
	//OSQPWorkspace returnW = *work;

	return exitflag;
}

void MPC::dense_to_sparse_csc(Eigen::MatrixXd &mat, Eigen::VectorXd &s_x, Eigen::VectorXi &s_i, Eigen::VectorXi &s_p, int &s_nnz, int &s_np)
{
	Eigen::SparseMatrix<double> sMat;
	sMat = mat.sparseView(); //std::cout << sMat << std::endl;
	s_nnz = sMat.nonZeros();
	Eigen::VectorXd v_x(s_nnz);
	Eigen::VectorXi v_i(s_nnz);
	s_np = mat.cols()+1; 
	Eigen::VectorXi v_p(s_np);
	int i = 0;
	int j = 0;
	int ant = -1;
	for (int k=0; k < s_np-1; k++)
	{
		//std::cout << "Outer " << k << std::endl; std::cout << "v r c i" << std::endl;
		for (Eigen::SparseMatrix<double>::InnerIterator it(sMat, k); it; ++it)
		{
			//std::cout << it.value() << " " << it.row() << " " << it.col() << " " << it.index() << std::endl;
			v_x(i) = it.value();
			v_i(i) = it.row();
			if (ant != it.col())
			{
				v_p(j) = i;
				j = j + 1;
			}
			i = i+1;
			ant = it.col();
		}
	}
	v_p(s_np-1) = s_nnz;
	s_x = v_x;
	s_i = v_i;
	s_p = v_p;
}

void MPC::adaptative_Cu_sigmoid(double e, double e0, double min_cu, double max_cu)
{
	//cost_u = (max_cu - min_cu)*e/e0 + min_cu; // linear function of error
	cost_u = 2*(max_cu - min_cu)*(1/(1+exp(-1*e))) + 2*min_cu - max_cu; // sigmoid function of error
	MPC::init_costs();
	MPC::init_H();
}

void MPC::adaptative_Cu_linear(double e, double e0, double min_cu, double max_cu)
{
	cost_u = (max_cu - min_cu)*e/e0 + min_cu; // linear function of error
	//cost_u = 2*(max_cu - min_cu)*(1/(1+exp(-2*e))) + 2*min_cu - max_cu; // sigmoid function of error
	MPC::init_costs();
	MPC::init_H();
}

void MPC::adaptative_Cx_linear(double e, double e0, double min_cx, double max_cx)
{
	double max_e = 3.0;//e0;
	if (e < max_e)
		cost_x = -(max_cx - min_cx)*e/e0 + max_cx;
	else
		cost_x = min_cx;
	/*double m_cx = 4*(max_cx - min_cx)/5 + min_cx;
	double lim = 0.3;
	if (e > lim)
	cost_x = -(m_cx - min_cx)*(e-lim)/(e0-lim) + m_cx; // linear function of error
	else
		cost_x = -(max_cx - m_cx)*e/lim + max_cx;*/ // linear function of error
	//cost_u = 2*(max_cu - min_cu)*(1/(1+exp(-2*e))) + 2*min_cu - max_cu; // sigmoid function of error
	MPC::init_costs();
	MPC::init_H();
}