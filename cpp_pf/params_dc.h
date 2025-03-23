#ifndef PARAMS_DC_H
#define PARAMS_DC_H

#include <string>
#include <unordered_map>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <complex>

/* PURPOSE: DEFINE DC NETWORK PARAMETERS */

//        nbuses_dc   \
//        nbranches_dc| -- > parameters linked with number
//        nconvs_dc   /
//        ---------------------------------------------------
//        rtf_dc  \
//        xtf_dc  |
//        bf_dc   |
//        rc_dc   |
//        xc_dc   | -- > parameters linked with vsc impedance
//        ztfc_dc |
//        gtfc_dc |
//        btfc_dc /
//        ----------------------------------------------------
//        convState_dc -- > converter state : inv. or rec.
//        ------------------------------------------------
//        aloss_dc \
//        bloss_dc | -- > parameters linked with converter power loss
//        closs_dc /
//        ------------------------------------------------------------
//        Y_dc, y_dc -- > paramters linked with dc admittance matrix
//        ----------------------------------------------------------------

struct DCNetworkParams {
	std::unordered_map<std::string, Eigen::MatrixXd> network_dc;
	double baseMW_dc, pol_dc;
	int nbuses_dc, nbranches_dc, nconvs;
	Eigen::MatrixXd bus_dc, branch_dc, conv;
	Eigen::SparseMatrix<std::complex<double>> Y_dc;
	Eigen::SparseMatrix<double> y_dc;
	Eigen::VectorXcd ztfc;
	Eigen::VectorXd rtf, xtf, bf, rec, xc;
	Eigen::VectorXd gtfc, btfc, aloss, bloss, closs;
	Eigen::VectorXd basekV_dc;
	Eigen::VectorXi convState;
	Eigen::VectorXi fbus_dc, tbus_dc;
};

DCNetworkParams params_dc(const std::string& case_name);

#endif // PARAMS_DC_H

