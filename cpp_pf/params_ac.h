#ifndef PARAMS_AC_H
#define PARAMS_AC_H

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unordered_map>
#include <vector>
#include <string>

/* PURPOSE: DEFINE AC NETWORK PARAMETERS */

//        nbuses_ac    \
//        nbranches_ac | --> parameters linked with number
//        ngens_ac /
// --------------------------------------------------------
//        recbuscount_ac \
//        genids_ac      | --> parameters linked with bus ID
//        convids_ac     /
// ----------------------------------------------------------
//        fbus_ac(from) \
//        tbus_ac(to)   |
//        YY_ac         |
//        GG_ac         |
//        BB_ac         | --> parameters linked with ac admittance matrix
//        GGft_ac       |
//        BBft_ac       |
//        GGtf_ac       |
//        BBtf_ac       /
// --------------------------------------------------------------------------
//        pd_ac        \
//        qd_ac        / --> parameters linked with ac load
// ---------------------------------------------------------
//        bus_ac       \
//        branch_ac    | --> reshape of bus, branch, generator, ...
//        generator_ac |
//        gencost_ac   /
// -----------------------------------------------------------------
//        recBuses_ac    \
//        recBranches_ac | --> Buses, Branches, Ref. for the different grids
//        recRef_ac      /
// ---------------------------------------------------------------------------

struct ACNetworkParams {
	std::unordered_map<std::string, Eigen::MatrixXd> network_ac;
	double baseMVA_ac;
	Eigen::MatrixXd bus_ac_entire, branch_ac_entire, generator_ac_entire, gencost_ac_entire;
	int ngrids;
	std::vector<Eigen::MatrixXd> bus_ac, branch_ac, generator_ac, gencost_ac;
	std::vector<std::vector<int>> recRef;
	std::vector<Eigen::VectorXd> pd_ac, qd_ac;
	std::vector<std::vector<int>> genids;
	std::vector<std::vector<std::pair<int, int>>> convids;
	std::vector<int> nbuses_ac, nbranches_ac, ngens_ac;
	std::vector<Eigen::SparseMatrix<double>> GG_ac, BB_ac, GG_ft_ac, BB_ft_ac, GG_tf_ac, BB_tf_ac;
	std::vector<Eigen::VectorXi> fbus_ac, tbus_ac;
	std::vector<Eigen::MatrixXd> anglelim_rad;
	std::vector<Eigen::VectorXi> IDtoCountmap;
	std::vector<int> refbuscount_ac;
};
ACNetworkParams params_ac(const std::string& acgrid_name, const std::string& dcgrid_name);

#endif // PARAMS_AC_H

