#ifndef VIZ_OPF_H
#define VIZ_OPF_H

#include <Eigen/Dense>
#include <vector>
#include <array>

struct OPFVisualData {
   
    Eigen::MatrixXd bus_entire_ac;
    Eigen::MatrixXd branch_entire_ac;
    Eigen::MatrixXd gen_entire_ac;
    Eigen::MatrixXd bus_dc;
    Eigen::MatrixXd branch_dc;
    Eigen::MatrixXd conv_dc;

    Eigen::VectorXd vn2_dc_k;
    Eigen::VectorXd ps_dc_k;
    Eigen::VectorXd qs_dc_k;
    Eigen::MatrixXd pij_dc_k;
                     
    std::vector<int> nbuses_ac;                     
    std::vector<int> ngens_ac;        
  
    std::vector<Eigen::VectorXd> vn2_ac_k;   
    std::vector<Eigen::VectorXd> pgen_ac_k;  
    std::vector<Eigen::VectorXd> qgen_ac_k; 
   
    std::vector<Eigen::MatrixXd> pij_ac_k;   
    std::vector<Eigen::MatrixXd> qij_ac_k;  

    int nconvs_dc;
    int nbuses_dc;
    int ngrids;

    double baseMVA_ac;
    double baseMW_dc;
    double pol_dc;

};

void viz_opf(const OPFVisualData& data);

#endif
