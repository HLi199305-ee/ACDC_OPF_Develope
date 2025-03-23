import numpy as np
from pyomo.environ import *
from pyomo.opt import SolverFactory
from params_acdc import params_ac, params_dc

def solve_opf (acgrid_name: str, dcgrid_name:str):
    # add dc grid 
    result_dc = params_dc(dcgrid_name)
    network_dc = result_dc["network_dc"]
    baseMW_dc = result_dc["baseMW_dc"]
    bus_dc = result_dc["bus_dc"]
    branch_dc = result_dc["branch_dc"]
    conv_dc = result_dc["conv_dc"]
    pol_dc = result_dc["pol_dc"]
    nbuses_dc = result_dc["nbuses_dc"]
    nbranches_dc = result_dc["nbranches_dc"]
    nconvs_dc = result_dc["nconvs_dc"]
    fbus_dc = result_dc["fbus_dc"]
    tbus_dc = result_dc["tbus_dc"]
    ybus_dc = result_dc["ybus_dc"]
    rtf_dc = result_dc["rtf_dc"]
    xtf_dc = result_dc["xtf_dc"]
    bf_dc = result_dc["bf_dc"]
    rec_dc = result_dc["rec_dc"]
    xc_dc = result_dc["xc_dc"]
    ztfc_dc = result_dc["ztfc_dc"]
    gtfc_dc = result_dc["gtfc_dc"]
    btfc_dc = result_dc["btfc_dc"]
    aloss_dc = result_dc["aloss_dc"]
    bloss_dc = result_dc["bloss_dc"]
    closs_dc = result_dc["closs_dc"]
    convState = result_dc["convState"]
    basekV_dc = result_dc["basekV_dc"]

    # add ac grid
    result_ac = params_ac(acgrid_name, dcgrid_name)
    network_ac = result_ac["network_ac"]
    baseMVA_ac = result_ac["baseMVA_ac"]
    bus_entire_ac = result_ac["bus_entire_ac"]
    branch_entire_ac = result_ac["branch_entire_ac"]
    gencost_entire_ac = result_ac["gencost_entire_ac"]
    ngrids = result_ac["ngrids"]
    bus_ac = result_ac["bus_ac"]
    branch_ac = result_ac["branch_ac"]
    generator_ac = result_ac["generator_ac"]
    gencost_ac = result_ac["gencost_ac"]
    recBuses_ac = result_ac["recBuses_ac"]
    recBranches_ac = result_ac["recBranches_ac"]
    recRef_ac = result_ac["recRef_ac"]
    pd_ac = result_ac["pd_ac"]
    qd_ac = result_ac["qd_ac"]
    convids = result_ac["convids"]
    genids = result_ac["genids"]
    nbuses_ac = result_ac["nbuses_ac"]
    nbranches_ac = result_ac["nbranches_ac"]
    ngens_ac = result_ac["ngens_ac"]
    GG_ac = result_ac["GG_ac"]
    BB_ac = result_ac["BB_ac"]
    refbuscount = result_ac["refbuscount"]
    GG_ft_ac = result_ac["GG_ft_ac"]
    BB_ft_ac = result_ac["BB_ft_ac"]
    GG_tf_ac = result_ac["GG_tf_ac"]
    BB_tf_ac = result_ac["BB_tf_ac"]
    fbus_ac = result_ac["fbus_ac"]
    tbus_ac = result_ac["tbus_ac"]

    # init. OPF model
    model = ConcreteModel()

    # define var. in dc grid
    # var_size_dc = 2 * nbuses_dc + 8 * nconvs_dc

    # 1. vn2_dc -the square of dc nodal voltage
    model.vn2_dc = Var(range(nbuses_dc), domain=NonNegativeReals)
    def lb_vn2_dc(model, i):
        return model.vn2_dc[i] >= bus_dc[i, 12] ** 2
    model.lb_vn2_dc = Constraint(range(nbuses_dc), rule = lb_vn2_dc)
    def ub_vn2_dc(model, i):
        return model.vn2_dc[i] <= bus_dc[i, 11] ** 2
    model.ub_vn2_dc = Constraint(range(nbuses_dc), rule = ub_vn2_dc)
    # 2. pn_dc -dc nodal active power injection 
    model.pn_dc = Var(range(nbuses_dc), domain=Reals) 
    # ====================================================================
    #                    ztf = rtf+jxtf        zc = rc+jxc
    #     U_s  ● -------------[###]-------●-------[###]---(vsc)---● U_c
    #                                     |             
    #                                 ──── ────  
    #                                 ──── ────   
    #                                     |    b_f    
    #                    equivalent impdance of VSC ac side
    # ==================================================================== 
    # 3. ps_dc -active power injection at node s of vsc ac side
    model.ps_dc = Var(range(nconvs_dc), domain=Reals)
    # 4. qs_dc -reactive power injection at node s of vsc ac side
    model.qs_dc = Var(range(nconvs_dc), domain=Reals)
    # 5. pc_dc -active power injection at node c of vsc ac side
    model.pc_dc = Var(range(nconvs_dc), domain=Reals)
    # 6. qc_dc -reactive power injection at node c of vsc ac side
    model.qc_dc = Var(range(nconvs_dc), domain=Reals)
    # 7. v2s_dc -the squared nodal voltage amplitude at node s of vsc ac side
    model.v2s_dc = Var(range(nconvs_dc), domain=NonNegativeReals)
    def lb_v2s_dc(model, i):
        return model.v2s_dc[i] >= conv_dc[i, 15] ** 2
    model.lb_v2s_dc = Constraint(range(nbuses_dc), rule = lb_v2s_dc)

    def ub_v2s_dc(model, i):
        return model.v2s_dc[i] <= conv_dc[i, 14] ** 2
    model.ub_v2s_dc = Constraint(range(nbuses_dc), rule = ub_v2s_dc)
    # 8. v2c_dc -the sqared nodal voltage amplitude at node c of vsc ac side
    model.v2c_dc = Var(range(nconvs_dc), domain=NonNegativeReals)
    def lb_v2c_dc(model, i):
        return model.v2c_dc[i] >= conv_dc[i, 15] ** 2
    model.lb_v2c_dc = Constraint(range(nbuses_dc), rule = lb_v2c_dc)

    def ub_v2c_dc(model, i):
        return model.v2c_dc[i] <= conv_dc[i, 14] ** 2
    model.ub_v2c_dc = Constraint(range(nbuses_dc), rule = ub_v2c_dc)
    # 9. Ic_dc -current amplitude of vsc
    model.Ic_dc = Var(range(nconvs_dc),domain=NonNegativeReals)
    def lb_Ic_dc(model, i):
        return model.Ic_dc[i] >= 0.0
    model.lb_Ic_dc = Constraint(range(nconvs_dc), rule = lb_Ic_dc)

    def ub_Ic_dc(model, i):
        return model.Ic_dc[i] <= conv_dc[i, 16]
    model.ub_Ic_dc = Constraint(range(nconvs_dc), rule = ub_Ic_dc)
    # 10. lc_dc -squared current amplitude of vsc
    model.lc_dc = Var(range(nconvs_dc),domain=NonNegativeReals)
    def lb_lc_dc(model, i):
        return model.lc_dc[i] >= 0.0
    model.lb_lc_dc = Constraint(range(nconvs_dc), rule = lb_lc_dc)

    def ub_lc_dc(model, i):
        return model.lc_dc[i] <= conv_dc[i, 16] ** 2
    model.ub_lc_dc = Constraint(range(nconvs_dc), rule = ub_lc_dc)
    # 11. pij_dc - dc branch power flow 
    model.pij_dc = Var(range(nbuses_dc), range(nbuses_dc), domain=Reals)
    
    # 12. lij_dc - the squared dc branch current
    model.lij_dc = Var(range(nbuses_dc), range(nbuses_dc), domain=NonNegativeReals)
    def lb_lij_dc(model, i, j):
        return model.lij_dc[i, j] >= 0.0
    model.lb_lij_dc = Constraint(range(nbuses_dc), range(nbuses_dc), rule = lb_lij_dc)
   
    # 13.  Ctt_dc, Ccc_dc, Ctc_dc, Stc_dc, Sct_dc -related to second-order cone relaxed ac power flow 
    model.Ctt_dc = Var(range(nconvs_dc), domain=NonNegativeReals)
    model.Ccc_dc = Var(range(nconvs_dc), domain=NonNegativeReals)
    model.Ctc_dc = Var(range(nconvs_dc), domain=Reals)
    model.Stc_dc = Var(range(nconvs_dc), domain=Reals)
    model.Cct_dc = Var(range(nconvs_dc), domain=Reals)
    model.Sct_dc = Var(range(nconvs_dc), domain=Reals)
    # # 14. yt_dc, ut_dc -related to big-M relax of the action of on-load tap changer
    # model.kut_dc = Var(range(nconvs_dc), domain=Reals)
    # model.yt_dc = Var(range(nconvs_dc), range(17), domain=Binary)
    # model.ut_dc = Var(range(nconvs_dc), range(17), domain=Reals)

    # 15. convPloss_dc -converter power loss
    model.convPloss_dc = Var(range(nconvs_dc), domain=NonNegativeReals)
    def lb_converter_loss_vsc(model, i):
        return model.convPloss_dc[i] >= 0.0
    model.lb_converter_loss_vsc = Constraint(range(nconvs_dc), rule = lb_converter_loss_vsc)

    def ub_converter_loss_vsc(model, i):
        return model.convPloss_dc[i] <= 1.0
    model.ub_converter_loss_vsc = Constraint(range(nconvs_dc), rule = ub_converter_loss_vsc)

    # -----------------------------------------------
    # Add operational constraints for dcgrid and vsc 
    # ------------------------------------------------
    # 1. constraints for dc power flow -second-order cone relaxation
    zij_dc = 1.0 / np.abs(ybus_dc.toarray())
    np.fill_diagonal(zij_dc, 0)
    zij_dc[np.isinf(zij_dc)] = 1e4

    def nodal_power_flow_dc(model, i):
        return model.pn_dc[i] == pol_dc * sum(model.pij_dc[i, :])
    model.nodal_power_flow_dc = Constraint(range(nbuses_dc), rule = nodal_power_flow_dc)

    def branch_power_flow_dc(model, i, j):
        return model.pij_dc[i, j] + model.pij_dc[j, i] == zij_dc[i, j] * model.lij_dc[i, j]
    model.branch_power_flow_dc = Constraint(range(nbuses_dc), range(nbuses_dc), rule = branch_power_flow_dc)

    def voltage_diff_dc(model, i, j):
        return model.vn2_dc[i] - model.vn2_dc[j] == zij_dc[i, j] * (model.pij_dc[i, j] - model.pij_dc[j, i])
    model.voltage_diff_dc = Constraint(range(nbuses_dc), range(nbuses_dc), rule = voltage_diff_dc)

    def cone_relaxation_dc(model, i, j):
        return model.pij_dc[i, j]**2 <= model.lij_dc[i, j] * model.vn2_dc[i]
    model.cone_relaxation_dc = Constraint(range(nbuses_dc), range(nbuses_dc), rule = cone_relaxation_dc)


    # 2. constraints for vsc ac side power flow -second-order cone relaxation
    def lb_Ccc_dc(model, i):
        return model.Ccc_dc[i] >= 0
    model.lb_Ccc_dc = Constraint(range(nconvs_dc), rule = lb_Ccc_dc)

    def lb_Ctt_dc(model, i):
        return model.Ctt_dc[i] >= 0
    model.lb_Ctt_dc = Constraint(range(nconvs_dc), rule = lb_Ctt_dc)

    def p_balance_s_vsc(model, i):
        return model.ps_dc[i] == model.Ctt_dc[i] * gtfc_dc[i] - model.Ctc_dc[i] * gtfc_dc[i] + model.Stc_dc[i] * btfc_dc[i]
    model.p_balance_s_vsc = Constraint(range(nconvs_dc), rule = p_balance_s_vsc)

    def q_balance_s_vsc(model, i):
        return model.qs_dc[i] == -model.Ctt_dc[i] * btfc_dc[i] + model.Ctc_dc[i] * btfc_dc[i] + model.Stc_dc[i] * gtfc_dc[i]
    model.q_balance_s_vsc = Constraint(range(nconvs_dc), rule = q_balance_s_vsc)

    def p_balance_c_vsc(model, i):
        return model.pc_dc[i] == model.Ccc_dc[i] * gtfc_dc[i] - model.Cct_dc[i] * gtfc_dc[i] + model.Sct_dc[i] * btfc_dc[i]
    model.p_balance_c_vsc = Constraint(range(nconvs_dc), rule = p_balance_c_vsc)

    def q_balance_c_vsc(model, i):
        return model.qc_dc[i] == -model.Ccc_dc[i] * btfc_dc[i] + model.Cct_dc[i] * btfc_dc[i] + model.Sct_dc[i] * gtfc_dc[i]
    model.q_balance_c_vsc = Constraint(range(nconvs_dc), rule = q_balance_c_vsc)

    def symmetry_Ctc_Cct_vsc(model, i):
        return model.Ctc_dc[i] == model.Cct_dc[i]
    model.symmetry_Ctc_Cct_vsc = Constraint(range(nconvs_dc), rule = symmetry_Ctc_Cct_vsc)

    def zeros_Stc_Sct_vsc(model, i):
        return model.Stc_dc[i] + model.Sct_dc[i] == 0
    model.zeros_Stc_Sct_vsc = Constraint(range(nconvs_dc), rule = zeros_Stc_Sct_vsc)

    def cone_relaxation_p1_vsc(model, i):
        return model.Cct_dc[i]**2 + model.Sct_dc[i]**2 <= model.Ccc_dc[i] * model.Ctt_dc[i]
    model.cone_relaxation_p1_vsc = Constraint(range(nconvs_dc), rule = cone_relaxation_p1_vsc)

    def cone_relaxation_p2_vsc(model, i):
        return model.Ctc_dc[i]**2 + model.Stc_dc[i]**2 <= model.Ccc_dc[i] * model.Ctt_dc[i]
    model.cone_relaxation_p2_vsc = Constraint(range(nconvs_dc), rule = cone_relaxation_p2_vsc)

    def voltage_relation_p1_dc_vsc(model, i):
        return model.v2s_dc[i] == model.Ctt_dc[i]
    model.voltage_relation_p1_dc_vsc = Constraint(range(nconvs_dc), rule = voltage_relation_p1_dc_vsc)

    def voltage_relation_p2_dc_vsc(model, i):
        return model.v2c_dc[i] == model.Ccc_dc[i]
    model.voltage_relation_p2_dc_vsc = Constraint(range(nconvs_dc), rule = voltage_relation_p2_dc_vsc)

    # 3. constraints for power loss inside converter 
    def converter_power_balance_vsc(model, i):
        return model.pc_dc[i] + model.pn_dc[i] + model.convPloss_dc[i] == 0
    model.converter_power_balance_vsc = Constraint(range(nconvs_dc), rule = converter_power_balance_vsc)

    def converter_power_loss_function_vsc(model, i):
        return model.convPloss_dc[i] == aloss_dc[i] + bloss_dc[i] * model.Ic_dc[i] + closs_dc[i] * model.lc_dc[i]
    model.converter_power_loss_function_vsc = Constraint(range(nconvs_dc), rule = converter_power_loss_function_vsc)

    def converter_cone_relaxation_p1_vsc(model, i):
        return model.pc_dc[i]**2 + model.qc_dc[i]**2 <= model.lc_dc[i] * model.v2c_dc[i]
    model.converter_cone_relaxation_p1_vsc = Constraint(range(nconvs_dc), rule = converter_cone_relaxation_p1_vsc)

    def converter_cone_relaxation_p2_vsc(model, i):
        return model.Ic_dc[i]**2 <= model.lc_dc[i]
    model.converter_cone_relaxation_p2_vsc = Constraint(range(nconvs_dc), rule = converter_cone_relaxation_p2_vsc)

    # 4. constraints for vsc control model
    def converter_dc_control_vsc(model, i):
        if conv_dc[i, 3] == 1:  # DC P control
            return model.pn_dc[i] == -conv_dc[i, 5] / baseMW_dc
        elif conv_dc[i, 3] == 2:  # DC voltage control
            return model.vn2_dc[i] == conv_dc[i, 7] ** 2
        else:  # Droop control
            return model.pn_dc[i] == (conv_dc[i, 23] - 1 / conv_dc[i, 22] * (0.5 + 0.5 * model.vn2_dc[i] - conv_dc[i, 24])) / baseMW_dc * (-1)
    model.converter_dc_control_vsc = Constraint(range(nconvs_dc), rule = converter_dc_control_vsc)

    def converter_ac_control_vsc(model, i):
        if conv_dc[i, 4] == 1:  # AC Q control
            return model.qs_dc[i] == -conv_dc[i, 6] / baseMW_dc
        else:  # AC voltage control
            return model.v2s_dc[i] == conv_dc[i, 7] ** 2
    model.converter_ac_control_vsc = Constraint(range(nconvs_dc), rule = converter_ac_control_vsc)

    def converter_mode_vsc(model, i):
        if convState[i] == 0:  # Rectifier mode
            return [
            model.ps_dc[i] >= 0,
            model.pn_dc[i] >= 0,
            model.pc_dc[i] <= 0
            ]
        else:  # Inverter mode
            return [
            model.ps_dc[i] <= 0,
            model.pn_dc[i] <= 0,
            model.pc_dc[i] >= 0
            ]
        
    def add_converter_mode_vsc(model, i, idx):
        return converter_mode_vsc(model, i)[idx] 
    model.converter_mode_vsc = Constraint(range(nconvs_dc), range(3), rule = add_converter_mode_vsc)

    # -----------------------------------------------
    #        Define ac grid primal variables 
    # ------------------------------------------------

    # 1. vn2_ac -the squared nodal voltage amplitude 
    model.vn2_ac = Var(
        [(ng, i) for ng in range(ngrids) for i in range(nbuses_ac[ng])],
        domain=NonNegativeReals,
    )
    
    # 2. pn_ac -nodal active power injection
    model.pn_ac = Var(
        [(ng, i) for ng in range(ngrids) for i in range(nbuses_ac[ng])],
        domain=Reals
    )

    # 3. qn_ac -nodal reactive power injection
    model.qn_ac = Var(
        [(ng, i) for ng in range(ngrids) for i in range(nbuses_ac[ng])],
        domain=Reals
    )

    # 4. pgen_ac -generator active power output
    model.pgen_ac = Var(
        [(ng, i) for ng in range(ngrids) for i in range(ngens_ac[ng])],
        domain=Reals
    )

    # 5. qgen_ac -generator reactive power ouput
    model.qgen_ac = Var(
        [(ng, i) for ng in range(ngrids) for i in range(ngens_ac[ng])],
        domain=Reals
    )

     # 6. pij_ac -branch active power flow
    model.pij_ac = Var(
        [(ng, i, j) for ng in range(ngrids) for i in range(nbuses_ac[ng]) for j in range(nbuses_ac[ng])],
        domain=Reals
    )

    # 7. qij_ac -branch reactive power flow
    model.qij_ac = Var(
        [(ng, i, j) for ng in range(ngrids) for i in range(nbuses_ac[ng]) for j in range(nbuses_ac[ng])],
        domain=Reals
    )

    # 8. ss_ac, cc_ac -related to second-order cone relaxed ac power flow
    model.ss_ac = Var(
        [(ng, i, j) for ng in range(ngrids) for i in range(nbuses_ac[ng]) for j in range(nbuses_ac[ng])],
        domain=Reals
    )
    
    model.cc_ac = Var(
        [(ng, i, j) for ng in range(ngrids) for i in range(nbuses_ac[ng]) for j in range(nbuses_ac[ng])],
        domain=NonNegativeReals
    )

    # -----------------------------------------------------------
    #  Add operational constraints for acgrid and ac/dc coupling
    # -----------------------------------------------------------  
    # 1. constraint for ac power flow -second-order cone relaxation
    model.bounds_vn2_ac = ConstraintList()
    for ng in range(ngrids):
        for i in range(nbuses_ac[ng]):
            model.bounds_vn2_ac.add(model.vn2_ac[ng, i] >= bus_ac[ng][i][12] ** 2)
            model.bounds_vn2_ac.add(model.vn2_ac[ng, i] <= bus_ac[ng][i][11] ** 2)

    # model.ref_voltage_ac = ConstraintList()
    # for ng in range(ngrids):
    #     refbus_idx = refbuscount[ng]  
    #     if 0 <= refbus_idx < nbuses_ac[ng]:
    #         ref_voltage_squared = generator_ac[ng][refbus_idx, 5] ** 2
    #         model.ref_voltage_ac.add(model.vn2_ac[ng, refbus_idx] == ref_voltage_squared)

    model.bounds_pgen_ac = ConstraintList()
    for ng in range(ngrids):
        for i in range(ngens_ac[ng]):
            model.bounds_pgen_ac.add(model.pgen_ac[ng, i] >= generator_ac[ng][i, 9] * generator_ac[ng][i, 7] / baseMVA_ac)
            model.bounds_pgen_ac.add(model.pgen_ac[ng, i] <= generator_ac[ng][i, 8] * generator_ac[ng][i, 7] / baseMVA_ac)

    model.bounds_qgen_ac = ConstraintList()
    for ng in range(ngrids):
        for i in range(ngens_ac[ng]):
            model.bounds_qgen_ac.add(model.qgen_ac[ng, i] >= generator_ac[ng][i, 4] * generator_ac[ng][i, 7] / baseMVA_ac)
            model.bounds_qgen_ac.add(model.qgen_ac[ng, i] <= generator_ac[ng][i, 3] * generator_ac[ng][i, 7] / baseMVA_ac)

    model.lb_cc_ac = ConstraintList()
    for ng in range(ngrids):
        for i in range(ngens_ac[ng]):
            model.lb_cc_ac.add(model.cc_ac[ng, i, i] >= 0)

    model.nodal_power_flow_ac = ConstraintList()
    for ng in range(ngrids):
        for i in range(nbuses_ac[ng]):
            pn_constraint_ac = BB_ac[ng][i, i] * model.ss_ac[ng, i, i]
            for j in range(nbuses_ac[ng]):
                pn_constraint_ac += model.cc_ac[ng, i, j] * GG_ac[ng][i, j] - model.ss_ac[ng, i, j] * BB_ac[ng][i, j]
            model.nodal_power_flow_ac.add(model.pn_ac[ng, i] == pn_constraint_ac)

            qn_constraint_ac = GG_ac[ng][i, i] * model.ss_ac[ng, i, i]
            for j in range(nbuses_ac[ng]):
                qn_constraint_ac -= model.cc_ac[ng, i, j] * BB_ac[ng][i, j] + model.ss_ac[ng, i, j] * GG_ac[ng][i, j]
            model.nodal_power_flow_ac.add(model.qn_ac[ng, i] == qn_constraint_ac)

    model.branch_power_flow_ac = ConstraintList()
    for ng in range(ngrids):
        for i in range(nbuses_ac[ng]):
            for j in range(nbuses_ac[ng]):
                pij_constraint_ac = (model.cc_ac[ng, i, i] - model.cc_ac[ng, i, j]) * (-GG_ac[ng][i, j]) \
                                + model.ss_ac[ng, i, j] * (-BB_ac[ng][i, j])
                model.branch_power_flow_ac.add(model.pij_ac[ng, i, j] == pij_constraint_ac)

                qij_constraint_ac = -(model.cc_ac[ng, i, i] - model.cc_ac[ng, i, j]) * (-BB_ac[ng][i, j]) \
                                + model.ss_ac[ng, i, j] * (-GG_ac[ng][i, j])
                model.branch_power_flow_ac.add(model.qij_ac[ng, i, j] == qij_constraint_ac)

    model.symmetry_zero_cc_ss_ac = ConstraintList()
    for ng in range(ngrids):
        for i in range(nbuses_ac[ng]):
            for j in range(nbuses_ac[ng]):
                model.symmetry_zero_cc_ss_ac.add(model.cc_ac[ng, i, j] == model.cc_ac[ng, j, i])
                model.symmetry_zero_cc_ss_ac.add(model.ss_ac[ng, i, j] + model.ss_ac[ng, j, i] == 0)

    model.cone_relaxation_ac = ConstraintList()
    for ng in range(ngrids):
        for i in range(nbuses_ac[ng]):
            for j in range(nbuses_ac[ng]):
                model.cone_relaxation_ac.add(model.cc_ac[ng, i, j] ** 2 + model.ss_ac[ng, i, j] ** 2 <= model.cc_ac[ng, i, i] * model.cc_ac[ng, j, j])

    model.voltage_relation_ac = ConstraintList()
    for ng in range(ngrids):
        for i in range(nbuses_ac[ng]):
            model.voltage_relation_ac.add(model.cc_ac[ng, i, i] == model.vn2_ac[ng, i])

    model.power_coupling_ac_vsc = ConstraintList()
    model.voltage_coupling_ac_vsc = ConstraintList()

    pm_ac = [None] * ngrids
    qm_ac = [None] * ngrids
    for ng in range(ngrids):
        
        pm_ac[ng] = {i: 0.0 for i in range(nbuses_ac[ng])}
        qm_ac[ng] = {i: 0.0 for i in range(nbuses_ac[ng])}

        for i in range(ngens_ac[ng]):
            index = int(generator_ac[ng][i, 0]) - 1
            pm_ac[ng][index] += model.pgen_ac[ng, i]
            qm_ac[ng][index] += model.qgen_ac[ng, i]

        for i in range(nconvs_dc):
            if int(conv_dc[i, 2]) == ng + 1:  
                index = int(conv_dc[i, 1]) - 1
                pm_ac[ng][index] -= model.ps_dc[i]
                qm_ac[ng][index] -= model.qs_dc[i]

        for i in range(nbuses_ac[ng]):
            model.power_coupling_ac_vsc.add(model.pn_ac[ng, i] == pm_ac[ng][i] - pd_ac[ng][i])
            model.power_coupling_ac_vsc.add(model.qn_ac[ng, i] == qm_ac[ng][i] - qd_ac[ng][i])

        for i in range(nconvs_dc):
            j = int(conv_dc[i, 1]) - 1 
            k = int(conv_dc[i, 2]) - 1 
            model.voltage_coupling_ac_vsc.add(model.vn2_ac[k, j] == model.v2s_dc[i])


    model.objective_terms = []

    for ng in range(ngrids):
        actgen_ac = [generator_ac[ng][i, 7] for i in range(ngens_ac[ng])]  
    
        if gencost_ac[ng][0, 3] == 3: 
            obj_gencost = sum(
                actgen_ac[i] * (
                    baseMVA_ac**2 * gencost_ac[ng][i, 4] * model.pgen_ac[ng, i]**2 +
                    baseMVA_ac * gencost_ac[ng][i, 5] * model.pgen_ac[ng, i] +
                    gencost_ac[ng][i, 6]
                ) for i in range(ngens_ac[ng])
            )
    
        elif gencost_ac[ng][0, 3] == 2:  
            obj_gencost = sum(
                actgen_ac[i] * (
                    baseMVA_ac * gencost_ac[ng][i, 4] * model.pgen_ac[ng, i] +
                    gencost_ac[ng][i, 5]
                ) for i in range(ngens_ac[ng])
            )
    
    model.objective_terms.append(obj_gencost)

    model.objective = Objective(expr=sum(model.objective_terms), sense=minimize)
 

    solver = SolverFactory("gurobi")
    solver.options["MIPGap"] = 1e-4        
    solver.options["TimeLimit"] = 360  
    solver.options["Threads"] = 8          
    solver.options["Presolve"] = 2         
    solver.options["OutputFlag"] = 1  
    solver.options["BarConvTol"] = 1e-9
    solver.options["FeasibilityTol"] = 1e-9
    solver.options["OptimalityTol"] = 1e-9
    solver.options["NumericFocus"] = 3
  
    solver.solve(model, tee=True)


    print("================================================================================")
    print("|   AC  Bus Data                                                               |")
    print("================================================================================")
    print("Area     Bus       Voltage             Generation               Load        ")
    print("#        #     Mag [pu] Ang [deg]   Pg [MW]  Qg [MVAr]    P [MW]  Q [MVAr]")
    print("-----   -----  --------  --------  --------  ---------   ------  --------")
    formatted_vm_ac = {ng: [np.sqrt(value(model.vn2_ac[ng, i])) for i in range(nbuses_ac[ng])] for ng in range(ngrids)}
    formatted_pgen_ac = {ng: [value(model.pgen_ac[ng, j]) * baseMVA_ac for j in range(ngens_ac[ng])] for ng in range(ngrids)}
    formatted_qgen_ac = {ng: [value(model.qgen_ac[ng, j]) * baseMVA_ac for j in range(ngens_ac[ng])] for ng in range(ngrids)}
    formatted_pd_ac = {ng: [pd_ac[ng][i] * baseMVA_ac for i in range(nbuses_ac[ng])] for ng in range(ngrids)}
    formatted_qd_ac = {ng: [qd_ac[ng][i] * baseMVA_ac for i in range(nbuses_ac[ng])] for ng in range(ngrids)}
    for ng in range(ngrids):
        genidx = [int(generator_ac[ng][j, 0]) for j in range(ngens_ac[ng])]  
        for i in range(nbuses_ac[ng]):
            printed_vm_ac = formatted_vm_ac[ng][i]
            if i == 0 and ng == 0:
                print(f"{ng+1:3}{i+1:9}{printed_vm_ac:10.3f}{0.0:10.1f}", end="")
            else:
                print(f"\n{ng+1:3}{i+1:9}{printed_vm_ac:10.3f}{0.0:10.1f}", end="")

            if i in recRef_ac[ng]:
                print("*", end="")

            if i + 1 in genidx:
                gen_idx = genidx.index(i + 1)
                printed_pgen = formatted_pgen_ac[ng][gen_idx]
                printed_qgen = formatted_qgen_ac[ng][gen_idx]
                
                if i in recRef_ac[ng]:
                    print(f"{printed_pgen:10.3f}{printed_qgen:9.3f}", end="")
                else:
                    print(f"{printed_pgen:11.3f}{printed_qgen:9.3f}", end="")

                printed_pd = formatted_pd_ac[ng][i]
                printed_qd = formatted_qd_ac[ng][i]
                print(f"{printed_pd:11.3f}{printed_qd:9.3f}", end="")

            else:
                print("          -       -", end="")
                printed_pd = formatted_pd_ac[ng][i]
                printed_qd = formatted_qd_ac[ng][i]
                print(f"{printed_pd:12.3f}{printed_qd:9.3f}", end="")
        
    print("\n-----   -----  --------  --------  --------  ---------   ------  --------")
    
    GenCost = value(model.objective)
    print(f"The total generation costs is ＄{GenCost:.2f}/MWh (€{GenCost / 1.08:.2f}/MWh)")
    print("\n")


    print("===========================================================================================")
    print("|     AC Grids Branch Data                                                                |")
    print("===========================================================================================")
    print("Area   Branch  From   To        From Branch Flow         To Branch Flow        Branch Loss")
    print("#      #       Bus#   Bus#    Pij [MW]   Qij [MVAr]    Pij [MW]   Qij [MVAr]  Pij_loss [MW]")
    print("----   ------  -----  -----  ---------  ----------   ----------  ----------  -------------")
    
    formatted_pij_ac = {
        ng: [[value(model.pij_ac[ng, i, j]) * baseMVA_ac for j in range(nbuses_ac[ng])] 
         for i in range(nbuses_ac[ng])]
    for ng in range(ngrids)
    }
    
    formatted_qij_ac = {
        ng: [[value(model.qij_ac[ng, i, j]) * baseMVA_ac for j in range(nbuses_ac[ng])] 
         for i in range(nbuses_ac[ng])]
    for ng in range(ngrids)
    }

    NetPloss_ac = 0.0
    for ng in range(ngrids):
        for i in range(nbranches_ac[ng]):
            printed_fbus_ac = fbus_ac[ng][i] 
            printed_tbus_ac = tbus_ac[ng][i] 
            
            printed_pij_from_to = formatted_pij_ac[ng][printed_fbus_ac-1][printed_tbus_ac-1]
            printed_qij_from_to = formatted_qij_ac[ng][printed_fbus_ac-1][printed_tbus_ac-1]
            printed_pij_to_from = formatted_pij_ac[ng][printed_tbus_ac-1][printed_fbus_ac-1]
            printed_qij_to_from = formatted_qij_ac[ng][printed_tbus_ac-1][printed_fbus_ac-1]
           
            
            printed_pij_loss_ac = abs(printed_pij_from_to + printed_pij_to_from)
            NetPloss_ac += printed_pij_loss_ac
            
            print(f"{ng+1:2} {i+1:6} {printed_fbus_ac:7} {printed_tbus_ac:6}"
                  f" {printed_pij_from_to:12.3f} {printed_qij_from_to:11.3f}"
                  f" {printed_pij_to_from:12.3f} {printed_qij_to_from:11.3f} {printed_pij_loss_ac:11.3f}")
    print("----   ------  -----  -----  ---------  -----------    --------  ----------  -------------")
    
    print(f"The total AC network losses is {NetPloss_ac:.3f} MW\n")
    print("\n")

    print("================================================================================")
    print("|   MTDC Bus Data                                                              |")
    print("================================================================================")
    print(" Bus   Bus    AC   DC Voltage   DC Power   PCC Bus Injection   Converter loss")
    print(" DC #  AC #  Area   Vdc [pu]    Pdc [MW]   Ps [MW]  Qs [MVAr]  Conv_Ploss [MW]")
    print("-----  ----  ----  ---------    --------   -------  --------    --------")
    formatted_vm_dc = np.sqrt([value(model.vn2_dc[i]) for i in range(nbuses_dc)])
    formatted_pn_dc = [value(model.pn_dc[i]) * baseMW_dc for i in range(nbuses_dc)]
    formatted_ps_dc = [value(model.ps_dc[i]) * baseMW_dc for i in range(nconvs_dc)]
    formatted_qs_dc = [value(model.qs_dc[i]) * baseMW_dc for i in range(nconvs_dc)]
    formatted_convPloss_dc = [value(model.convPloss_dc[i]) * baseMW_dc for i in range(nbuses_dc)]
    TotalConvPloss = 0.0
    for i in range(nbuses_dc):
        printed_ac_bus = int(conv_dc[i, 1])
        printed_ac_area = int(conv_dc[i, 2])
        printed_vm_dc = formatted_vm_dc[i]
        printed_pn_dc = formatted_pn_dc[i]
        printed_ps_dc = formatted_ps_dc[i]
        printed_qs_dc = formatted_qs_dc[i]
        printed_convPloss_dc =  formatted_convPloss_dc[i]
        TotalConvPloss += printed_convPloss_dc
        print(f"{i+1:4}{printed_ac_bus:6}{printed_ac_area:6}{printed_vm_dc:10.3f}{printed_pn_dc:14.3f}"
              f"{printed_ps_dc:10.3f}{printed_qs_dc:9.3f}{printed_convPloss_dc:11.3f}")
    print("-----  ----  ----  ---------    --------   -------  --------    --------")

    print(f"The total converter losses is {TotalConvPloss:.3f} MW\n")
    print("\n")

    print("===================================================================")
    print(" |     MTDC Branch Data                                            |")
    print(" ===================================================================")
    print(" Branch  From   To     From Branch    To Branch      Branch Loss")
    print(" #       Bus#   Bus#   Flow Pij [MW]  Flow Pij [MW]  Pij_loss [MW]")
    print(" ------  -----  -----   ---------      ---------      ---------")
    formatted_pij_dc = [[value(model.pij_dc[i, j]) for j in range(nbuses_dc)] for i in range(nbuses_dc)] * baseMW_dc * pol_dc
    
    NetPloss_dc = 0.0
    for i in range(nbranches_dc):
        printed_from_bus_dc = fbus_dc[i] 
        printed_to_bus_dc = tbus_dc[i] 

        printed_pij_from_to_dc = formatted_pij_dc[printed_from_bus_dc-1][printed_to_bus_dc-1] 
        printed_pij_to_from_dc = formatted_pij_dc[printed_to_bus_dc-1][printed_from_bus_dc-1]
        printed_pij_loss_dc = abs(printed_pij_from_to_dc + printed_pij_to_from_dc)


        print(f"{i+1:5} {printed_from_bus_dc:6} {printed_to_bus_dc:6}"
              f" {printed_pij_from_to_dc:11.3f} {printed_pij_to_from_dc:14.3f} {printed_pij_loss_dc:13.3f}")
        
        NetPloss_dc += printed_pij_loss_dc
    
    print(" ------  -----  -----   ---------      ---------      ---------")
    print(f" The total DC network losses is {NetPloss_dc:.3f} MW.\n")



if __name__ == "__main__":
    result_opf = solve_opf("ac14ac57", "mtdc3slack_a")