#include "viz_opf.h"
#include<matplot/matplot.h>
#include <cmath>
#include <utility>
#include <thread>
#include<unordered_set>
#include <algorithm>
#include <iomanip>

using namespace matplot;

namespace {
    // custom 
    static const std::vector<float> oranges_stops = {
        0.0f,   0.125f, 0.25f,  0.375f, 0.5f,   0.625f, 0.75f,  0.875f, 1.0f
    };
    static const std::vector<std::array<float, 3>> oranges_colors = {
        {1.00f, 0.96f, 0.92f},  
        {1.00f, 0.90f, 0.81f},  
        {0.99f, 0.82f, 0.64f},  
        {0.99f, 0.68f, 0.42f},  
        {0.99f, 0.55f, 0.23f}, 
        {0.95f, 0.41f, 0.07f},  
        {0.85f, 0.28f, 0.00f},  
        {0.65f, 0.21f, 0.01f},  
        {0.50f, 0.15f, 0.02f}   
    };


    static std::array<float, 3> oranges_colormap(float t) {
        const float t_min = 0.05f;  
        t = t_min + t * (1.0f - t_min);

        if (t <= oranges_stops.front()) return oranges_colors.front();
        if (t >= oranges_stops.back())  return oranges_colors.back();

        auto it = std::upper_bound(oranges_stops.begin(), oranges_stops.end(), t);
        int idx = static_cast<int>(std::distance(oranges_stops.begin(), it));

        float t0 = oranges_stops[idx - 1];
        float t1 = oranges_stops[idx];
        const auto& c0 = oranges_colors[idx - 1];
        const auto& c1 = oranges_colors[idx];

        float alpha = (t - t0) / (t1 - t0);  

        return {
            c0[0] + alpha * (c1[0] - c0[0]),
            c0[1] + alpha * (c1[1] - c0[1]),
            c0[2] + alpha * (c1[2] - c0[2])
        };
    }

} // namespace

void viz_opf(const OPFVisualData& d) {
        
      /**************************************************
      * REORDER DATA MATRIXES
      **************************************************/
    
        // 1. reorder ac data
        int numBuses_ac = d.bus_entire_ac.rows();
        int numColsBuses_ac = d.bus_entire_ac.cols();
        int numColsBranches_ac = d.branch_entire_ac.cols();

        Eigen::VectorXi oldBusIdxVec_ac = d.bus_entire_ac.col(0).cast<int>();
        Eigen::VectorXi busAreaVec_ac = d.bus_entire_ac.col(numColsBuses_ac - 1).cast<int>();

        Eigen::MatrixXd bus_entire_ac_new = d.bus_entire_ac;
        for (int i = 0; i < numBuses_ac; ++i) {
            bus_entire_ac_new(i, 0) = i + 1;
        }

        std::map<std::pair<int, int>, int> mapping_ac;
        for (int i = 0; i < numBuses_ac; ++i) {
            mapping_ac[{ oldBusIdxVec_ac(i), busAreaVec_ac(i) }] = i + 1;
        }

        Eigen::MatrixXd branch_entire_ac_new = d.branch_entire_ac;
        int numBranches_ac = d.branch_entire_ac.rows();
        for (int i = 0; i < numBranches_ac; ++i) {
            int oldFrom_ac = static_cast<int>(d.branch_entire_ac(i, 0));
            int oldTo_ac = static_cast<int>(d.branch_entire_ac(i, 1));
            int area_ac = static_cast<int>(d.branch_entire_ac(i, numColsBranches_ac - 1));

            auto keyFrom_ac = std::make_pair(oldFrom_ac, area_ac);
            auto itFrom_ac = mapping_ac.find(keyFrom_ac);
            if (itFrom_ac != mapping_ac.end()) {
                branch_entire_ac_new(i, 0) = itFrom_ac->second;
            }

            auto keyTo_ac = std::make_pair(oldTo_ac, area_ac);
            auto itTo_ac = mapping_ac.find(keyTo_ac);
            if (itTo_ac != mapping_ac.end()) {
                branch_entire_ac_new(i, 1) = itTo_ac->second;
            }
        }

        // 2. reorder dc data
        int numBuses_dc = d.bus_dc.rows();

        Eigen::MatrixXd bus_dc_new = d.bus_dc;
        bus_dc_new.col(0).array() += numBuses_ac;      

        Eigen::MatrixXd branch_dc_new = d.branch_dc;
        int numBranches_dc = d.branch_dc.rows();

        branch_dc_new.leftCols<2>().array() += numBuses_ac;

        // 3. reorder conv data
        int numConvs = d.conv_dc.rows();
       
        Eigen::MatrixXd conv_dc_new = d.conv_dc;
        conv_dc_new.col(0).array() += numBuses_ac;

        std::map<std::pair<int, int>, int> mapping_conv;
        for (int i = 0; i < numBuses_ac; ++i) {
            int oldBusIdx_ac = static_cast<int>(d.bus_entire_ac(i, 0));
            int newBusIdx_ac = static_cast<int>(bus_entire_ac_new(i, 0));
            int area_ac = static_cast<int>(d.bus_entire_ac(i, numColsBuses_ac - 1));
            mapping_conv[{oldBusIdx_ac, area_ac}] = newBusIdx_ac;
        }

        for (int i = 0; i < numConvs; ++i) {
            int oldConvIdx_ac = static_cast<int>(d.conv_dc(i, 1));
            int area_ac = static_cast<int>(d.conv_dc(i, 2));
            auto it = mapping_conv.find({ oldConvIdx_ac, area_ac });
            if (it != mapping_conv.end()) {
                conv_dc_new(i, 1) = it->second;
            }
        }

        // 4. reorder generator data
        std::map<std::pair<int, int>, int> mappingGen;
        for (int i = 0; i < numBuses_ac; ++i) {
            int oldBusIdx_ac = static_cast<int>(d.bus_entire_ac(i, 0));
            int newBusIdx_ac = static_cast<int>(bus_entire_ac_new(i, 0));
            int area_ac = static_cast<int>(d.bus_entire_ac(i, numColsBuses_ac - 1));
            mappingGen[{oldBusIdx_ac, area_ac}] = newBusIdx_ac;
        }

        Eigen::MatrixXd gen_ac_entire_new = d.gen_entire_ac;
        int numGens = d.gen_entire_ac.rows();
        int numGenCols = d.gen_entire_ac.cols();
        
        for (int i = 0; i < numGens; ++i) {
            int oldBusIdx_gen = static_cast<int>(d.gen_entire_ac(i, 0));
            int area_ac = static_cast<int>(d.gen_entire_ac(i, numGenCols - 1));
            auto it = mappingGen.find({ oldBusIdx_gen, area_ac });
            if (it != mappingGen.end()) {
                gen_ac_entire_new(i, 0) = it->second;
            }
        }

        int idxGen = 0;
        for (int ng = 0; ng < d.ngrids; ++ng) {
            for (int j = 0; j < d.ngens_ac[ng]; ++j) {
                gen_ac_entire_new(idxGen, 1) = d.pgen_ac_k[ng](j) * d.baseMVA_ac;
                gen_ac_entire_new(idxGen, 2) = d.qgen_ac_k[ng](j) * d.baseMVA_ac;
                ++idxGen;
            }
        }

        
      /**************************************************
      * REORDER DATA MATRIXES
      **************************************************/
        
        int numNodes = numBuses_ac + numBuses_dc;
        Eigen::VectorXi acNodes(numBuses_ac),
                        dcNodes(numBuses_dc),
                        genNodes(numGens),
                        nodeNums(numNodes);

        acNodes = bus_entire_ac_new.col(0).cast<int>();   
        dcNodes = bus_dc_new.col(0).cast<int>();         
        genNodes = gen_ac_entire_new.col(0).cast<int>();  

        nodeNums.head(numBuses_ac) = acNodes;
        nodeNums.tail(numBuses_dc) = dcNodes;


      /**************************************************
      * CONSTRUCT NETWORK GRAPH
      **************************************************/
       
        // 1. build toNode
        const int nFrom = numBranches_ac + numBranches_dc + numConvs;
        Eigen::VectorXi fromNode(nFrom);
        
        fromNode.head(numBranches_ac) = branch_entire_ac_new.col(0).cast<int>();
        fromNode.segment(numBranches_ac, numBranches_dc) = branch_dc_new.col(0).cast<int>();
        fromNode.tail(numConvs) = conv_dc_new.col(0).cast<int>();

        // 2. build toNode
        auto nTo = nFrom;
        Eigen::VectorXi toNode(nTo);

        toNode.head(numBranches_ac) = branch_entire_ac_new.col(1).cast<int>();
        toNode.segment(numBranches_ac, numBranches_dc) = branch_dc_new.col(1).cast<int>();
        toNode.tail(numConvs) = conv_dc_new.col(1).cast<int>();

        // 3. merge allNodes
        auto nEdge = nTo;
        std::vector<int> allNodes;
        allNodes.reserve(2 * nEdge);
        allNodes.insert(allNodes.end(), fromNode.data(), fromNode.data() + fromNode.size());
        allNodes.insert(allNodes.end(), toNode.data(), toNode.data() + toNode.size());

        std::sort(allNodes.begin(), allNodes.end());
        allNodes.erase(std::unique(allNodes.begin(), allNodes.end()), allNodes.end());

        std::vector<std::pair<size_t, size_t>> edges;
        edges.reserve(fromNode.size());
        for (size_t k = 0; k < fromNode.size(); ++k) {
            size_t u = static_cast<size_t>(fromNode[k] - 1); 
            size_t v = static_cast<size_t>(toNode[k] - 1);
            if (u > v) std::swap(u, v);
            edges.emplace_back(u, v);
        }

        std::sort(edges.begin(), edges.end(),
            [](auto& a, auto& b) {
                if (a.first != b.first) return a.first < b.first;
                return a.second < b.second;
            });

        auto g = graph(edges);
        g->layout_algorithm(network::layout::kawai);

        auto xs = g->x_data();
        auto ys = g->y_data();


      /**************************************************
      * CONSTRUCT NETWORK GRAPH
      **************************************************/
        
        std::vector<size_t> idx_dc;
        idx_dc.reserve(dcNodes.size());
        for (int i = 0; i < dcNodes.size(); ++i) {
            idx_dc.push_back(static_cast<size_t>(dcNodes(i) - 1));
        }

        std::vector<size_t> idx_ac;
        idx_ac.reserve(xs.size());
        for (size_t i = 0; i < xs.size(); ++i) {
            if (std::find(idx_dc.begin(), idx_dc.end(), i) == idx_dc.end())
                idx_ac.push_back(i);
        }

        std::vector<double> x_ac, y_ac, x_dc, y_dc;
        x_ac.reserve(idx_ac.size());  y_ac.reserve(idx_ac.size());
        x_dc.reserve(idx_dc.size());  y_dc.reserve(idx_dc.size());
        for (auto i : idx_ac) {
            x_ac.push_back(xs[i]);
            y_ac.push_back(ys[i]);
        }
        for (auto i : idx_dc) {
            x_dc.push_back(xs[i]);
            y_dc.push_back(ys[i]);
        }

        std::vector<double> loadPower(xs.size(), 0), genPower(xs.size(), 0);
        for (size_t k = 0; k < idx_ac.size(); ++k) {
            size_t idx = idx_ac[k];
            double Pd = bus_entire_ac_new(idx, 2);
            double Qd = bus_entire_ac_new(idx, 3);
            loadPower[idx] = std::hypot(Pd, Qd);
        }

        for (int i = 0; i < gen_ac_entire_new.rows(); ++i) {
            size_t idx = static_cast<size_t>(gen_ac_entire_new(i, 0) - 1);
            double Pg = gen_ac_entire_new(i, 1);
            double Qg = gen_ac_entire_new(i, 2);
            genPower[idx] = std::hypot(Pg, Qg);
        }



        /*
       Edit node label
       */

        std::vector<int> orig_idx_ac(numBuses_ac);
        for (int i = 0; i < numBuses_ac; ++i) {
            orig_idx_ac[i] = static_cast<int>(d.bus_entire_ac(i, 0));
        }

        std::vector<int> orig_idx_dc(numBuses_dc);
        for (int i = 0; i < numBuses_dc; ++i) {
            orig_idx_dc[i] = static_cast<int>(d.bus_dc(i, 0));
        }


        Eigen::VectorXd voltMag_ac(xs.size());
        voltMag_ac.setZero();
        int idx = 0;
        for (int ng = 0; ng < d.ngrids; ++ng) {
            for (int i = 0; i < d.nbuses_ac[ng]; ++i) {
                voltMag_ac(idx) = std::sqrt(d.vn2_ac_k[ng](i));
                ++idx;
            }
        }

        Eigen::VectorXd voltMag_dc(xs.size());
        voltMag_dc.setZero();
        for (int i = 0; i < d.nbuses_dc; ++i) {
            size_t idx = static_cast<size_t>(d.bus_dc(i, 0) - 1);
            voltMag_dc(idx) = std::sqrt(d.vn2_dc_k(i));
        }

        std::unordered_map<int, int> acNodeToRow, dcNodeToRow;
        for (int i = 0; i < acNodes.size(); ++i)
            acNodeToRow[acNodes(i)] = i;
        for (int i = 0; i < dcNodes.size(); ++i)
            dcNodeToRow[dcNodes(i)] = i;

        Eigen::VectorXd voltLabel(allNodes.size());
        voltLabel.setZero();

        for (int i = 0; i < allNodes.size(); ++i) {
            int node = allNodes[i];
            if (acNodeToRow.count(node)) {
                int row = acNodeToRow[node];
                voltLabel(i) = voltMag_ac(row);
            }
            else if (dcNodeToRow.count(node)) {
                int row = dcNodeToRow[node];
                voltLabel(i) = voltMag_dc(row);
            }
            else {
                voltLabel(i) = 0.0;
            }
        }


        /*
        Edit branch
        */
        Eigen::MatrixXd acBranches = branch_entire_ac_new.block(0, 0, branch_entire_ac_new.rows(), 2);
        Eigen::MatrixXd dcBranches = branch_dc_new.block(0, 0, branch_dc_new.rows(), 2);
        Eigen::MatrixXd convBranches = conv_dc_new.block(0, 0, conv_dc_new.rows(), 2);

        std::unordered_set<int> acNodeSet;
        acNodeSet.reserve(acNodes.size());
        for (int i = 0; i < acNodes.size(); ++i) {
            acNodeSet.insert(acNodes(i));
        }

        std::unordered_set<int> dcNodeSet;
        dcNodeSet.reserve(dcNodes.size());
        for (int i = 0; i < dcNodes.size(); ++i) {
            dcNodeSet.insert(dcNodes(i));
        }

        std::vector<double> edgePower(edges.size(), 0.0);

        for (size_t k = 0; k < edges.size(); ++k) {
            auto [u, v] = edges[k];
            int bus_u = static_cast<int>(u) + 1;
            int bus_v = static_cast<int>(v) + 1;

            // 1) AC→AC
            if (acNodeSet.count(bus_u) && acNodeSet.count(bus_v)) {
                for (int row = 0; row < branch_entire_ac_new.rows(); ++row) {
                    int i_val = static_cast<int>(branch_entire_ac_new(row, 0));
                    int j_val = static_cast<int>(branch_entire_ac_new(row, 1));

                    if ((i_val == bus_u && j_val == bus_v) || (i_val == bus_v && j_val == bus_u)) {
                        int i = static_cast<int>(d.branch_entire_ac(row, 0));
                        int j = static_cast<int>(d.branch_entire_ac(row, 1));
                        int ng = static_cast<int>(d.branch_entire_ac(row, d.branch_entire_ac.cols() - 1));

                        double P = d.pij_ac_k[ng - 1](i - 1, j - 1);
                        double Q = d.qij_ac_k[ng - 1](i - 1, j - 1);
                        edgePower[k] = std::hypot(P, Q) * d.baseMVA_ac;
                        break;
                    }
                }
            }
            // 2) DC→DC
            else if (dcNodeSet.count(bus_u) && dcNodeSet.count(bus_v)) {
                for (int row = 0; row < branch_dc_new.rows(); ++row) {
                    int f_val = static_cast<int>(branch_dc_new(row, 0));
                    int h_val = static_cast<int>(branch_dc_new(row, 1));

                    if ((f_val == bus_u && h_val == bus_v) || (f_val == bus_v && h_val == bus_u)) {
                        int f = static_cast<int>(d.branch_dc(row, 0));
                        int h = static_cast<int>(d.branch_dc(row, 1));
                        double Pdc = d.pij_dc_k(f - 1, h - 1) * d.baseMW_dc * d.pol_dc;
                        edgePower[k] = std::abs(Pdc);
                        break;
                    }
                }
            }
            // 3) Converter 边
            else {
                for (int i = 0; i < d.nconvs_dc; ++i) {
                    int f = static_cast<int>(conv_dc_new(i, 0));
                    int t = static_cast<int>(conv_dc_new(i, 1));
                    if ((f == bus_u && t == bus_v) || (f == bus_v && t == bus_u)) {
                        double Ps = d.ps_dc_k[i] * d.baseMW_dc;
                        double Qs = d.qs_dc_k[i] * d.baseMW_dc;
                        edgePower[k] = std::hypot(Ps, Qs);
                        break;
                    }
                }
            }
        }

        auto minPower = *std::min_element(edgePower.begin(), edgePower.end());
        auto maxPower = *std::max_element(edgePower.begin(), edgePower.end());


        // 打开 Figure & Axes
        auto fig = figure(true);
        fig->size(1920 * 2, 1920);
        fig->color({ 1, 1, 1, 0 });
        auto ax_net = axes(fig);
        ax_net->hold(on);
        ax_net->box(false);
        ax_net->xticks({});  ax_net->yticks({});
        ax_net->axis({});
        ax_net->title("AC/DC OPF Results");


        // 画边
        bool first_edge = true;
        Eigen::MatrixXf edgeColors(edgePower.size(), 3);

        for (size_t k = 0; k < edgePower.size(); ++k) {
            float normVal = static_cast<float>(
                (edgePower[k] - minPower) / (maxPower - minPower + 1e-6f));
            auto rgb = oranges_colormap(normVal);  // std::array<float,3>
            edgeColors(k, 0) = rgb[0];  // R
            edgeColors(k, 1) = rgb[1];  // G
            edgeColors(k, 2) = rgb[2];  // B
        }

        for (size_t k = 0; k < edges.size(); ++k) {
            auto [u, v] = edges[k];
            std::vector<double> x = { xs[u], xs[v] };
            std::vector<double> y = { ys[u], ys[v] };

            std::array<float, 3> rgb = {
                edgeColors(k, 0),
                edgeColors(k, 1),
                edgeColors(k, 2)
            };

            auto h = plot(x, y, "r-");
            h->color(rgb);
            h->line_width(3);
            if (first_edge) {
                h->display_name("Branch Lines");
                first_edge = false;
            }
            else {
                h->display_name("");
            }

        }

        // 画节点
        bool first_gen = true;
        bool first_load = true;
        double factor = 0.22;
        for (size_t idx : idx_ac) {
            double loadSize = 1e-3 + loadPower[idx] * factor;
            double genSize = 1e-3 + genPower[idx] * factor;
            bool   hasGen = genPower[idx] > 0.0;

            std::vector<double> xp{ xs[idx] };
            std::vector<double> yp{ ys[idx] };

            if (hasGen) {
                if (loadSize >= genSize) {
                    auto h1 = scatter(xp, yp, loadSize);
                    h1->marker_face(true);
                    h1->marker_color({ 0.9f, 0.01f, 0.01f });
                    h1->marker_face_color({ 0.9f, 0.01f, 0.01f });
                    if (first_load) {
                        h1->display_name("AC Loads");
                        first_load = false;
                    }

                    auto h2 = scatter(xp, yp, genSize);
                    h2->marker_face(true);
                    h2->marker_color({ 0.01f, 0.5f, 0.5f });
                    h2->marker_face_color({ 0.01f, 0.5f, 0.5f });
                    h2->display_name("");
                    if (first_gen) {
                        h2->display_name("AC Generators");
                        first_gen = false;
                    }
                }
                else {
                    auto h3 = scatter(xp, yp, genSize);
                    h3->marker_face(true);
                    h3->marker_color({ 0.01f, 0.5f, 0.5f });
                    h3->marker_face_color({ 0.01f, 0.5f, 0.5f });
                    if (first_gen) {
                        h3->display_name("AC Generators");
                        first_gen = false;
                    }

                    auto h4 = scatter(xp, yp, loadSize);
                    h4->marker_face(true);
                    h4->marker_color({ 0.9f, 0.01f, 0.01f });
                    h4->marker_face_color({ 0.9f, 0.01f, 0.01f });
                    if (first_load) {
                        h4->display_name("AC Loads");
                        first_load = false;
                    }
                }
            }
            else {
                auto h5 = scatter(xp, yp, loadSize);
                h5->marker_face(true);
                h5->marker_color({ 0.9f, 0.01f, 0.01f });
                h5->marker_face_color({ 0.9f, 0.01f, 0.01f });
                h5->display_name("");
                if (first_load) {
                    h5->display_name("AC Loads");
                    first_load = false;
                }
            }
        }

        bool first_dc = true;
        auto h6 = scatter(x_dc, y_dc, 25);
        h6->marker_face(true);
        h6->marker_color({ 0.1f, 0.1f, 0.8f });
        h6->marker_face_color({ 0.1f, 0.1f, 0.8f });
        h6->marker_style(line_spec::marker_style::upward_pointing_triangle);
        if (first_dc) {
            h6->display_name("DC Node");
            first_dc = false;
        }


        // 添加节点标签
        for (auto idx : idx_ac) {
            std::string acNodeLabel = std::string("#") + std::to_string(orig_idx_ac[idx]);
            text(xs[idx], ys[idx], acNodeLabel)
                ->color({ 0.1f, 0.1f, 0.1f })
                .font_size(8);
        }

        for (auto idx : idx_dc) {
            int local_i = static_cast<int>(idx) - numBuses_ac;
            std::string dcNodeLabel = std::string("#") + std::to_string(orig_idx_dc[local_i]);
            text(xs[idx], ys[idx], dcNodeLabel)
                ->color({ 0.1f, 0.1f, 0.1f })
                .font_size(8);
        }

        for (int i = 0; i < allNodes.size(); ++i) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(3) << voltLabel(i) << " p.u.";
            text(xs[i] + 4, ys[i] + 4, oss.str())
                ->color({ 0.0f, 0.0f, 0.0f })
                .font_size(8);
        }

        // 网络画完后，记录视图限界
        auto orig_x = ax_net->xlim();
        auto orig_y = ax_net->ylim();


        // 1) 把 ax_net 压窄到左侧 0.05~0.80 区域，为色条留空间
        ax_net->position({ 0.2f, 0.10f, 0.60f, 0.80f });

        // -------------------------------------------------------------------------
        // (4) 创建窄轴 ax_bar 并用 image() 画连续渐变色块 --------------------------
        // -------------------------------------------------------------------------
        auto ax_bar = fig->add_axes();
        ax_bar->position({ 0.10f, 0.10f, 0.04f, 0.80f });
        ax_bar->xticks({});
        ax_bar->box(false);

        // ------ 1) 颜色映射保持不变 ------------------------------------------------
        std::vector<std::vector<double>> oranges_map;
        for (auto& c : oranges_colors) oranges_map.push_back({ c[0], c[1], c[2] });
        ax_bar->colormap(oranges_map);

        // ---- 2. 色条本身 --------------------------------------------------------
        constexpr int nseg = 200;
        vector_2d Z(nseg, vector_1d(2));
        for (int i = 0; i < nseg; ++i) {
            double v = minPower + (maxPower - minPower) * i / double(nseg - 1);
            Z[i][0] = Z[i][1] = v;
        }

        // 预留左侧 25?% 轴宽给刻度；色块只画右侧 75?%
        double gap = 0.25;                    // 0~gap 用来放刻度
        ax_bar->image(gap, 1.0,               // xmin, xmax
            0.0, double(nseg - 1),  // ymin, ymax
            Z, true);

        ax_bar->y_axis().reverse();
        ax_bar->ylim({ 0, nseg - 1 });
        ax_bar->xlim({ 0, 1 });                 // 0~gap 也要可见



        // ③ 在更右侧再写一个竖排标题
        //    Matplot++ 旧版 `text()` 支持 rotation(angle)；angle=90 就竖排
        ax_bar->title("Branch Power/MVA（MW）");
        ax_bar->title_font_size_multiplier(1.1);   // 字再大一点（可选）
        ax_bar->title_visible(true);               // 一般默认就是 true

        /* 如果想让标题跟色条居中，把 x?label 去掉以免占空间 */
        ax_bar->xlabel("");                        // 保证上方


        // -------------------------------------------------------------------------
        // (5) 在最右侧再开一个小轴：ax_legend ------------------------------------
        // -------------------------------------------------------------------------
        auto ax_legend = fig->add_axes();
        // 让它与 ax_net 垂直对齐，高度相同，宽度 0.10 (= 10%)
        ax_legend->position({ 0.82f, 0.30f, 0.12f, 0.45f });

        // 把坐标轴元素全部关掉
        ax_legend->box(true);
        ax_legend->xticks({});
        ax_legend->yticks({});
        ax_legend->xlim({ 0, 1 });
        ax_legend->ylim({ 0, 3 });
        ax_legend->hold(on);


        // y 坐标从上到下排 4 行
        double y0 = 2.6;
        double dy = 0.8;

        // ── 1) Branch Lines ───────────────────────────────────────────────
        ax_legend->plot({ 0.05, 0.35 }, { y0, y0 }, "r-")->line_width(3);
        ax_legend->text(0.45, y0, "Branch Lines")
            ->font_size(10)
            .alignment(matplot::labels::alignment::left);

        // ── 2) AC Loads (红圆) ────────────────────────────────────────────
        ax_legend->scatter({ 0.20 }, { y0 - dy }, 45)
            ->marker_face(true)
            .marker_color({ 0.85f,0.0f,0.0f })
            .marker_face_color({ 0.85f,0.0f,0.0f });
        ax_legend->text(0.45, y0 - dy, "AC Loads")
            ->font_size(10)
            .alignment(matplot::labels::alignment::left);

        // ── 3) AC Generators (绿圆) ───────────────────────────────────────
        ax_legend->scatter({ 0.20 }, { y0 - 2 * dy }, 45)
            ->marker_face(true)
            .marker_color({ 0.0f,0.6f,0.0f })
            .marker_face_color({ 0.0f,0.6f,0.0f });
        ax_legend->text(0.45, y0 - 2 * dy, "AC Generators")
            ->font_size(10)
            .alignment(matplot::labels::alignment::left);

        // ── 4) DC Nodes (蓝三角) ──────────────────────────────────────────
        ax_legend->scatter({ 0.20 }, { y0 - 3 * dy }, 45)
            ->marker_face(true)
            .marker_style(matplot::line_spec::marker_style::upward_pointing_triangle)
            .marker_color({ 0.1f,0.1f,0.85f })
            .marker_face_color({ 0.1f,0.1f,0.85f });
        ax_legend->text(0.45, y0 - 3 * dy, "VSC Converters")
            ->font_size(10)
            .alignment(matplot::labels::alignment::left);


        // 最后画图
        ax_net->axis(matplot::equal);
        // auto lgd = matplot::legend(ax_net, { "Branch Lines" , "Branch Lines" });
        // lgd->location(legend::general_alignment::bottomright);
        matplot::show();


}