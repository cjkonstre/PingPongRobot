//updates the pulley positions in kin_conf. uses the current values as starting pos
//uses least squares 

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "config/config_loader.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <fstream>

//#define DO_MANUAL_POINTS
std::array<std::array<double, 3>, 5> autopoints = {{
    {{0, 1.2716, 0}},
    {{0, 0, 0}}, 
    {{1.524, 0, 0}}, 
    {{1.524, 1.2716, 0}},
    {{0.6, 1.2716, 0}}
}} ;

struct CableResidual {
    CableResidual(const Eigen::Vector3d& ai, double Li)
        : ai_(ai), Li_(Li) {}

    template <typename T>
    bool operator()(const T* const c, T* residuals) const {
        Eigen::Map<const Eigen::Matrix<T,3,1>> C(c);
        Eigen::Matrix<T,3,1> A = ai_.cast<T>();

        residuals[0] = (C - A).norm() - T(Li_);

        return true;
    }

private:
    const Eigen::Vector3d ai_;
    const double Li_;
};

int main() {

    //getting initial guess
    auto kinConfig = load_configs(KINCONFIG_PATH);
    auto initialGuesses = kinConfig.pulleyPoss;

    for (int anchorI = 0; anchorI<7; anchorI++){
        ceres::Problem problem;
        std::cout << "doing anchor " << anchorI << "\n";

        std::vector<Eigen::Vector3d> points;
        std::vector<double> lengths;
        std::cout << "Collecting data, to finish enter nothing. finishing w/o any data skips this pulley.\n";
        #ifdef DO_MANUAL_POINTS
        while (true) {
            std::string line;
            Eigen::Vector3d point;

            std::cout << "points x (meters, empty to finish): ";
            std::getline(std::cin, line);

            if (line.empty()) break;

            double x;
            point[0] = std::stod(line);

            std::cout << "points y (meters): ";
            std::cin >> point[1];

            // ---- Z ----
            std::cout << "points z (meters): ";
            std::cin >> point[2];

            points.push_back(point);

            // ---- Length ----
            std::cout << "length to pulley (meters): ";
            double L; std::cin >> L;
            lengths.push_back(L);

            // Clear trailing newline before next getline
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
        #else

        for (const auto& point : autopoints) {
            std::cout << "point = ["
                    << point[0] << ", "
                    << point[1] << ", "
                    << point[2] << "]\n";

            std::cout << "length to pulley (meters): ";
            std::string line;
            std::getline(std::cin, line);

            if (line.empty()) break;

            double L = std::stod(line);
            lengths.push_back(L);

            points.emplace_back(point[0], point[1], point[2]);
        }

        
        #endif
        if (points.size() == 0) {
            std::cout << "skipping anchor " << anchorI << "\n";
            continue;
        }

        std::array<double, 3> c = initialGuesses[anchorI];

        for (size_t i = 0; i < points.size(); i++) {
            ceres::CostFunction* cost =
                new ceres::AutoDiffCostFunction<CableResidual, 1, 3>(
                    new CableResidual(points[i], lengths[i])
                );

            problem.AddResidualBlock(cost, nullptr, c.data());
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        std::cout << summary.FullReport() << "\n";
        float dist = sqrt(pow(c[0]-initialGuesses[anchorI][0],2)+pow(c[1]-initialGuesses[anchorI][1],2)+pow(c[2]-initialGuesses[anchorI][2],2));
        std::cout << "c = [" << c[0] << ", " << c[1] << ", " << c[2] << "] ; this is " 
                << dist*100 << "cm from the origional value and " << sqrt(c[0]*c[0]+c[1]*c[1]+c[2]*c[2]) << "m from the origin" << std::endl;
         
        std::cout << "Write to config file? (y/): ";
        std::string input;
        std::getline(std::cin, input);
        if (input == "y") {

            std::ifstream in(KINCONFIG_PATH);
            json infile = json::parse(in);

            infile["pulleyPoss"][anchorI] = c;

            std::ofstream out(KINCONFIG_PATH);
            out << std::setw(4) << infile << std::endl;

            in.close();
            out.close();
            
            std::cout << "pulley " << anchorI << " updated.\n";
        }
    }
}

/*
"pulleyPoss": [
    [1.9912, 1.3416000000000001, 1.11015],
    [1.824, 1.3416000000000001, 0.008799999999999919],
    [-0.43720000000000003, -0.4907, 0.04335],
    [0.727, -0.4907, 0.8890000000000001],
    [-0.46720000000000006, 1.3416000000000001, 1.11015],
    [-0.3, 1.3416000000000001, 0.008799999999999919],
    [1.9612, -0.4907, 0.04335]
]

*/

/*

[
    1.9835611744365131,
    1.3337334946279185,
    1.0932458063988064
],
[
    1.826755136845263,
    1.3605526285275644,
    0.06168588955560601
],
[
    -0.4584794413044624,
    -0.47862628495184334,
    0.0984639792158707
],
[
    0.7355521194130822,
    -0.5223411548938575,
    0.8799066342131675
],
[
    -0.45886463088712875,
    1.3551759397375387,
    1.1256236574304723
],
[
    -0.2768275554559478,
    1.3375516626006927,
    0.13083551126750334
],
[
    2.3588821086801586,
    -0.2335090468285671,
    0.0016995361536422775
]*/

/*
[
            1.983561693620706,
            1.3337335817245108,
            1.0932452467209843
        ],
        [
            1.8267553842103121,
            1.3605528667310667,
            0.06165042874540345
        ],
        [
            -0.45847934245385047,
            -0.4786260267147885,
            0.09846721548561574
        ],
        [
            0.7355521194130822,
            -0.5223411548938575,
            0.8799066342131675
        ],
        [
            -0.4588646224284371,
            1.3551759509383308,
            1.1256236607129633
        ],
        [
            -0.2768275554559478,
            1.3375516626006927,
            0.13083551126750334
        ],
        [
            1.9950554448029416,
            -0.4947567427418786,
            0.09318473263263329
        ]
*/