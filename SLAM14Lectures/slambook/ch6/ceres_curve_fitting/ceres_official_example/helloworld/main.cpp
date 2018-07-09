#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

struct CostFunctor{
    template<typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = 10.0 - x[0];
        return true;
    }
};

int main(int argc, char **argv){
    google::InitGoogleLogging(argv[0]);
    double x = 0.5;
    const double initial_x = x;

    Problem problem;

    CostFunction* cost_function = new AutoDiffCostFunction<CostFunctor, 1, 1> (new CostFunctor);
    problem.AddResidualBlock(cost_function, NULL, &x);

    // run the Solver
    Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << "x : " << initial_x
            << " -> " << x << "\n";
}