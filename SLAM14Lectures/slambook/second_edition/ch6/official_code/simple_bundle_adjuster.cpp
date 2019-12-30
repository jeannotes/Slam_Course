#include <cmath>
#include <cstdio>
#include <iostream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
using namespace std;
DEFINE_bool(big_menu, true, "Include 'advanced' options in the menu listing");

class BALProblem
{
public:
    BALProblem() {}
    ~BALProblem()
    {
        delete[] camera_index_;
        delete[] point_index_;
        delete[] parameters_;
    }

    const int num_observations() const { return num_observations_; }
    const double *observations() const { return observations_; }
    double *mutable_cameras() { return parameters_; }
    double *mutable_points() { return parameters_ + 9 * num_cameras_; }
    double *mutable_camera_for_observation(int i) { return mutable_cameras() + camera_index_[i] * 9; }
    double *mutable_point_for_observation(int i) { return mutable_points() + point_index_[i] * 3; }

    int LoadFile(const char *filename)
    {
        FILE *fptr = fopen(filename, "r");
        if (fptr == NULL)
        {
            LOG(FATAL) << "BAD FILE";
            return 0;
        }
        FscanfOrDie(fptr, "%d", &num_cameras_);
        FscanfOrDie(fptr, "%d", &num_points_);
        FscanfOrDie(fptr, "%d", &num_observations_);
        num_parameters_ = num_cameras_ * 9 + num_points_ * 3;
        // load observations
        camera_index_ = new int[num_observations_];
        point_index_ = new int[num_observations_];
        observations_ = new double[2 * num_observations_];
        for (int i = 0; i < num_observations_; i++)
        {
            FscanfOrDie(fptr, "%d", camera_index_ + i);
            FscanfOrDie(fptr, "%d", point_index_ + i);
            for (int j = 0; j < 2; j++)
            {
                FscanfOrDie(fptr, "%lf", observations_ + 2 * i + j);
            }
        }
        //load parameters
        parameters_ = new double[num_parameters_];
        for (int i = 0; i < num_parameters_; i++)
        {
            FscanfOrDie(fptr, "%lf", parameters_ + i);
        }
        return 1;
    }

    template <typename T>
    void FscanfOrDie(FILE *fptr, const char *format, T *value)
    {
        int res = fscanf(fptr, format, value);
        if (res != 1)
            LOG(FATAL) << "INVALID FILE";
    }

private:
    int num_cameras_;
    int num_points_;
    int num_observations_;
    int num_parameters_;

    int *camera_index_;
    int *point_index_;
    double *observations_;
    double *parameters_;
};

// struct ReprojectionError
// {
//     ReprojectionError(double x, double y) : observed_x(x), observed_y(y) {}
//     template <typename T>
//     bool operator()(const T *camera, const T *point, T *residual) const
//     {
//         T p[3];
//         ceres::AngleAxisRotatePoint(camera, point, p);
//         p[0] += camera[3];
//         p[1] += camera[4];
//         p[2] += camera[5];
//         T xp = -p[0] / p[2];
//         T yp = -p[1] / p[2];
//         T r2 = xp * xp + yp * yp;
//         T distortion = 1.0 + camera[7] * r2 + camera[8] * r2 * r2;

//         T predict_x = camera[6] * distortion * xp;
//         T predict_y = camera[6] * distortion * yp;

//         residual[0] = predict_x - observed_x;
//         residual[1] = predict_y - observed_y;

//         return true;
//     }
//     static ceres::CostFunction *Create(double x, double y)
//     {
//         return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 9, 3>(
//             new ReprojectionError(x, y)));
//     }

// private:
//     double observed_x;
//     double observed_y;
// };

struct ReprojectionError
{
    ReprojectionError(double x, double y) : observed_x(x), observed_y(y) {}
    template<typename T>
    bool operator()(const T* camera,const T* point,T* residual)const{
        T p[3];
        ceres::AngleAxisRotatePoint(camera, point, p);
        p[0] += camera[0];
        p[1] += camera[1];
        p[2] += camera[2];
        T xp = -p[0] / p[2];
        T yp = -p[1] / p[2];
        T r2 = xp*xp+yp*yp;
        T distortion = 1.0 + camera[7] * r2 + camera[8] * r2 * r2;
        T predict_x = xp * camera[6] * distortion;
        T predict_y = yp * camera[6] * distortion;

        residual[0] = predict_x - observed_x;
        residual[1] = predict_y - observed_y;

        return true;
    }

    static ceres::CostFunction* Create(double x,double y){
        return (new ceres::AutoDiffCostFunction<ReprojectionError,2,9,3>(new  ReprojectionError(x,y)));
    }

protected:
    double observed_x, observed_y;
};

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    BALProblem bal_problem;
    if (!bal_problem.LoadFile(argv[1]))
    {
        LOG(FATAL) << "fatal opening file and argv:" << argv[1];
        return -1;
    }
    const double *observations = bal_problem.observations();
    ceres::Problem problem;
    for (int i = 0; i < bal_problem.num_observations(); i++)
    {
        ceres::CostFunction *cost_function = ReprojectionError::Create(observations[2 * i], observations[2 * i + 1]);
        problem.AddResidualBlock(cost_function, NULL,
                                 bal_problem.mutable_camera_for_observation(i),
                                 bal_problem.mutable_point_for_observation(i));
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(WARNING) << "OK";
    cout << "\n"
         << summary.FullReport();
    return 0;
}

/*
Solver Summary (v 2.0.0-eigen-(3.2.92)-lapack-suitesparse-(4.4.6)-cxsparse-(3.1.4)-eigensparse-no_openmp)

                                     Original                  Reduced
Parameter blocks                        22122                    22122
Parameters                              66462                    66462
Residual blocks                         83718                    83718
Residuals                              167436                   167436

Minimizer                        TRUST_REGION

Dense linear algebra library            EIGEN
Trust region strategy     LEVENBERG_MARQUARDT

                                        Given                     Used
Linear solver                     DENSE_SCHUR              DENSE_SCHUR
Threads                                     1                        1
Linear solver ordering              AUTOMATIC                 22106,16
Schur structure                         2,3,9                    2,3,9

Cost:
Initial                          4.185660e+06
Final                            1.803390e+04
Change                           4.167626e+06

Minimizer iterations                        7
Successful steps                            7
Unsuccessful steps                          0

Time (in seconds):
Preprocessor                         0.106646

  Residual only evaluation           0.081070 (7)
  Jacobian & residual evaluation     0.371161 (7)
  Linear solver                      0.359499 (7)
Minimizer                            0.892232

Postprocessor                        0.003854
Total                                1.002732

Termination:                      CONVERGENCE (Function tolerance reached. |cost_change|/cost: 1.769764e-09 <= 1.000000e-06)

*/