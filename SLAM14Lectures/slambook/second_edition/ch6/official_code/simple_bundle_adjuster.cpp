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

struct SnavelyReprojectionError
{
    SnavelyReprojectionError(double x, double y) : observed_x(x), observed_y(y) {}
    template <typename T>
    bool operator()(const T *camera, const T *point, T *residual) const
    {
        T p[3];
        ceres::AngleAxisRotatePoint(camera, point, p);
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];
        T xp = -p[0] / p[2];
        T yp = -p[1] / p[2];
        T r2 = xp * xp + yp * yp;
        T distortion = 1.0 + camera[7] * r2 + camera[8] * r2 * r2;

        T predict_x = camera[6] * distortion * xp;
        T predict_y = camera[6] * distortion * yp;

        residual[0] = predict_x - observed_x;
        residual[1] = predict_y - observed_y;

        return true;
    }
    static ceres::CostFunction *Create(double x, double y)
    {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
            new SnavelyReprojectionError(x, y)));
    }

private:
    double observed_x;
    double observed_y;
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
        ceres::CostFunction *cost_function = SnavelyReprojectionError::Create(observations[2 * i], observations[2 * i + 1]);
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
