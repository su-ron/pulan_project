
#include "common/path/path_utils.h"

namespace planning {

void PathUtils::ExtendAlong(const double &distance,
                            std::vector<Point> *initial_path,
                            const double &interval, const double &direction) {
  if (initial_path->empty()) return;

  int num_point = static_cast<int>(distance / interval);
  Point last_point = initial_path->back();
  double dir = last_point.yaw();

  Point tmp;
  for (int i = 1; i < num_point; ++i) {
    tmp.set_x(last_point.x() + interval * i * direction * cos(dir));
    tmp.set_y(last_point.y() + interval * i * direction * sin(dir));
    tmp.set_yaw(dir);
    initial_path->push_back(tmp);
  }

  double residual_dis = distance - num_point * interval;
  if (residual_dis < std::numeric_limits<double>::epsilon()) return;

  tmp.set_x(initial_path->back().x() + residual_dis * direction * cos(dir));
  tmp.set_y(initial_path->back().y() + residual_dis * direction * sin(dir));
  tmp.set_yaw(dir);
  initial_path->push_back(tmp);
}

void PathUtils::EstimateKappa(const double &max_kappa, std::vector<Point> *path,
                              bool use_filter) {
  // 具体算法实现细节参考链接 :
  // https://zhuanlan.zhihu.com/p/72083902
  const double eps = 10e-4;
  if (path->size() < 3) {
    printf("The path length is less than 3 for estimation of kappa \n");
    return;
  }

  Matrix M, X, Y;
  M.resize(3, 3);
  X.resize(3, 1);
  Y.resize(3, 1);
  size_t path_len = path->size();
  for (int i = 1; i < path_len - 1; ++i) {
    Point p1 = path->operator[](i - 1);
    Point p2 = path->operator[](i);
    Point p3 = path->operator[](i + 1);
    double t_a = p2 - p1;
    double t_b = p3 - p2;

    M << 1, -t_a, t_a * t_a, 1, 0, 0, 1, t_b, t_b * t_b;
    X << p1.x(), p2.x(), p3.x();
    Y << p1.y(), p2.y(), p3.y();

    Matrix A = M.colPivHouseholderQr().solve(X);
    Matrix B = M.colPivHouseholderQr().solve(Y);

    double a2 = A(1, 0);  // dx
    double a3 = A(2, 0);  // 0.5ddx
    double b2 = B(1, 0);  // dy
    double b3 = B(2, 0);  // 0.5ddy
    double kappa = -2 * (a3 * b2 - a2 * b3) / pow(a2 * a2 + b2 * b2 + eps, 1.5);
    path->operator[](i).set_kappa(kappa);
    path->at(i).set_dx(a2);
    path->at(i).set_dy(b2);
    path->at(i).set_ddx(2 * a3);
    path->at(i).set_ddy(2 * b3);
  }

  path->front().set_kappa(path->operator[](1).kappa());
  path->front().set_dx(path->operator[](1).dx());
  path->front().set_dy(path->operator[](1).dy());
  path->front().set_ddx(path->operator[](1).ddx());
  path->front().set_ddy(path->operator[](1).ddy());

  path->back().set_kappa(path->operator[](path_len - 2).kappa());
  path->back().set_dx(path->operator[](path_len - 2).dx());
  path->back().set_dy(path->operator[](path_len - 2).dy());
  path->back().set_ddx(path->operator[](path_len - 2).ddx());
  path->back().set_ddy(path->operator[](path_len - 2).ddy());

  if (use_filter) {
    math::Butterworth2ndFilter lp(0.1, 4);
    for (auto &point : *path) {
      double filtered_kappa = lp.filter(point.kappa());
      if (fabs(filtered_kappa) > max_kappa) {
        filtered_kappa = std::copysign(max_kappa, filtered_kappa);
      }
      point.set_kappa(filtered_kappa);
    }
  }
}

}  // namespace planning

