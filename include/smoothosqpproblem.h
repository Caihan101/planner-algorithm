#ifndef SMOOTHOSQPPROBLEM_H
#define SMOOTHOSQPPROBLEM_H

#include <utility>
#include <vector>
#include <climits>
#include <cfloat>
#include <iostream>
#include <array>
#include <vector>
#include <tuple>
#include <cmath>
#include <limits>
#include <numeric>

#include <osqp/osqp.h>



class FemPosDeviationSqpOsqpInterface {
 public:
  //优化器的构造函数
  FemPosDeviationSqpOsqpInterface() = default;

  virtual ~FemPosDeviationSqpOsqpInterface() = default;

  //传入参考点，即需要优化量
  void set_ref_points(
      const std::vector<std::pair<double, double>>& ref_points) {
    ref_points_ = ref_points;
  }

  //传入优化量的上下边界，已弃用
  void set_bounds_around_refs(const std::vector<double>& bounds_around_refs) {
    bounds_around_refs_ = bounds_around_refs;
  }

  //传入边界框，即优化量的上下边界
  void set_boundsBox(const std::vector<std::vector<double>>& bounds_box) {
    bounds_box_ = bounds_box;
  }

  //设定目标函数中位置偏移量权重
  void set_weight_fem_pos_deviation(const double weight_fem_pos_deviation) {
    weight_fem_pos_deviation_ = weight_fem_pos_deviation;
  }

  //设定目标函数中路径长度权重
  void set_weight_path_length(const double weight_path_length) {
    weight_path_length_ = weight_path_length;
  }

  //设定目标函数中曲率权重
  void set_weight_ref_deviation(const double weight_ref_deviation) {
    weight_ref_deviation_ = weight_ref_deviation;
  }

  //设定目标函数中曲率松弛量权重
  void set_weight_curvature_constraint_slack_var(
      const double weight_curvature_constraint_slack_var) {
    weight_curvature_constraint_slack_var_ =
        weight_curvature_constraint_slack_var;
  }

  //设定曲率约束
  void set_curvature_constraint(const double curvature_constraint) {
    curvature_constraint_ = curvature_constraint;
  }

  //设定osqp单次求解最大迭代次数
  void set_max_iter(const int max_iter) { max_iter_ = max_iter; }

  //设定时间限制
  void set_time_limit(const double time_limit) { time_limit_ = time_limit; }

  //设定osqp求解器中的Verbose，默认为false
  void set_verbose(const bool verbose) { verbose_ = verbose; }

  //设定osqp求解器中的scaled_termination， 默认为true
  void set_scaled_termination(const bool scaled_termination) {
    scaled_termination_ = scaled_termination;
  }

  //设定osqp求解器中的warm_start， 默认为true
  void set_warm_start(const bool warm_start) { warm_start_ = warm_start; }

  // 设置外层最大迭代次数
  void set_sqp_pen_max_iter(const int sqp_pen_max_iter) {
    sqp_pen_max_iter_ = sqp_pen_max_iter;
  }

  // 设置外层迭代停止阈值
  void set_sqp_ftol(const double sqp_ftol) { sqp_ftol_ = sqp_ftol; }

  // 设置内层最大迭代次数
  void set_sqp_sub_max_iter(const int sqp_sub_max_iter) {
    sqp_sub_max_iter_ = sqp_sub_max_iter;
  }

  // 设置内层迭代停止阈值
  void set_sqp_ctol(const double sqp_ctol) { sqp_ctol_ = sqp_ctol; }

  // 求解
  bool Solve();

  // 外部获取结果的接口
  const std::vector<std::pair<double, double>>& opt_xy() const {
    return opt_xy_;
  }

 private:

  // 初始化kernal
  void CalculateKernel(std::vector<c_float>* P_data,
                       std::vector<c_int>* P_indices,
                       std::vector<c_int>* P_indptr);

  // 计算offset 
  void CalculateOffset(std::vector<c_float>* q);
  
  //将三点转化为线性项
  std::vector<double> CalculateLinearizedFemPosParams(
      const std::vector<std::pair<double, double>>& points, const size_t index);

  // 计算仿射约束
  void CalculateAffineConstraint(
      const std::vector<std::pair<double, double>>& points,
      std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
      std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
      std::vector<c_float>* upper_bounds);

  // 设置PrimalWarmStart
  void SetPrimalWarmStart(const std::vector<std::pair<double, double>>& points,
                          std::vector<c_float>* primal_warm_start);

  // 优化求解
  bool OptimizeWithOsqp(const std::vector<c_float>& primal_warm_start,
                        OSQPWorkspace** work);

  // 计算约束损失
  double CalculateConstraintViolation(
      const std::vector<std::pair<double, double>>& points);

 private:
  // Init states and constraints
  // 优化量 参考点
  std::vector<std::pair<double, double>> ref_points_;

  // 上下边界， box形式
  std::vector<std::vector<double>> bounds_box_;

  // 上下边界， 不再使用
  std::vector<double> bounds_around_refs_;

  // 曲率约束
  double curvature_constraint_ = 0.15;

  // Weights in optimization cost function 
  // 位置偏移量权重
  double weight_fem_pos_deviation_ = 1.0e6;
  
  // 路径长度权重
  double weight_path_length_ = 1.0e2;

  // 曲率权重
  double weight_ref_deviation_ = 1.0e1;

  // 曲率约束松弛遍历约束
  double weight_curvature_constraint_slack_var_ = 1.0e4;

  // Settings of osqp 
  //最大迭代次数
  int max_iter_ = 4000;

  //时间限制
  double time_limit_ = 0.0;

  //osqp求解器中的设置，默认false
  bool verbose_ = false;

  //osqp求解器中的设置，默认true
  bool scaled_termination_ = true;

  //osqp求解器中的设置，默认ture
  bool warm_start_ = true;

  // Settings of sqp 最大迭代次数以及终止条件
  int sqp_pen_max_iter_ = 300;
  double sqp_ftol_ = 1.0e-2;
  int sqp_sub_max_iter_ = 300;
  double sqp_ctol_ = 1.0;

  // Optimization problem definitions
  //参考点的数量
  int num_of_points_ = 0;

  //位置变量数量，等于参考点数量二倍
  int num_of_pos_variables_ = 0;

  //松弛变量数量，等于参考点数量减2
  int num_of_slack_variables_ = 0;
  
  //变量总数量 所有变量数量之和
  int num_of_variables_ = 0;

  //变量约束数量，等于变量数量
  int num_of_variable_constraints_ = 0;

  //曲率约束数量 等于参考点数量减2
  int num_of_curvature_constraints_ = 0;

  //约束总数量 所有约束数量之和
  int num_of_constraints_ = 0;

  // Optimized_result
  // 优化结果
  std::vector<std::pair<double, double>> opt_xy_;

  // 松弛变量
  std::vector<double> slack_;

  // 平均间隔长度
  double average_interval_length_ = 0.0;
};



#endif // SMOOTHOSQPPROBLEM_H
