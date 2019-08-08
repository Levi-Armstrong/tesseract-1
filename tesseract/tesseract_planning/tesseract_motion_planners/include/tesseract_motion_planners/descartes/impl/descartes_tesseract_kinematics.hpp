#include <tesseract_motion_planners/descartes/descartes_tesseract_kinematics.h>
#include <Eigen/Eigen>
#include <console_bridge/console.h>

namespace tesseract_motion_planners {

template <typename FloatType>
bool DescartesTesseractKinematics<FloatType>::ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
        std::vector<FloatType>& solution_set) const
{
  // Convert to appropriate Eigen types
  Eigen::Isometry3d p_double;
  p_double = p.template cast<double> ();
  Eigen::VectorXd solution_eigen;

  // Solve IK
  bool success = tesseract_ik_->calcInvKin(solution_eigen, p_double, ik_seed_);

  // Convert back to vector
  std::vector<FloatType> solution_vec(solution_eigen.data(), solution_eigen.data() + solution_eigen.rows() * solution_eigen.cols());
  solution_set = solution_vec;

  // TODO: Figure this out
//   Convert to array for redundant solutions
//  FloatType* sol = solution_vec.data() + tesseract_ik_->numJoints();
//  std::vector<FloatType> redundant_sol = redundant_sol_fn_(sol);

//  // Convert back to std::vector
//  if (is_valid_fn_(redundant_sol))
//    solution_set = redundant_sol;
////    solution_set.insert(end(solution_set), redundant_sol, redundant_sol + tesseract_ik_->numJoints());  // If good then add to solution

  return success;
}

template <typename FloatType>
bool DescartesTesseractKinematics<FloatType>::fk(const FloatType* pose, Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const
{
  // Convert the Array to an Eigen VectorX<FloatType>
  Eigen::Matrix<FloatType, Eigen::Dynamic, 1> joints;
  for (int i = 0; i < tesseract_fk_->numJoints(); i++)
    joints(i, 0) = pose[i];

  // Get the solution from the Tesseract Kinematics
  Eigen::Isometry3d solution_double;
  bool success = tesseract_fk_->calcFwdKin(solution_double, joints);

  // Cast from double to FloatType
  solution = solution_double.cast <FloatType>();
  return success;
}

template <typename FloatType>
int DescartesTesseractKinematics<FloatType>::dof() const
{
  return tesseract_fk_->numJoints();
}

template <typename FloatType>
void DescartesTesseractKinematics<FloatType>::analyzeIK(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p) const
{
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "AnalyzeIK: ", ";");

  std::stringstream ss;
  ss << p.matrix().format(CommaInitFmt);
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  std::string valid_fn_defined = "\tIs Valid Function: " + std::string(is_valid_fn_ ? "True" : "False");
  CONSOLE_BRIDGE_logInform(valid_fn_defined.c_str());
  std::string redundant_fn_defined = "\tIs Valid Function: " + std::string(is_valid_fn_ ? "True" : "False");
  CONSOLE_BRIDGE_logInform(redundant_fn_defined.c_str());

//  std::vector<FloatType> solution_set;
//  ik(p, nullptr, nullptr, solution_set);
//  ss.clear();
//  ss << "\tSampling without functions, found solutions: " << solution_set.size() / 8;
//  CONSOLE_BRIDGE_logInform(ss.str().c_str());

//  solution_set.clear();
//  ik(p, is_valid_fn_, nullptr, solution_set);
//  ss.clear();
//  ss << "\tSampling with only IsValid functions, found solutions: " << solution_set.size() / 8;
//  CONSOLE_BRIDGE_logInform(ss.str().c_str());

//  solution_set.clear();
//  ik(p, nullptr, redundant_sol_fn_, solution_set);
//  ss.clear();
//  ss << "\tSampling with only Redundant Solutions functions, found solutions: " << solution_set.size() / 8;
//  CONSOLE_BRIDGE_logInform(ss.str().c_str());

//  solution_set.clear();
//  ik(p, is_valid_fn_, redundant_sol_fn_, solution_set);
//  ss.clear();
//  ss << "\tSampling with both functions, found solutions: " << solution_set.size() / 8;
//  CONSOLE_BRIDGE_logInform(ss.str().c_str());
}

template <typename FloatType>
void DescartesTesseractKinematics<FloatType>::setIKSeed(const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1> >& seed)
{
  ik_seed_ = seed.template cast <double> ();
}
}
