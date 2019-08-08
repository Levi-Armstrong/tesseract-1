#ifndef DESCARTES_TESSERACT_KINEMATICS_H
#define DESCARTES_TESSERACT_KINEMATICS_H

#include <Eigen/Geometry>
#include <vector>
#include <descartes_light/interface/kinematics_interface.h>
#include <descartes_light/utils.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>

namespace tesseract_motion_planners
{
/** @brief Provides a Descartes interface for Tesseract Kinematics */
template <typename FloatType>
class DescartesTesseractKinematics : public descartes_light::KinematicsInterface<FloatType>
{
public:
  DescartesTesseractKinematics(const tesseract_kinematics::ForwardKinematics::ConstPtr tesseract_fk,
                               const tesseract_kinematics::InverseKinematics::ConstPtr tesseract_ik,
                               const descartes_light::IsValidFn<FloatType>& is_valid_fn,
                               const descartes_light::GetRedundantSolutionsFn<FloatType>& redundant_sol_fn) :
    tesseract_fk_(tesseract_fk),
    tesseract_ik_(tesseract_ik)
  {}

  bool ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
          std::vector<FloatType>& solution_set) const override;
  bool fk(const FloatType* pose, Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const override;

  int dof() const override;

  void analyzeIK(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p) const override;

  void setIKSeed(const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1> >& seed);

private:

  tesseract_kinematics::ForwardKinematics::ConstPtr tesseract_fk_;
  tesseract_kinematics::InverseKinematics::ConstPtr tesseract_ik_;
  descartes_light::IsValidFn<FloatType> is_valid_fn_;
  descartes_light::GetRedundantSolutionsFn<FloatType> redundant_sol_fn_;
  Eigen::VectorXd ik_seed_;

};

using DescartesTesseractKinematicsD = DescartesTesseractKinematics<double>;
using DescartesTesseractKinematicsF = DescartesTesseractKinematics<float>;
}

#endif
