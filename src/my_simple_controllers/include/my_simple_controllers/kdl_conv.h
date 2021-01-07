#include <kdl/frames.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace kdl_conv {

/// Convert a KDL vector to an Eigen vector.
Eigen::Vector3d toEigen(KDL::Vector const & input);

/// Convert an Eigen vector to a KDL vector.
KDL::Vector toKdlVector(Eigen::Vector3d const & input);

/// Convert a KDL rotation to an Eigen quaternion.
Eigen::Quaterniond toEigen(KDL::Rotation const & input);

/// Convert a KDL rotation to an Eigen rotation matrix.
Eigen::Matrix3d toEigenMatrix(KDL::Rotation const & input);

/// Convert an Eigen quaternion to a KDL rotation.
KDL::Rotation toKdlRotation(Eigen::Quaterniond const & input);

/// Convert an Eigen rotation matrix to a KDL rotation.
KDL::Rotation toKdlRotation(Eigen::Matrix3d const & input);

/// Convert a KDL frame to an Eigen isometry.
Eigen::Isometry3d toEigen(KDL::Frame const & input);

/// Convert an Eigen isometry to a KDL frame.
KDL::Frame toKdlFrame(Eigen::Isometry3d const & input);

}