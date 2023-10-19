/*
 * helpers.hpp
 *
 *  Created on: Apr 22, 2021
 *      Author: jelavice
 */

#pragma once

#include "icp_localization_ros2/common/typedefs.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Geometry/Transform.h>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
// #include "pointmatcher/
// #include "pointmatcher_ros/transform.h"
#include "pointmatcher/IO.h"
#include "pointmatcher/Parametrizable.h"
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/PointMatcherPrivate.h"
#include "pointmatcher/Registrar.h"
#include "pointmatcher/Timer.h"
#include <boost/algorithm/string/erase.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <memory>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace icp_loco {

Eigen::Vector3d getPositionFromParameterServer(std::shared_ptr<rclcpp::Node> nh,
                                               const std::string &prefix);
Eigen::Quaterniond
getOrientationFromParameterServer(std::shared_ptr<rclcpp::Node> nh,
                                  const std::string &prefix,
                                  bool isRPYInDegrees = false);

template <typename Scalar>
Eigen::Matrix<Scalar, -1, -1>
getTransformationMatrix(const Eigen::Matrix<Scalar, 3, 1> &position,
                        const Eigen::Quaternion<Scalar> &orientation);

template <typename Scalar>
void getPositionAndOrientation(const Eigen::Matrix<Scalar, -1, -1> &T,
                               Eigen::Matrix<Scalar, 3, 1> *position,
                               Eigen::Quaternion<Scalar> *orientation);

Eigen::Quaternionf toFloat(const Eigen::Quaterniond &q);
Eigen::Quaterniond toDouble(const Eigen::Quaternionf &q);

Eigen::MatrixXf toFloat(const Eigen::MatrixXd &m);
Eigen::MatrixXd toDouble(const Eigen::MatrixXf &m);

} // namespace icp_loco

template <typename Scalar>
Eigen::Matrix<Scalar, -1, -1> icp_loco::getTransformationMatrix(
    const Eigen::Matrix<Scalar, 3, 1> &position,
    const Eigen::Quaternion<Scalar> &orientation) {
  Eigen::Matrix<Scalar, -1, -1> T(4, 4);
  T.setIdentity();
  T.block(0, 3, 3, 1) = position;
  T.block(0, 0, 3, 3) = orientation.toRotationMatrix();
  return T;
}

template <typename Scalar>
geometry_msgs::msg::Pose
eigenMatrixToPoseMsg(const Eigen::Matrix<Scalar, -1, -1> &T) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = T(0, 3);
  pose.position.y = T(1, 3);
  pose.position.z = T(2, 3);
  Eigen::Isometry3d iso;
  iso.matrix() = T;
  Eigen::Quaternion<Scalar> q(iso.rotation());
  q.normalize();
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  return pose;
}

template <typename Scalar>
void icp_loco::getPositionAndOrientation(
    const Eigen::Matrix<Scalar, -1, -1> &T,
    Eigen::Matrix<Scalar, 3, 1> *position,
    Eigen::Quaternion<Scalar> *orientation) {
  geometry_msgs::msg::Pose pose = eigenMatrixToPoseMsg<Scalar>(T);
  *position = Eigen::Matrix<Scalar, 3, 1>(pose.position.x, pose.position.y,
                                          pose.position.z);
  const auto &q = pose.orientation;
  *orientation = Eigen::Quaternion<Scalar>(q.w, q.x, q.y, q.z);
  orientation->normalize();
}

template <typename T>
typename PointMatcher<T>::DataPoints
rosMsgToPointMatcherCloud(const sensor_msgs::msg::PointCloud2 &rosMsg,
                          const bool isDense) {
  using namespace std;
  // FIXME: continue from here, need to decode time properly
  typedef PointMatcher<T> PM;
  typedef PointMatcherIO<T> PMIO;
  typedef typename PMIO::PMPropTypes PM_types;
  typedef typename PM::DataPoints DataPoints;
  typedef typename DataPoints::Label Label;
  typedef typename DataPoints::Labels Labels;
  typedef typename DataPoints::View View;
  typedef typename DataPoints::TimeView TimeView;

  if (rosMsg.fields.empty())
    return DataPoints();

  // fill labels
  // conversions of descriptor fields from pcl
  // see http://www.ros.org/wiki/pcl/Overview
  Labels featLabels;
  Labels descLabels;
  Labels timeLabels;
  vector<bool> isFeature;
  vector<PM_types> fieldTypes;
  for (auto it(rosMsg.fields.begin()); it != rosMsg.fields.end(); ++it) {
    const string name(it->name);
    const size_t count(std::max<size_t>(it->count, 1));
    if (name == "x" || name == "y" || name == "z") {
      featLabels.push_back(Label(name, count));
      isFeature.push_back(true);
      fieldTypes.push_back(PM_types::FEATURE);
    } else if (name == "rgb" || name == "rgba") {
      descLabels.push_back(Label("color", (name == "rgba") ? 4 : 3));
      isFeature.push_back(false);
    } else if ((it + 1) != rosMsg.fields.end() && it->name == "normal_x" &&
               (it + 1)->name == "normal_y") {
      if ((it + 2) != rosMsg.fields.end() && (it + 2)->name == "normal_z") {
        descLabels.push_back(Label("normals", 3));
        it += 2;
        isFeature.push_back(false);
        isFeature.push_back(false);
        fieldTypes.push_back(PM_types::DESCRIPTOR);
        fieldTypes.push_back(PM_types::DESCRIPTOR);
      } else {
        descLabels.push_back(Label("normals", 2));
        it += 1;
        isFeature.push_back(false);
        fieldTypes.push_back(PM_types::DESCRIPTOR);
      }
      isFeature.push_back(false);
      fieldTypes.push_back(PM_types::DESCRIPTOR);
    } else if ((it + 1) != rosMsg.fields.end() &&
               boost::algorithm::ends_with(name, "_splitTime_high32") &&
               boost::algorithm::ends_with(((it + 1)->name),
                                           "_splitTime_low32")) {
      // time extraction
      // const string beginning = name.substr(0, name.size()-4);
      string startingName = name;
      boost::algorithm::erase_last(startingName, "_splitTime_high32");
      const string beginning = startingName;

      timeLabels.push_back(Label(beginning, 1));
      it += 1;
      isFeature.push_back(false);
      fieldTypes.push_back(PM_types::TIME);
      fieldTypes.push_back(PM_types::TIME);
    }
    // else if (name == "stamps")
    else if (name == "time") {
      timeLabels.push_back(Label(name, count));
      isFeature.push_back(false);
    } else {
      descLabels.push_back(Label(name, count));
      isFeature.push_back(false);
      fieldTypes.push_back(PM_types::DESCRIPTOR);
    }
  }

  featLabels.push_back(Label("pad", 1));
  assert(isFeature.size() == rosMsg.fields.size());
  assert(fieldTypes.size() == rosMsg.fields.size());

  // create cloud
  const unsigned pointCount(rosMsg.width * rosMsg.height);
  DataPoints cloud(featLabels, descLabels, timeLabels, pointCount);
  cloud.getFeatureViewByName("pad").setConstant(1);

  // fill cloud
  // TODO: support big endian, pass through endian-swapping method just after
  // the *reinterpret_cast
  typedef sensor_msgs::msg::PointField PF;
  size_t fieldId = 0;
  for (auto it(rosMsg.fields.begin()); it != rosMsg.fields.end();
       ++it, ++fieldId) {
    if (it->name == "rgb" || it->name == "rgba") {
      // special case for colors
      if (((it->datatype != PF::UINT32) && (it->datatype != PF::INT32) &&
           (it->datatype != PF::FLOAT32)) ||
          (it->count != 1))
        throw runtime_error(
            (boost::format("Colors in a point cloud must be a single element "
                           "of size 32 bits, found %1% elements of type %2%") %
             it->count % unsigned(it->datatype))
                .str());
      View view(cloud.getDescriptorViewByName("color"));
      int ptId(0);
      for (size_t y(0); y < rosMsg.height; ++y) {
        const uint8_t *dataPtr(&rosMsg.data[0] + rosMsg.row_step * y);
        for (size_t x(0); x < rosMsg.width; ++x) {
          const uint32_t rgba(
              *reinterpret_cast<const uint32_t *>(dataPtr + it->offset));
          const T colorA(T((rgba >> 24) & 0xff) / 255.);
          const T colorR(T((rgba >> 16) & 0xff) / 255.);
          const T colorG(T((rgba >> 8) & 0xff) / 255.);
          const T colorB(T((rgba >> 0) & 0xff) / 255.);
          view(0, ptId) = colorR;
          view(1, ptId) = colorG;
          view(2, ptId) = colorB;
          if (view.rows() > 3)
            view(3, ptId) = colorA;
          dataPtr += rosMsg.point_step;
          ptId += 1;
        }
      }
    } else if (boost::algorithm::ends_with(it->name, "_splitTime_high32") ||
               boost::algorithm::ends_with(it->name, "_splitTime_low32")) {
      string startingName = it->name;
      bool isHigh = false;
      if (boost::algorithm::ends_with(it->name, "_splitTime_high32")) {
        boost::algorithm::erase_last(startingName, "_splitTime_high32");
        isHigh = true;
      }
      if (boost::algorithm::ends_with(it->name, "_splitTime_low32")) {
        boost::algorithm::erase_last(startingName, "_splitTime_low32");
      }

      TimeView timeView(cloud.getTimeViewByName(startingName));
      // use view to read data

      int ptId(0);
      const size_t count(std::max<size_t>(it->count, 1));
      for (size_t y(0); y < rosMsg.height; ++y) {
        const uint8_t *dataPtr(&rosMsg.data[0] + rosMsg.row_step * y);
        for (size_t x(0); x < rosMsg.width; ++x) {
          const uint8_t *fPtr(dataPtr + it->offset);
          for (unsigned dim(0); dim < count; ++dim) {
            if (isHigh) {
              const uint32_t high32 = *reinterpret_cast<const uint32_t *>(fPtr);
              const uint32_t low32 = uint32_t(timeView(dim, ptId));
              timeView(dim, ptId) =
                  (((uint64_t)high32) << 32) | ((uint64_t)low32);
            } else {
              const uint32_t high32 = uint32_t(timeView(dim, ptId) >> 32);
              const uint32_t low32 = *reinterpret_cast<const uint32_t *>(fPtr);
              timeView(dim, ptId) =
                  (((uint64_t)high32) << 32) | ((uint64_t)low32);
            }
            dataPtr += rosMsg.point_step;
            ptId += 1;
          }
        }
      }

    } else {

      // get view for editing data
      View view(
          (it->name == "normal_x")
              ? cloud.getDescriptorRowViewByName("normals", 0)
              : ((it->name == "normal_y")
                     ? cloud.getDescriptorRowViewByName("normals", 1)
                     : ((it->name == "normal_z")
                            ? cloud.getDescriptorRowViewByName("normals", 2)
                            : ((isFeature[fieldId])
                                   ? cloud.getFeatureViewByName(it->name)
                                   : cloud.getDescriptorViewByName(
                                         it->name)))));

      // use view to read data
      int ptId(0);
      const size_t count(std::max<size_t>(it->count, 1));
      for (size_t y(0); y < rosMsg.height; ++y) {
        const uint8_t *dataPtr(&rosMsg.data[0] + rosMsg.row_step * y);
        for (size_t x(0); x < rosMsg.width; ++x) {
          const uint8_t *fPtr(dataPtr + it->offset);
          for (unsigned dim(0); dim < count; ++dim) {
            switch (it->datatype) {
            case PF::INT8:
              view(dim, ptId) = T(*reinterpret_cast<const int8_t *>(fPtr));
              fPtr += 1;
              break;
            case PF::UINT8:
              view(dim, ptId) = T(*reinterpret_cast<const uint8_t *>(fPtr));
              fPtr += 1;
              break;
            case PF::INT16:
              view(dim, ptId) = T(*reinterpret_cast<const int16_t *>(fPtr));
              fPtr += 2;
              break;
            case PF::UINT16:
              view(dim, ptId) = T(*reinterpret_cast<const uint16_t *>(fPtr));
              fPtr += 2;
              break;
            case PF::INT32:
              view(dim, ptId) = T(*reinterpret_cast<const int32_t *>(fPtr));
              fPtr += 4;
              break;
            case PF::UINT32:
              view(dim, ptId) = T(*reinterpret_cast<const uint32_t *>(fPtr));
              fPtr += 4;
              break;
            case PF::FLOAT32:
              view(dim, ptId) = T(*reinterpret_cast<const float *>(fPtr));
              fPtr += 4;
              break;
            case PF::FLOAT64:
              view(dim, ptId) = T(*reinterpret_cast<const double *>(fPtr));
              fPtr += 8;
              break;
            default:
              abort();
            }
          }
          dataPtr += rosMsg.point_step;
          ptId += 1;
        }
      }
    }
  }

  if (isDense == false) {
    shared_ptr<typename PM::DataPointsFilter> filter(
        PM::get().DataPointsFilterRegistrar.create(
            "RemoveNaNDataPointsFilter"));
    return filter->filter(cloud);
  }

  return cloud;
}

template <typename T>
sensor_msgs::msg::PointCloud2
pointMatcherCloudToRosMsg(const typename PointMatcher<T>::DataPoints &pmCloud,
                          const std::string &frame_id,
                          const rclcpp::Time &stamp) {
  using namespace std;
  sensor_msgs::msg::PointCloud2 rosCloud;
  typedef sensor_msgs::msg::PointField PF;

  // check type and get sizes
  BOOST_STATIC_ASSERT(is_floating_point<T>::value);
  BOOST_STATIC_ASSERT((is_same<T, long double>::value == false));
  uint8_t dataType;
  size_t scalarSize;
  if (typeid(T) == typeid(float)) {
    dataType = PF::FLOAT32;
    scalarSize = 4;
  } else {
    dataType = PF::FLOAT64;
    scalarSize = 8;
  }

  size_t timeSize = 4; // we split in two UINT32

  // build labels

  // features
  unsigned offset(0);
  assert(!pmCloud.featureLabels.empty());
  assert(pmCloud.featureLabels[pmCloud.featureLabels.size() - 1].text == "pad");
  for (auto it(pmCloud.featureLabels.begin());
       it != pmCloud.featureLabels.end(); ++it) {
    // last label is padding
    if ((it + 1) == pmCloud.featureLabels.end())
      break;
    PF pointField;
    pointField.name = it->text;
    pointField.offset = offset;
    pointField.datatype = dataType;
    pointField.count = it->span;
    rosCloud.fields.push_back(pointField);
    offset += it->span * scalarSize;
  }
  bool addZ(false);
  if (!pmCloud.featureLabels.contains("z")) {
    PF pointField;
    pointField.name = "z";
    pointField.offset = offset;
    pointField.datatype = dataType;
    pointField.count = 1;
    rosCloud.fields.push_back(pointField);
    offset += scalarSize;
    addZ = true;
  }

  // descriptors
  const bool isDescriptor(!pmCloud.descriptorLabels.empty());
  bool hasColor(false);
  unsigned colorPos(0);
  unsigned colorCount(0);
  unsigned inDescriptorPos(0);
  for (auto it(pmCloud.descriptorLabels.begin());
       it != pmCloud.descriptorLabels.end(); ++it) {
    PF pointField;
    if (it->text == "normals") {
      assert((it->span == 2) || (it->span == 3));
      pointField.datatype = dataType;
      pointField.name = "normal_x";
      pointField.offset = offset;
      pointField.count = 1;
      rosCloud.fields.push_back(pointField);
      offset += scalarSize;
      pointField.name = "normal_y";
      pointField.offset = offset;
      pointField.count = 1;
      rosCloud.fields.push_back(pointField);
      offset += scalarSize;
      if (it->span == 3) {
        pointField.name = "normal_z";
        pointField.offset = offset;
        pointField.count = 1;
        rosCloud.fields.push_back(pointField);
        offset += scalarSize;
      }
    } else if (it->text == "color") {
      colorPos = inDescriptorPos;
      colorCount = it->span;
      hasColor = true;
      pointField.datatype =
          (colorCount == 4) ? uint8_t(PF::UINT32) : uint8_t(PF::FLOAT32);
      pointField.name = (colorCount == 4) ? "rgba" : "rgb";
      pointField.offset = offset;
      pointField.count = 1;
      rosCloud.fields.push_back(pointField);
      offset += 4;
    } else {
      pointField.datatype = dataType;
      pointField.name = it->text;
      pointField.offset = offset;
      pointField.count = it->span;
      rosCloud.fields.push_back(pointField);
      offset += it->span * scalarSize;
    }
    inDescriptorPos += it->span;
  }

  // time
  bool hasTime(false);
  for (auto it(pmCloud.timeLabels.begin()); it != pmCloud.timeLabels.end();
       ++it) {
    PF pointField;
    // if (it->text == "stamps")
    if (it->text == "time") {
      hasTime = true;

      // for Rviz view

      pointField.datatype = PF::FLOAT32;

      // pointField.datatype = PF::UINT32;
      pointField.name = "elapsedTimeSec";
      pointField.offset = offset;
      pointField.count = 1;
      rosCloud.fields.push_back(pointField);
      offset += 4;

      // Split time in two because there is not PF::UINT64
      pointField.datatype = PF::UINT32;
      pointField.name = it->text + "_splitTime_high32";
      pointField.offset = offset;
      pointField.count = 1;
      rosCloud.fields.push_back(pointField);
      offset += timeSize;

      pointField.datatype = PF::UINT32;
      pointField.name = it->text + "_splitTime_low32";
      pointField.offset = offset;
      pointField.count = 1;
      rosCloud.fields.push_back(pointField);
      offset += timeSize;
    }
  }

  // fill cloud with data
  rosCloud.header.frame_id = frame_id;
  rosCloud.header.stamp = stamp;
  rosCloud.height = 1;
  rosCloud.width = pmCloud.features.cols();
#ifdef BOOST_BIG_ENDIAN
  rosCloud.is_bigendian = true;
#else  // BOOST_BIG_ENDIAN
  rosCloud.is_bigendian = false;
#endif // BOOST_BIG_ENDIAN
  rosCloud.point_step = offset;
  rosCloud.row_step = rosCloud.point_step * rosCloud.width;
  rosCloud.is_dense = true;
  rosCloud.data.resize(rosCloud.row_step * rosCloud.height);

  const unsigned featureDim(pmCloud.features.rows() - 1);
  const unsigned descriptorDim(pmCloud.descriptors.rows());
  const unsigned timeDim(pmCloud.times.rows());

  assert(descriptorDim == inDescriptorPos);
  const unsigned postColorPos(colorPos + colorCount);
  assert(postColorPos <= inDescriptorPos);
  const unsigned postColorCount(descriptorDim - postColorPos);

  for (unsigned pt(0); pt < rosCloud.width; ++pt) {
    uint8_t *fPtr(&rosCloud.data[pt * offset]);

    memcpy(fPtr, reinterpret_cast<const uint8_t *>(&pmCloud.features(0, pt)),
           scalarSize * featureDim);
    fPtr += scalarSize * featureDim;
    if (addZ) {
      memset(fPtr, 0, scalarSize);
      fPtr += scalarSize;
    }
    if (isDescriptor) {
      if (hasColor) {
        // before color
        memcpy(fPtr,
               reinterpret_cast<const uint8_t *>(&pmCloud.descriptors(0, pt)),
               scalarSize * colorPos);
        fPtr += scalarSize * colorPos;
        // compact color
        uint32_t rgba;
        unsigned colorR(unsigned(pmCloud.descriptors(colorPos + 0, pt) * 255.) &
                        0xFF);
        unsigned colorG(unsigned(pmCloud.descriptors(colorPos + 1, pt) * 255.) &
                        0xFF);
        unsigned colorB(unsigned(pmCloud.descriptors(colorPos + 2, pt) * 255.) &
                        0xFF);
        unsigned colorA(0);
        if (colorCount == 4)
          colorA =
              unsigned(pmCloud.descriptors(colorPos + 3, pt) * 255.) & 0xFF;
        rgba = colorA << 24 | colorR << 16 | colorG << 8 | colorB;
        memcpy(fPtr, reinterpret_cast<const uint8_t *>(&rgba), 4);
        fPtr += 4;
        // after color
        memcpy(fPtr,
               reinterpret_cast<const uint8_t *>(
                   &pmCloud.descriptors(postColorPos, pt)),
               scalarSize * postColorCount);
        fPtr += scalarSize * postColorCount;
      } else {
        memcpy(fPtr,
               reinterpret_cast<const uint8_t *>(&pmCloud.descriptors(0, pt)),
               scalarSize * descriptorDim);
        fPtr += scalarSize * descriptorDim;
      }
    }

    // TODO: reactivate that properly
    // if(isTime)
    //{
    //	for(unsigned d = 0; d<timeDim; d++)
    //	{
    //		const uint32_t nsec = (uint32_t) pmCloud.times(d,pt);
    //		const uint32_t sec = (uint32_t) (pmCloud.times(d,pt) >> 32);
    //		memcpy(fPtr, reinterpret_cast<const uint8_t*>(&sec), timeSize);
    //		fPtr += timeSize;
    //		memcpy(fPtr, reinterpret_cast<const uint8_t*>(&nsec), timeSize);
    //		fPtr += timeSize;
    //	}
    //}
    if (hasTime) {
      // PointCloud2 can not contain uint64_t variables
      // uint32_t are used for publishing, pmCloud.times(0, pt)/1000 (time in
      // micro seconds)

      // uint32_t temp = (uint32_t)(pmCloud.times(0, pt)/(uint64_t)1000);

      const size_t ptrSize = timeSize * timeDim;

      // Elapsed time
      const float elapsedTime =
          (float)(pmCloud.times(0, pt) - pmCloud.times(0, 0)) * 1e-9f;
      memcpy(fPtr, reinterpret_cast<const uint8_t *>(&elapsedTime), ptrSize);
      fPtr += ptrSize;

      // high32
      const uint32_t high32 = (uint32_t)(pmCloud.times(0, pt) >> 32);
      memcpy(fPtr, reinterpret_cast<const uint8_t *>(&high32), ptrSize);
      fPtr += ptrSize;

      // low32
      const uint32_t low32 = (uint32_t)(pmCloud.times(0, pt));
      memcpy(fPtr, reinterpret_cast<const uint8_t *>(&low32), ptrSize);
      fPtr += ptrSize;
    }
  }

  // fill remaining information
  rosCloud.header.frame_id = frame_id;
  rosCloud.header.stamp = stamp;

  return rosCloud;
}

namespace pointmatcher_ros {

template <typename ScalarType> class RosPointCloud2Deserializer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PM = PointMatcher<ScalarType>;
  using DataPoints = typename PM::DataPoints;

  /**
   * @brief Deserializes a sensor_msgs/PointCloud2 objects into a Pointmatcher
   * point cloud (DataPoints)
   *
   * @param rosMsg        Input message of type sensor_msgs/PointCloud2.
   * @return DataPoints   Output point cloud.
   */
  static DataPoints deserialize(const sensor_msgs::msg::PointCloud2 &rosMsg);

private:
  using PMIO = PointMatcherIO<ScalarType>;
  using PM_types = typename PMIO::PMPropTypes;
  using Index = typename DataPoints::Index;
  using Label = typename DataPoints::Label;
  using Labels = typename DataPoints::Labels;
  // using IndexMatrix = typename PM::IndexMatrix;
  // using IndexGridView = typename DataPoints::IndexGridView;
  using View = typename DataPoints::View;
  using FieldNamesList = std::vector<std::string>;

  /**
   * @brief Extract the feature and descriptor labels of a given
   * sensor_msgs/PointCloud2 message.
   *
   * @param rosMsg        Input message of type sensor_msgs/PointCloud2.
   * @param featLabels    Feature labels. Assumed to be empty, will be filled by
   * this method.
   * @param descLabels    Descriptor labels. Assumed to be empty, will be filled
   * by this method.
   */
  static void extractFieldLabels(const sensor_msgs::msg::PointCloud2 &rosMsg,
                                 Labels &featLabels, Labels &descLabels);

  /**
   * @brief Fills data from a scalar descriptor into a Pointmatcher point cloud.
   *
   * @param rosMsg        Input message of type sensor_msgs/PointCloud2.
   * @param fieldName     Name of the point cloud field that corresponds to the
   * descriptor.
   * @param pointCount    Number of points in the input point cloud.
   * @param view          View on the output point cloud, will be modified.
   */
  static void
  fillScalarDataIntoView(const sensor_msgs::msg::PointCloud2 &rosMsg,
                         const std::string &fieldName, const size_t pointCount,
                         View &view);

  /**
   * @brief Fills data from a vector feature or descriptor into a Pointmatcher
   * point cloud.
   *
   * @param rosMsg            Input message of type sensor_msgs/PointCloud2.
   * @param fieldName         Name of the point cloud field that corresponds to
   * the descriptor.
   * @param is3dPointCloud    Whether the point cloud is 3D or 2D.
   * @param pointCount        Number of points in the input point cloud.
   * @param view              View on the output point cloud, will be modified.
   */
  static void
  fillVectorDataIntoView(const sensor_msgs::msg::PointCloud2 &rosMsg,
                         const FieldNamesList &fieldNames,
                         const bool is3dPointCloud, const size_t pointCount,
                         View &view);

  /**
   * @brief Fills data from a color descriptor into a Pointmatcher point cloud.
   *
   * @param rosMsg            Input message of type sensor_msgs/PointCloud2.
   * @param fieldName         Name of the point cloud field that corresponds to
   * the descriptor.
   * @param pointCount        Number of points in the input point cloud.
   * @param view              View on the output point cloud, will be modified.
   */
  static void fillColorDataIntoView(const sensor_msgs::msg::PointCloud2 &rosMsg,
                                    const FieldNamesList &fieldNames,
                                    const size_t pointCount, View &view);

  /**
   * @brief Fills data from a Lidar Ring descriptor into a Pointmatcher point
   * cloud.
   * @remark This method is needed because Hesai Lidars rings are usually
   * encoded as 16-bit uints, not floating-point.
   *
   * @param rosMsg
   * @param fieldName
   * @param pointCount
   * @param view
   */
  static void
  fillPerPointRingDataIntoView(const sensor_msgs::msg::PointCloud2 &rosMsg,
                               const std::string &fieldName,
                               const size_t pointCount, View &view);

  /**
   * @brief Fills data from an Absolute Timestamp descriptor into a Pointmatcher
   * point cloud.
   * @remark This method is needed because Hesai Lidars store their timestamps
   * in Unix Time, which breaks downstream nodes that expect a relative stamp.
   * This method transforms the timestamp to be relative to the start of the
   * Lidar spin.
   *
   * @param rosMsg
   * @param fieldName
   * @param pointCount
   * @param view
   */
  static void fillPerPointAbsoluteTimestampDataIntoView(
      const sensor_msgs::msg::PointCloud2 &rosMsg, const std::string &fieldName,
      const size_t pointCount, View &view);

  /**
   * @brief Fills the 'index grid' from a ROS message into a Pointmatcher point
   * cloud.
   * @remark This method expectes the ROS message to contain a non-dense
   * (organized) point cloud.
   *
   * @param rosMsg            Input message of type sensor_msgs/PointCloud2.
   * @param pointCount        Number of points in the input point cloud.
   * @param cloud             Point cloud. Its index grid will be modified
   */
  // static void fillIndexGrid(const sensor_msgs::msg::PointCloud2 &rosMsg,
  //                           const size_t pointCount, DataPoints &cloud);

  /**
   * @brief Fills a Pointmatcher point cloud with data from a
   * sensor_msgs/PointCloud2 message.
   *
   * @param rosMsg            Input message of type sensor_msgs/PointCloud2.
   * @param is3dPointCloud    Whether the point cloud is 3D or 2D.
   * @param pointCloud        View on the output point cloud, will be modified.
   */
  static void fillPointCloudValues(const sensor_msgs::msg::PointCloud2 &rosMsg,
                                   const bool is3dPointCloud,
                                   DataPoints &pointCloud);
};

template <typename ScalarType>
typename RosPointCloud2Deserializer<ScalarType>::DataPoints
RosPointCloud2Deserializer<ScalarType>::deserialize(
    const sensor_msgs::msg::PointCloud2 &rosMsg) {
  // If the msg is empty return an empty point cloud.
  if (rosMsg.fields.empty()) {
    return DataPoints();
  }

  // Label containers.
  Labels featLabels;
  Labels descLabels;

  // Fill field labels.
  extractFieldLabels(rosMsg, featLabels, descLabels);

  // Create cloud
  const size_t pointCount{rosMsg.width * rosMsg.height};
  DataPoints pointCloud(featLabels, descLabels, pointCount);

  // Determine the point cloud dimensionality (2D or 3D).
  // All points are represented in homogeneous coordinates, so dim 4 -> 3D and
  // dim 3 -> 2D.
  const bool is3dPointCloud{(featLabels.size() - 1) == 3};

  // Fill cloud with data.
  fillPointCloudValues(rosMsg, is3dPointCloud, pointCloud);

  return pointCloud;
}

template <typename ScalarType>
void RosPointCloud2Deserializer<ScalarType>::extractFieldLabels(
    const sensor_msgs::msg::PointCloud2 &rosMsg, Labels &featLabels,
    Labels &descLabels) {
  // Conversions of descriptor fields from pcl.
  // see http://www.ros.org/wiki/pcl/Overview
  std::vector<PM_types> fieldTypes;
  std::vector<bool> isFeature;
  for (auto it(rosMsg.fields.begin()); it != rosMsg.fields.end(); ++it) {
    const std::string &name(it->name);
    const size_t count{std::max<size_t>(it->count, 1)};
    if (name == "x" || name == "y" || name == "z") {
      featLabels.push_back(Label(name, count));
      isFeature.push_back(true);
      fieldTypes.push_back(PM_types::FEATURE);
    } else if (name == "rgb" || name == "rgba") {
      descLabels.push_back(Label("color", (name == "rgba") ? 4 : 3));
      isFeature.push_back(false);
      fieldTypes.push_back(PM_types::DESCRIPTOR);
    } else if ((it + 1) != rosMsg.fields.end() && it->name == "normal_x" &&
               (it + 1)->name == "normal_y") {
      if ((it + 2) != rosMsg.fields.end() && (it + 2)->name == "normal_z") {
        descLabels.push_back(Label("normals", 3));
        isFeature.push_back(false);
        isFeature.push_back(false);
        fieldTypes.push_back(PM_types::DESCRIPTOR);
        fieldTypes.push_back(PM_types::DESCRIPTOR);
        it += 2;
      } else {
        descLabels.push_back(Label("normals", 2));
        isFeature.push_back(false);
        fieldTypes.push_back(PM_types::DESCRIPTOR);
        it += 1;
      }
      isFeature.push_back(false);
      fieldTypes.push_back(PM_types::DESCRIPTOR);
    } else {
      descLabels.push_back(Label(name, count));
      isFeature.push_back(false);
      fieldTypes.push_back(PM_types::DESCRIPTOR);
    }
  }

  // Add padding after features.
  // Libpointmatcher represents points in homogeneous coordinates and padding is
  // just another name for the scale factor.
  featLabels.push_back(Label("pad", 1));

  // TODO(ynava) The variables 'isFeature' and 'fieldTypes' are just kept for
  // running assertions. Consider removing them and performing a runtime check.
  assert(isFeature.size() == rosMsg.fields.size());
  assert(fieldTypes.size() == rosMsg.fields.size());
}

template <typename ScalarType>
void RosPointCloud2Deserializer<ScalarType>::fillScalarDataIntoView(
    const sensor_msgs::msg::PointCloud2 &rosMsg, const std::string &fieldName,
    const size_t pointCount, View &view) {
  // Use iterator to read data and write it into view.
  sensor_msgs::PointCloud2ConstIterator<ScalarType> iter(rosMsg, fieldName);
  for (size_t i = 0; i < pointCount; ++i, ++iter) {
    view(0, i) = *iter;
  }
}

template <typename ScalarType>
void RosPointCloud2Deserializer<ScalarType>::fillVectorDataIntoView(
    const sensor_msgs::msg::PointCloud2 &rosMsg,
    const FieldNamesList &fieldNames, const bool is3dPointCloud,
    const size_t pointCount, View &view) {
  // Create iterators to read data from the message buffer.
  sensor_msgs::PointCloud2ConstIterator<ScalarType> iterX(rosMsg,
                                                          fieldNames[0]);
  sensor_msgs::PointCloud2ConstIterator<ScalarType> iterY(rosMsg,
                                                          fieldNames[1]);

  // Dispatch a deserialization routine based on dimensions.
  if (is3dPointCloud) {
    sensor_msgs::PointCloud2ConstIterator<ScalarType> iterZ(rosMsg,
                                                            fieldNames[2]);
    for (size_t i = 0; i < pointCount; ++i, ++iterX, ++iterY, ++iterZ) {
      view(0, i) = *iterX;
      view(1, i) = *iterY;
      view(2, i) = *iterZ;
    }
  } else {
    for (size_t i = 0; i < pointCount; ++i, ++iterX, ++iterY) {
      view(0, i) = *iterX;
      view(1, i) = *iterY;
    }
  }
}

template <typename ScalarType>
void RosPointCloud2Deserializer<ScalarType>::fillColorDataIntoView(
    const sensor_msgs::msg::PointCloud2 &rosMsg,
    const FieldNamesList &fieldNames, const size_t pointCount, View &view) {
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iterR(rosMsg, fieldNames[0]);
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iterG(rosMsg, fieldNames[1]);
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iterB(rosMsg, fieldNames[2]);
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iterA(rosMsg, fieldNames[3]);
  for (size_t i = 0; i < pointCount; ++i, ++iterR, ++iterG, ++iterB, ++iterA) {
    // PointCloud2Iterator implicitly casts to the type specified in its
    // template arguments.
    view(0, i) = static_cast<int>(*iterR) / 255.0;
    view(1, i) = static_cast<int>(*iterG) / 255.0;
    view(2, i) = static_cast<int>(*iterB) / 255.0;
    if (view.rows() > 3) {
      view(3, i) = static_cast<int>(*iterA) / 255.0;
    }
  }
}

template <typename ScalarType>
void RosPointCloud2Deserializer<ScalarType>::fillPerPointRingDataIntoView(
    const sensor_msgs::msg::PointCloud2 &rosMsg, const std::string &fieldName,
    const size_t pointCount, View &view) {
  // Use iterator to read data and write it into view.
  sensor_msgs::PointCloud2ConstIterator<uint16_t> iter(rosMsg, fieldName);
  for (size_t i = 0; i < pointCount; ++i, ++iter) {
    view(0, i) = static_cast<int>((*iter));
  }
}

template <typename ScalarType>
void RosPointCloud2Deserializer<ScalarType>::
    fillPerPointAbsoluteTimestampDataIntoView(
        const sensor_msgs::msg::PointCloud2 &rosMsg,
        const std::string &fieldName, const size_t pointCount, View &view) {
  // Use iterator to read data and write it into view.
  sensor_msgs::PointCloud2ConstIterator<double> iter(rosMsg, fieldName);
  const double scanTimestamp{rosMsg.header.stamp.sec +
                             rosMsg.header.stamp.nanosec * 1e-9};
  for (size_t i = 0; i < pointCount; ++i, ++iter) {
    view(0, i) = static_cast<ScalarType>((*iter) - scanTimestamp);
  }
}

// template <typename ScalarType>
// void RosPointCloud2Deserializer<ScalarType>::fillIndexGrid(
//     const sensor_msgs::msg::PointCloud2 &rosMsg, const size_t pointCount,
//     DataPoints &cloud) {
//   using ArrayBooleans = Eigen::Array<bool, 1, Eigen::Dynamic>;
//   // Compute the size of the index grid.
//   const Index nbColumns{rosMsg.height};
//   const Index nbRows{rosMsg.width};
//   assert(nbColumns * nbRows == pointCount);


//   // Initialize the index grid.
//   cloud.allocateIndexGrid(nbColumns, nbRows);
//   // Instantiate array of booleans to flag points as valid.
//   // The following line implements a fast version of isNaN(), suggested by
//   // the
//   // author of Eigen: https://forum.kde.org/viewtopic.php?f=74&t=91514
//   const ArrayBooleans isValidPoint{
//       (cloud.features.array() == cloud.features.array()).colwise().all()};

//   // Fill the index grid at each cell with a linear index value.
//   const Index maxLinearIndex{static_cast<Index>(pointCount)};
//   for (Index linearIndex{0}; linearIndex < maxLinearIndex; ++linearIndex) {
//     if (!isValidPoint(linearIndex)) {
//       continue;
//     }

//     // Point the index grid cell to the current element.
//     // Note that here a strong assumption is made, about the storage order of
//     // the index grid being column-major.
//     cloud.indexGrid(linearIndex) = linearIndex;
//   }
// }

template <typename ScalarType>
void RosPointCloud2Deserializer<ScalarType>::fillPointCloudValues(
    const sensor_msgs::msg::PointCloud2 &rosMsg, const bool is3dPointCloud,
    DataPoints &pointCloud) {
  const size_t pointCount{rosMsg.width * rosMsg.height};

  // Point coordinates.
  {
    View view(pointCloud.features.block(0, 0, pointCloud.features.rows(),
                                        pointCloud.features.cols()));
    const FieldNamesList fieldNames{"x", "y", "z"};
    fillVectorDataIntoView(rosMsg, fieldNames, is3dPointCloud, pointCount,
                           view);
    pointCloud.getFeatureViewByName("pad").setOnes();
  }

  // Normals.
  if (pointCloud.descriptorExists("normals")) {
    View view(pointCloud.getDescriptorViewByName("normals"));
    const FieldNamesList fieldNames{"normal_x", "normal_y", "normal_z"};
    fillVectorDataIntoView(rosMsg, fieldNames, is3dPointCloud, pointCount,
                           view);
  }

  // Colors.
  if (pointCloud.descriptorExists("color")) {
    View view(pointCloud.getDescriptorViewByName("color"));
    const FieldNamesList fieldNames{"r", "g", "b", "a"};
    fillColorDataIntoView(rosMsg, fieldNames, pointCount, view);
  }

  // Per-point absolute timestamp stored as double precision (Hesai LiDAR)
  if (pointCloud.descriptorExists("timestamp")) {
    View view(pointCloud.getDescriptorViewByName("timestamp"));
    const std::string fieldName{"timestamp"};
    fillPerPointAbsoluteTimestampDataIntoView(rosMsg, fieldName, pointCount,
                                              view);
  }

  // Per-point absolute timestamp stored as double precision (Hesai LiDAR)
  if (pointCloud.descriptorExists("ring")) {
    View view(pointCloud.getDescriptorViewByName("ring"));
    const std::string fieldName{"ring"};
    fillPerPointRingDataIntoView(rosMsg, fieldName, pointCount, view);
  }

  // Scalar descriptors.
  const FieldNamesList preprocessedFieldLabels{
      "xyz",      "x",        "y",     "z",         "normals", "normal_x",
      "normal_y", "normal_z", "color", "rgb",       "rgba",    "r",
      "g",        "b",        "a",     "timestamp", "ring"};
  for (const auto &field : rosMsg.fields) {
    // Ignore descriptors that we have previously written into our point
    // cloud
    // matrix.
    if (std::find(preprocessedFieldLabels.begin(),
                  preprocessedFieldLabels.end(),
                  field.name) != preprocessedFieldLabels.end()) {
      continue;
    }

    View view{pointCloud.getDescriptorViewByName(field.name)};
    fillScalarDataIntoView(rosMsg, field.name, pointCount, view);
  }

  // Index grid.
  // if (!rosMsg.is_dense && is3dPointCloud) {
  //   fillIndexGrid(rosMsg, pointCount, pointCloud);
  // }
}

// Explicit template instantiations for floating point types.
template class RosPointCloud2Deserializer<float>;
template class RosPointCloud2Deserializer<double>;

} // namespace pointmatcher_ros