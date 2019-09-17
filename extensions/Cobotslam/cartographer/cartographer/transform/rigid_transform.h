/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_H_
#define CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_H_

#include <cmath>
#include <iostream>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"

namespace cartographer {
namespace transform {
//2D变换，模板类型不能为int型
template <typename FloatType>
class Rigid2 {
 public:
  using Vector = Eigen::Matrix<FloatType, 2, 1>;
  using Rotation2D = Eigen::Rotation2D<FloatType>;
//默认构造函数，平移为0向量，旋转为单位矩阵
  Rigid2() : translation_(Vector::Zero()), rotation_(Rotation2D::Identity()) {}
//带参数构造函数，旋转量可为yaw或旋转矩阵。
  Rigid2(const Vector& translation, const Rotation2D& rotation)
      : translation_(translation), rotation_(rotation) {}
  Rigid2(const Vector& translation, const double rotation)
      : translation_(translation), rotation_(rotation) {}
//类的静态成员函数，返回Rigid2
  static Rigid2 Rotation(const double rotation) {
    return Rigid2(Vector::Zero(), rotation);
  }

  static Rigid2 Rotation(const Rotation2D& rotation) {
    return Rigid2(Vector::Zero(), rotation);
  }

  static Rigid2 Translation(const Vector& vector) {
    return Rigid2(vector, Rotation2D::Identity());
  }

  static Rigid2<FloatType> Identity() { return Rigid2<FloatType>(); }
//类型转换
//cast()按照指定的参数类型将数据成员进行类型转换：
//Rigid2数据成员原本是double,对象rigid2转换成float可调用：
//rigid2.cast<float>()
  template <typename OtherType>
  Rigid2<OtherType> cast() const {
    return Rigid2<OtherType>(translation_.template cast<OtherType>(),
                             rotation_.template cast<OtherType>());
  }
//返回平移向量
  const Vector& translation() const { return translation_; }
//返回旋转矩阵:
  Rotation2D rotation() const { return rotation_; }

  double normalized_angle() const {
    return common::NormalizeAngleDifference(rotation().angle());
  }
//对T求逆，原理见PPT
  Rigid2 inverse() const {
    const Rotation2D rotation = rotation_.inverse();
    const Vector translation = -(rotation * translation_);
    return Rigid2(translation, rotation);
  }

  std::string DebugString() const {
    std::string out;
    out.append("{ t: [");
    out.append(std::to_string(translation().x()));
    out.append(", ");
    out.append(std::to_string(translation().y()));
    out.append("], r: [");
    out.append(std::to_string(rotation().angle()));
    out.append("] }");
    return out;
  }

 private:
//2行1列 的矩阵.平移向量[dx,dy]
  Vector translation_;
//旋转矩阵
  Rotation2D rotation_;
};


/*
Rotation2D介绍：
Rotation2D 是二维旋转里根据逆时针旋转θ角而生成的旋转矩阵:
[cosθ , -sinθ     
 sinθ ,  cosθ ]

  */


/*
定义 * 乘法操作符:2个Rigid2相乘,得到第三个Rigid2,
等效于连续变换2次
实现细节：
1，最终的[dx'',dy'']等于lhs在rhs方向上的投影[dx',dy']加上rhs自身的[dx,dy]
2，角度相加(等效于旋转矩阵相乘)

------公式推导--------

P'=R1*P+t1;

P''=R2*P'+t2,即

P''=R2(R1*P+t1)+t2=R2*R1*P+R2*t1+t2.

--------------------

*/


//重载T*T的乘法，即两次刚体变换
template <typename FloatType>
Rigid2<FloatType> operator*(const Rigid2<FloatType>& lhs,
                            const Rigid2<FloatType>& rhs) {
  return Rigid2<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      lhs.rotation() * rhs.rotation());
}

//重载T×2d点，即对2d点做旋转和平移，返回2d点
template <typename FloatType>
typename Rigid2<FloatType>::Vector operator*(
    const Rigid2<FloatType>& rigid,
    const typename Rigid2<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
}
//重载<<，debug
// This is needed for gmock.
template <typename T>
std::ostream& operator<<(std::ostream& os,
                         const cartographer::transform::Rigid2<T>& rigid) {
  os << rigid.DebugString();
  return os;
}
//模板特化
using Rigid2d = Rigid2<double>;
using Rigid2f = Rigid2<float>;


/*
Rigid3是三维刚性变换.SE3
该类含义2个数据成员：

Vector translation_;
Quaternion rotation_;

1),3个构造函数分别初始化平移矩阵和旋转矩阵。

2),3个静态成员函数分别按照给定方式旋转.
   Rotation(),Translation(),Identity()
构造函数应为单位四元数

*/
template <typename FloatType>
class Rigid3 {
 public:
//3行1列的平移向量
  using Vector = Eigen::Matrix<FloatType, 3, 1>;
//四元数，原理见PPT
  using Quaternion = Eigen::Quaternion<FloatType>;
//角轴，原理见PPT
  using AngleAxis = Eigen::AngleAxis<FloatType>;
//默认构造函数，平移为0向量，旋转为单位阵
  Rigid3() : translation_(Vector::Zero()), rotation_(Quaternion::Identity()) {}
//构造函数，提供四元数初始化旋转
  Rigid3(const Vector& translation, const Quaternion& rotation)
      : translation_(translation), rotation_(rotation) {}
//构造函数，提供角轴初始化旋转
  Rigid3(const Vector& translation, const AngleAxis& rotation)
      : translation_(translation), rotation_(rotation) {}
//静态成员函数, 只旋转，不平移。角轴
  static Rigid3 Rotation(const AngleAxis& angle_axis) {
    return Rigid3(Vector::Zero(), Quaternion(angle_axis));
  }
//静态成员函数, 只旋转，不平移。四元数
  static Rigid3 Rotation(const Quaternion& rotation) {
    return Rigid3(Vector::Zero(), rotation);
  }
//静态成员函数, 只平移。四元数
  static Rigid3 Translation(const Vector& vector) {
    return Rigid3(vector, Quaternion::Identity());
  }
//静态成员函数，数组参数形式构造T
  static Rigid3 FromArrays(const std::array<FloatType, 4>& rotation,
                           const std::array<FloatType, 3>& translation) {
    return Rigid3(Eigen::Map<const Vector>(translation.data()),
                  Eigen::Quaternion<FloatType>(rotation[0], rotation[1],
                                               rotation[2], rotation[3]));
  }
//T的单位阵
  static Rigid3<FloatType> Identity() { return Rigid3<FloatType>(); }

  template <typename OtherType>
  Rigid3<OtherType> cast() const {
    return Rigid3<OtherType>(translation_.template cast<OtherType>(),
                             rotation_.template cast<OtherType>());
  }

  const Vector& translation() const { return translation_; }
  const Quaternion& rotation() const { return rotation_; }
//T的逆矩阵
  Rigid3 inverse() const {
//四元数求逆直接求共轭
    const Quaternion rotation = rotation_.conjugate();
    const Vector translation = -(rotation * translation_);
    return Rigid3(translation, rotation);
  }

  std::string DebugString() const {
    std::string out;
    out.append("{ t: [");
    out.append(std::to_string(translation().x()));
    out.append(", ");
    out.append(std::to_string(translation().y()));
    out.append(", ");
    out.append(std::to_string(translation().z()));
    out.append("], q: [");
    out.append(std::to_string(rotation().w()));
    out.append(", ");
    out.append(std::to_string(rotation().x()));
    out.append(", ");
    out.append(std::to_string(rotation().y()));
    out.append(", ");
    out.append(std::to_string(rotation().z()));
    out.append("] }");
    return out;
  }
//验证T是否合法
  bool IsValid() const {
    return !std::isnan(translation_.x()) && !std::isnan(translation_.y()) &&
           !std::isnan(translation_.z()) &&
           std::abs(FloatType(1) - rotation_.norm()) < FloatType(1e-3);
//四元数是否归一化，没有归一化R不正交
  }

 private:
  Vector translation_;
  Quaternion rotation_;
};

template <typename FloatType>
Rigid3<FloatType> operator*(const Rigid3<FloatType>& lhs,
                            const Rigid3<FloatType>& rhs) {
  return Rigid3<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      (lhs.rotation() * rhs.rotation()).normalized());
//四元数要归一化
}

template <typename FloatType>
typename Rigid3<FloatType>::Vector operator*(
    const Rigid3<FloatType>& rigid,
    const typename Rigid3<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
}

// This is needed for gmock.
template <typename T>
std::ostream& operator<<(std::ostream& os,
                         const cartographer::transform::Rigid3<T>& rigid) {
  os << rigid.DebugString();
  return os;
}

using Rigid3d = Rigid3<double>;
using Rigid3f = Rigid3<float>;

// Converts (roll, pitch, yaw) to a unit length quaternion. Based on the URDF
// specification http://wiki.ros.org/urdf/XML/joint.
//从欧拉角转四元数
Eigen::Quaterniond RollPitchYaw(double roll, double pitch, double yaw);

// Returns an transform::Rigid3d given a 'dictionary' containing 'translation'
// (x, y, z) and 'rotation' which can either we an array of (roll, pitch, yaw)
// or a dictionary with (w, x, y, z) values as a quaternion.
Rigid3d FromDictionary(common::LuaParameterDictionary* dictionary);

}  // namespace transform
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_H_
