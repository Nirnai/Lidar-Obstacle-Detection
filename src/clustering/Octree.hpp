#pragma once

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>

#include <Eigen/Core>
#include <memory>
#include <vector>

// Node Info
class OctreeNodeInfo {
 public:
  OctreeNodeInfo() : origin_(0, 0, 0), size_(0), depth_(0), child_index_(0) {}
  OctreeNodeInfo(const Eigen::Vector3d origin, const double& size,
                 const size_t& depth, const size_t& child_index)
      : origin_(origin),
        size_(size),
        depth_(depth),
        child_index_(child_index) {}

  ~OctreeNodeInfo() {}

 private:
  Eigen::Vector3d origin_;
  double size_;
  size_t depth_;
  size_t child_index_;
};

// Node
class OctreeNode {
 public:
  OctreeNode() {}
  virtual ~OctreeNode() {}
};

// Internal Node
class OctreeInternalNode : public OctreeNode {
 public:
  OctreeInternalNode() : children_(8) {}

 private:
  std::vector<std::shared_ptr<OctreeNode>> children_;
};

// Lead Node
class OctreeLeafNode : public OctreeNode {
 public:
  virtual bool operator==(const OctreeLeafNode& other) const = 0;
  virtual std::shared_ptr<OctreeLeafNode> clone() const = 0;
  static std::function<std::shared_ptr<OctreeLeafNode>()> getInitFunction() {
    return []() -> std::shared_ptr<OctreeLeafNode> {
      return std::make_shared<OctreeLeafNode>();
    };
  }
  static std::function<void(std::shared_ptr<OctreeLeafNode>)>
  getUpdateFunction();
};

// Octree
class Octree {
 public:
  Octree() : origin_(0, 0, 0), size_(0), max_depth_(0) {}
  Octree(size_t max_depth)
      : origin_(0, 0, 0), size_(0), max_depth_(max_depth) {}
  Octree(const size_t& max_depth, const Eigen::Vector3d& origin,
         const double& size)
      : max_depth_(max_depth), origin_(origin), size_(size) {}
  Octree(const Octree& src_octree);
  Octree(const pcl::PointCloud<pcl::PointXYZ>& cloud,
         double size_expand = 0.01) {}

  Octree(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
         double size_expand = 0.01) {
    Eigen::Vector4f min_bound, max_bound;
    pcl::getMinMax3D(*cloud, min_bound, max_bound);
    Eigen::Vector4f center = (min_bound + max_bound) / 2;
  }

  ~Octree() {}

  Octree& clear();
  bool isEmpty() const { return root_node_ == nullptr; }
  Eigen::Vector3d getMinBound() const {
    if (isEmpty()) {
      return Eigen::Vector3d::Zero();
    } else {
      return origin_;
    }
  }

  Eigen::Vector3d getMaxBound() const {
    if (isEmpty()) {
      return Eigen::Vector3d::Zero();
    } else {
      return origin_ + Eigen::Vector3d(size_, size_, size_);
    }
  };

  Eigen::Vector3d getCenter() const {
    return origin_ + Eigen::Vector3d(size_, size_, size_) / 2;
  }

  void insertPoint(
      const Eigen::Vector3d& point,
      const std::function<std::shared_ptr<OctreeLeafNode>()>& f_init,
      const std::function<void(std::shared_ptr<OctreeLeafNode>)>& f_update);

  void traverse(
      const std::function<void(const std::shared_ptr<OctreeNode>&,
                               const std::shared_ptr<OctreeNodeInfo>&)>& f);

 private:
  std::shared_ptr<OctreeNode> root_node_ = nullptr;
  Eigen::Vector3d origin_;
  double size_;
  size_t max_depth_;
};
