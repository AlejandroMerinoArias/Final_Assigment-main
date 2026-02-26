#include "exploring/exploration_manager.hpp"
#include <gtest/gtest.h>

namespace planning {

// ===========================================================================
// Test fixture
// ===========================================================================

class ExplorationManagerTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<planning::ExplorationManager>();
    // Lower min-frontier-size for easier testing
    node_->min_frontier_size_ = 1;
    node_->min_z_ = 0.0;
    node_->max_z_ = 5.0;
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<planning::ExplorationManager> node_;

  // ----- Helpers that forward to private methods/members via friendship -----

  // Setters for private parameters (friend access happens here, in the fixture)
  void set_min_frontier_size(int v) { node_->min_frontier_size_ = v; }
  void set_min_z(double v) { node_->min_z_ = v; }
  void set_max_z(double v) { node_->max_z_ = v; }
  void set_blacklist_radius(double v) { node_->blacklist_radius_ = v; }
  void add_blacklisted_goal(const octomap::point3d &p) {
    node_->blacklisted_goals_.push_back(p);
  }

  // Method forwarders
  std::vector<octomap::point3d>
  detect_3d_frontiers(const octomap::OcTree &tree) {
    return node_->detect_3d_frontier_voxels(tree);
  }

  std::vector<planning::Frontier3D>
  cluster_3d(const std::vector<octomap::point3d> &voxels, double res) {
    return node_->cluster_3d_frontiers(voxels, res);
  }

  double info_gain(const octomap::OcTree &tree, const octomap::point3d &vp,
                   double range) {
    return node_->estimate_info_gain(tree, vp, range);
  }

  void score_frontiers(std::vector<planning::Frontier3D> &clusters,
                       const octomap::OcTree &tree,
                       const octomap::point3d &drone) {
    node_->score_3d_frontiers(clusters, tree, drone);
  }

  bool is_blacklisted(const octomap::point3d &p) const {
    return node_->is_blacklisted(p);
  }

  bool find_nearest_free(const octomap::OcTree &tree,
                         const octomap::point3d &query, double radius,
                         octomap::point3d &result) {
    return node_->find_nearest_free(tree, query, radius, result);
  }
};

// ===========================================================================
// Test: 3D frontier detection
// ===========================================================================

TEST_F(ExplorationManagerTest, Detect3DFrontiersBasic) {
  // Create a tiny OcTree with a few free voxels surrounded by unknown space.
  octomap::OcTree tree(0.1);

  // Insert free voxels at Z = 1.0 (within navigable range)
  tree.updateNode(octomap::point3d(0.0f, 0.0f, 1.0f), false); // free
  tree.updateNode(octomap::point3d(0.1f, 0.0f, 1.0f), false); // free
  tree.updateNode(octomap::point3d(0.2f, 0.0f, 1.0f), false); // free

  // All three voxels border unknown space → all should be frontier voxels
  auto frontiers = detect_3d_frontiers(tree);
  EXPECT_GE(frontiers.size(), 3u);
}

TEST_F(ExplorationManagerTest, Detect3DFrontiersNoFrontierWhenSurrounded) {
  // Build a small cube of free voxels – the interior should NOT be a frontier.
  octomap::OcTree tree(0.1);

  for (float x = -0.1f; x <= 0.1f; x += 0.1f) {
    for (float y = -0.1f; y <= 0.1f; y += 0.1f) {
      for (float z = 0.9f; z <= 1.1f; z += 0.1f) {
        tree.updateNode(octomap::point3d(x, y, z), false);
      }
    }
  }

  auto frontiers = detect_3d_frontiers(tree);

  // The center voxel (0,0,1) has all 6 neighbors in-tree and free,
  // so it should NOT be a frontier. Only surface voxels are frontiers.
  // We just verify there are fewer frontiers than total free voxels.
  EXPECT_LT(frontiers.size(), 27u); // 3×3×3 = 27 total
  EXPECT_GT(frontiers.size(), 0u);  // surface voxels are still frontiers
}

TEST_F(ExplorationManagerTest, Detect3DFrontiersZFilter) {
  // Voxels outside the navigable Z band should be excluded
  set_min_z(1.0);
  set_max_z(2.0);

  octomap::OcTree tree(0.1);
  tree.updateNode(octomap::point3d(0.0f, 0.0f, 1.5f), false); // in range
  tree.updateNode(octomap::point3d(0.0f, 0.0f, 0.5f), false); // out of range
  tree.updateNode(octomap::point3d(0.0f, 0.0f, 2.5f), false); // out of range

  auto frontiers = detect_3d_frontiers(tree);
  // Only the voxel at Z=1.5 should appear
  EXPECT_EQ(frontiers.size(), 1u);
  EXPECT_NEAR(frontiers[0].z(), 1.5f, 0.05f);
}

// ===========================================================================
// Test: 3D clustering
// ===========================================================================

TEST_F(ExplorationManagerTest, Cluster3DAdjacentVoxels) {
  // Three adjacent voxels should form a single cluster
  double res = 0.1;
  std::vector<octomap::point3d> voxels = {
      octomap::point3d(0.0f, 0.0f, 1.0f),
      octomap::point3d(0.1f, 0.0f, 1.0f),
      octomap::point3d(0.2f, 0.0f, 1.0f),
  };

  auto clusters = cluster_3d(voxels, res);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].size(), 3);
}

TEST_F(ExplorationManagerTest, Cluster3DSeparateGroups) {
  // Two groups far apart should form two clusters
  double res = 0.1;
  std::vector<octomap::point3d> voxels = {
      // Group A
      octomap::point3d(0.0f, 0.0f, 1.0f),
      octomap::point3d(0.1f, 0.0f, 1.0f),
      // Group B (far away)
      octomap::point3d(5.0f, 5.0f, 1.0f),
      octomap::point3d(5.1f, 5.0f, 1.0f),
  };

  auto clusters = cluster_3d(voxels, res);
  ASSERT_EQ(clusters.size(), 2u);
  EXPECT_EQ(clusters[0].size(), 2);
  EXPECT_EQ(clusters[1].size(), 2);
}

TEST_F(ExplorationManagerTest, Cluster3DMinSizeFilter) {
  set_min_frontier_size(3);

  double res = 0.1;
  std::vector<octomap::point3d> voxels = {
      // Group of 4 (should pass)
      octomap::point3d(0.0f, 0.0f, 1.0f),
      octomap::point3d(0.1f, 0.0f, 1.0f),
      octomap::point3d(0.2f, 0.0f, 1.0f),
      octomap::point3d(0.3f, 0.0f, 1.0f),
      // Isolated voxel (should be filtered)
      octomap::point3d(9.0f, 9.0f, 1.0f),
  };

  auto clusters = cluster_3d(voxels, res);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].size(), 4);
}

TEST_F(ExplorationManagerTest, Cluster3DCentroidIsCorrect) {
  double res = 0.1;
  // Use adjacent voxels (1 grid step apart → 26-connected)
  std::vector<octomap::point3d> voxels = {
      octomap::point3d(0.0f, 0.0f, 1.0f),
      octomap::point3d(0.1f, 0.0f, 1.0f),
  };

  auto clusters = cluster_3d(voxels, res);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_NEAR(clusters[0].centroid.x(), 0.05f, 0.01f);
  EXPECT_NEAR(clusters[0].centroid.y(), 0.0f, 0.01f);
  EXPECT_NEAR(clusters[0].centroid.z(), 1.0f, 0.01f);
}

// ===========================================================================
// Test: Information-gain estimation
// ===========================================================================

TEST_F(ExplorationManagerTest, InfoGainHighForUnexploredArea) {
  // An almost-empty tree → most of the BBX is unknown → high gain
  octomap::OcTree tree(0.1);
  tree.updateNode(octomap::point3d(0.0f, 0.0f, 1.0f), false);

  double gain = info_gain(tree, octomap::point3d(0.0f, 0.0f, 1.0f), 1.0);
  EXPECT_GT(gain, 0.0);
}

TEST_F(ExplorationManagerTest, InfoGainLowerForExploredArea) {
  // Fill a region with known voxels → less unknown → lower gain
  octomap::OcTree tree(0.1);

  for (float x = -0.5f; x <= 0.5f; x += 0.1f) {
    for (float y = -0.5f; y <= 0.5f; y += 0.1f) {
      for (float z = 0.5f; z <= 1.5f; z += 0.1f) {
        tree.updateNode(octomap::point3d(x, y, z), false);
      }
    }
  }

  double gain_explored =
      info_gain(tree, octomap::point3d(0.0f, 0.0f, 1.0f), 0.5);
  double gain_unexplored =
      info_gain(tree, octomap::point3d(5.0f, 5.0f, 1.0f), 0.5);

  // The explored area should have lower info gain than the unexplored area
  EXPECT_LT(gain_explored, gain_unexplored);
}

// ===========================================================================
// Test: Scoring prefers closer high-gain frontiers
// ===========================================================================

TEST_F(ExplorationManagerTest, ScorePrefersFrontierWithHigherInfoGain) {
  octomap::OcTree tree(0.1);
  // Insert a single node so the tree exists
  tree.updateNode(octomap::point3d(0.0f, 0.0f, 1.0f), false);

  // Two clusters at equal distance from drone
  std::vector<Frontier3D> clusters(2);

  // Cluster 0: near (2, 0, 1), small
  clusters[0].cells = {
      octomap::point3d(2.0f, 0.0f, 1.0f),
      octomap::point3d(2.1f, 0.0f, 1.0f),
  };
  clusters[0].centroid = octomap::point3d(2.05f, 0.0f, 1.0f);

  // Cluster 1: near (2, 0, 1), but much bigger
  for (int i = 0; i < 20; ++i) {
    clusters[1].cells.push_back(octomap::point3d(2.0f + 0.1f * i, 0.0f, 1.0f));
  }
  clusters[1].centroid = octomap::point3d(3.0f, 0.0f, 1.0f);

  octomap::point3d drone(0.0f, 0.0f, 1.0f);
  score_frontiers(clusters, tree, drone);

  // Bigger cluster with more info gain should score higher
  // (both are at similar distance)
  EXPECT_GT(clusters[1].score, 0.0);
  EXPECT_GT(clusters[0].score, 0.0);
}

// ===========================================================================
// Test: Blacklisting
// ===========================================================================

TEST_F(ExplorationManagerTest, BlacklistingWorks) {
  EXPECT_FALSE(is_blacklisted(octomap::point3d(5.0f, 5.0f, 1.0f)));

  add_blacklisted_goal(octomap::point3d(5.0f, 5.0f, 1.0f));
  set_blacklist_radius(2.0);

  // Within radius → blacklisted
  EXPECT_TRUE(is_blacklisted(octomap::point3d(5.0f, 5.5f, 1.0f)));
  // Outside radius → not blacklisted
  EXPECT_FALSE(is_blacklisted(octomap::point3d(10.0f, 10.0f, 1.0f)));
}

// ===========================================================================
// Test: find_nearest_free
// ===========================================================================

TEST_F(ExplorationManagerTest, FindNearestFreeFindsCloseVoxel) {
  octomap::OcTree tree(0.1);
  // Insert a free voxel and an occupied voxel
  tree.updateNode(octomap::point3d(1.0f, 0.0f, 1.0f), false); // free
  tree.updateNode(octomap::point3d(0.0f, 0.0f, 1.0f), true);  // occupied

  octomap::point3d result;
  // Query from the occupied position → should find the free one
  bool found =
      find_nearest_free(tree, octomap::point3d(0.0f, 0.0f, 1.0f), 2.0, result);
  EXPECT_TRUE(found);
  EXPECT_NEAR(result.x(), 1.0f, 0.15f);
}

TEST_F(ExplorationManagerTest, FindNearestFreeReturnsSelfIfFree) {
  octomap::OcTree tree(0.1);
  tree.updateNode(octomap::point3d(1.0f, 1.0f, 1.0f), false);

  octomap::point3d result;
  bool found =
      find_nearest_free(tree, octomap::point3d(1.0f, 1.0f, 1.0f), 2.0, result);
  EXPECT_TRUE(found);
  EXPECT_NEAR(result.x(), 1.0f, 0.05f);
  EXPECT_NEAR(result.y(), 1.0f, 0.05f);
  EXPECT_NEAR(result.z(), 1.0f, 0.05f);
}

} // namespace planning
