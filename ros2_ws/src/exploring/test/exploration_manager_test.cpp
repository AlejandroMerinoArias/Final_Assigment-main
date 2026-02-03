#include <gtest/gtest.h>
#include "exploring/exploration_manager.hpp"

namespace planning
{

// Test fixture
class ExplorationManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<planning::ExplorationManager>();
    // Set minimal limit to 1 for easier testing
    node_->min_frontier_size_ = 1;
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<planning::ExplorationManager> node_;

  // Helpers to access private methods via friendship
  std::vector<std::pair<int, int>> detect_frontiers(
    const nav_msgs::msg::OccupancyGrid & map)
  {
    return node_->detect_frontier_cells(map);
  }

  std::vector<planning::FrontierCluster> cluster_frontiers(
    const std::vector<std::pair<int, int>> & cells,
    const nav_msgs::msg::OccupancyGrid & map)
  {
    return node_->cluster_frontiers(cells, map);
  }

  void calculate_centroid(
    planning::FrontierCluster & cluster,
    const nav_msgs::msg::OccupancyGrid & map)
  {
    node_->calculate_centroid(cluster, map);
  }

  void score_frontiers(
    std::vector<planning::FrontierCluster> & clusters,
    double drone_x, double drone_y)
  {
    node_->score_frontiers(clusters, drone_x, drone_y);
  }
};

TEST_F(ExplorationManagerTest, DetectFrontierSimple)
{
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = 3;
  map.info.height = 3;
  map.info.resolution = 1.0;
  map.data.resize(9, 0); // All free

  //  0  0  0
  //  0  0 -1  <-- Center (1,1) is free, Right (1,2) is unknown
  //  0  0  0
  map.data[4] = 0;   // Center free
  map.data[5] = -1;  // Right unknown

  auto frontiers = detect_frontiers(map);

  // Center (1,1) has neighbor (1,2) which is -1.
  // So (1,1) should be a frontier.
  ASSERT_EQ(frontiers.size(), 1u);
  EXPECT_EQ(frontiers[0].first, 1); // row
  EXPECT_EQ(frontiers[0].second, 1); // col
}

TEST_F(ExplorationManagerTest, ClusterFrontiers)
{
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = 5;
  map.info.height = 5;

  // 3 frontier cells in a line
  // (1,1), (1,2), (1,3)
  std::vector<std::pair<int, int>> cells;
  cells.emplace_back(1, 1);
  cells.emplace_back(1, 2);
  cells.emplace_back(1, 3);

  // Isolated cell (3,3)
  cells.emplace_back(3, 3);

  auto clusters = cluster_frontiers(cells, map);

  // Should have 2 clusters (line of 3, and isolated 1)
  ASSERT_EQ(clusters.size(), 2u);

  // Verify sizes (order depends on implementation, likely order of input)
  // But since (1,1) is first in list, BFS should find (1,1)-(1,3) first.
  EXPECT_EQ(clusters[0].cells.size(), 3u);
  EXPECT_EQ(clusters[1].cells.size(), 1u);
}

TEST_F(ExplorationManagerTest, CentroidCalculation)
{
  nav_msgs::msg::OccupancyGrid map;
  map.info.resolution = 1.0;
  map.info.origin.position.x = 0.0;
  map.info.origin.position.y = 0.0;

  planning::FrontierCluster cluster;
  // Two cells: (0,0) and (0,1)
  // World: (0.5, 0.5) and (1.5, 0.5)
  cluster.cells.emplace_back(0, 0);
  cluster.cells.emplace_back(0, 1);

  calculate_centroid(cluster, map);

  EXPECT_DOUBLE_EQ(cluster.centroid.x, 1.0); // Midpoint of 0.5 and 1.5
  EXPECT_DOUBLE_EQ(cluster.centroid.y, 0.5);
}

TEST_F(ExplorationManagerTest, ScoreFrontiersGreedy)
{
  std::vector<planning::FrontierCluster> clusters(2);

  // Far cluster
  clusters[0].centroid.x = 10.0;
  clusters[0].centroid.y = 0.0;

  // Near cluster
  clusters[1].centroid.x = 2.0;
  clusters[1].centroid.y = 0.0;

  double drone_x = 0.0;
  double drone_y = 0.0;

  score_frontiers(clusters, drone_x, drone_y);

  // Near cluster should have higher score (1/2 vs 1/10)
  EXPECT_GT(clusters[1].score, clusters[0].score);
  EXPECT_DOUBLE_EQ(clusters[1].score, 0.5);
  EXPECT_DOUBLE_EQ(clusters[0].score, 0.1);
}

} // namespace planning
