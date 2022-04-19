#include "panoptic_mapping/map_management/plane_clustering.h"

#include <cstdlib>
#include <fstream>
#include <memory>
#include <queue>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <experimental/filesystem>
#include <gtest/gtest.h>

#include "panoptic_mapping/map/submap_collection.h"

namespace panoptic_mapping {

namespace test {

PlaneCollection::Config buidConfig(int verbosity = 4, float z_threshold = 0.15,
                                   float xy_threshold = 0.01, int max_walls = 3,
                                   int max_floors = 1, int max_ceilings = 1,
                                   float normal_cluster_threshold = 0.1,
                                   float position_cluster_threshold = 0.05,
                                   float max_outlier_percentage = 0.70,
                                   float satisfying_outlier_percent = 0.70,
                                   int ransac_num_iterations = 1000,
                                   uint_fast32_t random_generator_seed = 100) {
  PlaneCollection::Config cfg;
  cfg.verbosity = verbosity;
  cfg.z_threshold = z_threshold;
  cfg.xy_threshold = xy_threshold;
  cfg.max_walls = max_walls;
  cfg.max_floors = max_floors;
  cfg.max_ceilings = max_ceilings;
  cfg.normal_cluster_threshold = normal_cluster_threshold;
  cfg.position_cluster_threshold = position_cluster_threshold;
  cfg.max_outlier_percentage = max_outlier_percentage;
  cfg.satisfying_outlier_percent = satisfying_outlier_percent;
  cfg.ransac_num_iterations = ransac_num_iterations;
  cfg.random_generator_seed = random_generator_seed;
  cfg.publish_bboxes_topic = "";
  cfg.publish_normals_topic = "";
  return cfg;
}

std::shared_ptr<SubmapCollection> loadMap(const std::string& map_filename) {
  std::shared_ptr<SubmapCollection> submaps =
      std::make_shared<SubmapCollection>();
  bool loaded = submaps->loadFromFile(map_filename);
  CHECK(loaded);
  return std::move(submaps);
}

const std::string home_path = std::getenv("HOME");
std::shared_ptr<SubmapCollection> all_submaps =
    loadMap(home_path + "datasets/test/testing_submaps.panmap");

}  // namespace test
}  // namespace panoptic_mapping

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
