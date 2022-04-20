#include "drake/geometry/render_gltf_client/internal_http_service.h"

#include <fstream>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/filesystem.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {
namespace {

namespace fs = drake::filesystem;
using data_map_t = std::map<std::string, std::string>;
using file_name_path_t = std::pair<std::string, std::optional<std::string>>;
using file_map_t = std::map<std::string, file_name_path_t>;

// A concrete implementation of HttpService that does nothing.
class EmptyService : public HttpService {
 public:
  EmptyService() : HttpService() {}

 protected:
  HttpResponse DoPostForm(
      const std::string& /* temp_directory */, const std::string& url,
      int /* port */,
      const std::map<std::string, std::string>& /* data_fields */,
      const std::map<std::string,
                     std::pair<std::string, std::optional<std::string>>>&
          file_fields,
      bool /* verbose */ = false) override {
    HttpResponse ret;
    ret.http_code = 200;
    return ret;
  }
};

GTEST_TEST(HttpService, ThrowIfFilesMissing) {
  // Create an EmptyService and some files to test with.
  EmptyService es;
  const auto temp_dir = drake::temp_directory();
  const std::string url{"127.0.0.1/render"};
  const int port{8000};
  const data_map_t empty_data_fields;

  const auto test_txt_path = (fs::path(temp_dir) / "test.txt").string();
  std::ofstream test_txt{test_txt_path};
  test_txt << "test!\n";
  test_txt.close();

  const auto fake_jpg_path = (fs::path(temp_dir) / "fake.jpg").string();
  std::ofstream fake_jpg{fake_jpg_path};
  fake_jpg << "not really a jpg!\n";
  fake_jpg.close();

  {
    file_map_t file_fields;
    file_fields.insert(std::pair<std::string, file_name_path_t>(
        {"test", {test_txt_path, std::nullopt}}));
    file_fields.insert(std::pair<std::string, file_name_path_t>(
        {"image", {fake_jpg_path, "image/jpeg"}}));
    // No exception should be thrown if all files are present.
    DRAKE_EXPECT_NO_THROW(
        es.PostForm(temp_dir, url, port, empty_data_fields, file_fields));
  }

  // Some file paths that do not exist for testing.
  const std::string missing_1_key = "missing_1";
  const std::pair<std::string, std::optional<std::string>> missing_1_value = {
      "/unlikely/to/exist.file_extension", std::nullopt};
  const std::string missing_1_desc{
      "missing_1='/unlikely/to/exist.file_extension'"};
  ASSERT_FALSE(fs::is_regular_file(missing_1_value.first));

  const std::string missing_2_key = "missing_2";
  const std::pair<std::string, std::optional<std::string>> missing_2_value = {
      "/this/is/not/a.real_file", std::nullopt};
  const std::string missing_2_desc{"missing_2='/this/is/not/a.real_file'"};
  ASSERT_FALSE(fs::is_regular_file(missing_2_value.first));

  // The exception message prefix.
  const std::string prefix = "Provided file fields had missing file\\(s\\): ";
  // Since it is a map, order can change -- only support building regex for 2.
  auto make_regex = [&prefix](const std::string& p1, const std::string& p2) {
    return fmt::format("{0}(({1}, {2})|({2}, {1}))\\.", prefix, p1, p2);
  };

  {
    file_map_t file_fields;
    file_fields.insert(std::pair<std::string, file_name_path_t>(
        {missing_1_key, missing_1_value}));
    // Exception thrown: one file, does not exist.
    DRAKE_EXPECT_THROWS_MESSAGE(
        es.PostForm(temp_dir, url, port, empty_data_fields, file_fields),
        prefix + missing_1_desc + "\\.");
  }

  {
    file_map_t file_fields;
    file_fields.insert(std::pair<std::string, file_name_path_t>(
        {missing_1_key, missing_1_value}));
    file_fields.insert(std::pair<std::string, file_name_path_t>(
        {missing_2_key, missing_2_value}));
    // Exception thrown: multiple files, none exist.
    DRAKE_EXPECT_THROWS_MESSAGE(
        es.PostForm(temp_dir, url, port, empty_data_fields, file_fields),
        make_regex(missing_1_desc, missing_2_desc));
  }

  {
    file_map_t file_fields;
    file_fields.insert(std::pair<std::string, file_name_path_t>(
        {"test", {test_txt_path, std::nullopt}}));
    file_fields.insert(std::pair<std::string, file_name_path_t>(
        {missing_1_key, missing_1_value}));
    // Exception thrown: one file exists, the other does not.
    DRAKE_EXPECT_THROWS_MESSAGE(
        es.PostForm(temp_dir, url, port, empty_data_fields, file_fields),
        prefix + missing_1_desc + "\\.");
  }

  {
    file_map_t file_fields;
    file_fields.insert(std::pair<std::string, file_name_path_t>(
        {"test", {test_txt_path, std::nullopt}}));
    file_fields.insert(std::pair<std::string, file_name_path_t>(
        {missing_1_key, missing_1_value}));
    file_fields.insert(std::pair<std::string, file_name_path_t>(
        {"image", {fake_jpg_path, "image/jpeg"}}));
    file_fields.insert(std::pair<std::string, file_name_path_t>(
        {missing_2_key, missing_2_value}));
    // Exception thrown: multiple files exist, multiple do not.
    DRAKE_EXPECT_THROWS_MESSAGE(
        es.PostForm(temp_dir, url, port, empty_data_fields, file_fields),
        make_regex(missing_1_desc, missing_2_desc));
  }

  // 3 deletions: 2 files + 1 folder.
  EXPECT_EQ(fs::remove_all(temp_dir), 3);
}

}  // namespace
}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
