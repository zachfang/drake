#include "drake/geometry/render_gltf_client/internal_http_service.h"

#include <vector>

#include <fmt/format.h>

#include "drake/common/filesystem.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

using DataMap = std::map<std::string, std::string>;
using FileMap =
    std::map<std::string, std::pair<std::string, std::optional<std::string>>>;

HttpResponse HttpService::PostForm(
    const std::string& temp_directory, const std::string& url, int port,
    const std::map<std::string, std::string>& data_fields,
    const std::map<std::string,
                   std::pair<std::string, std::optional<std::string>>>&
        file_fields,
    bool verbose) {
  std::vector<std::string> missing_files;
  for (const auto& [field_name, field_data_pair] : file_fields) {
    const auto& file_path = field_data_pair.first;
    if (!drake::filesystem::is_regular_file(file_path)) {
      missing_files.emplace_back(fmt::format("{}='{}'", field_name, file_path));
    }
  }

  if (missing_files.size() > 0) {
    throw std::runtime_error(
        fmt::format("Provided file fields had missing file(s): {}.",
                    fmt::join(missing_files, ", ")));
  }

  return DoPostForm(temp_directory, url, port, data_fields, file_fields,
                    verbose);
}

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
