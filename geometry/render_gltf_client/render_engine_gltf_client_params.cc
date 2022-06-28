#include "drake/geometry/render_gltf_client/render_engine_gltf_client_params.h"

#include <string>

namespace drake {
namespace geometry {

std::string RenderEngineGltfClientParams::GetUrl() const {
  std::string url = base_url;
  std::string endpoint = render_endpoint;

  const std::string throw_message =
      "RenderEngineGltfClientParams: invalid base_url or render_endpoint is "
      "provided that contains only `/`.";

  auto all_slashes = [](const std::string& str) {
    if (str.empty()) return false;
    return std::all_of(str.begin(), str.end(), [](char c){ return c == '/'; });
  };

  if (!all_slashes(url)) {
    while (url.size() > 0 && url.back() == '/') url.pop_back();
  } else {
    throw std::logic_error(throw_message);
  }

  if (endpoint == "/") {
    // Return the full URL as ``url`/` in this case.
    endpoint = "";
  } else if (!all_slashes(endpoint)) {
    while (endpoint.size() > 0 && endpoint.front() == '/') endpoint.erase(0, 1);
  } else {
    throw std::logic_error(throw_message);
  }

  return url + "/" + endpoint;
}

}  // namespace geometry
}  // namespace drake
