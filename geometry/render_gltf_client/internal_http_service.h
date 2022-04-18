#pragma once

#include <fstream>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/filesystem.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

/* @name Server Parameter Validation Helpers */
//@{
/* Throws an std::exception if the provided url is empty or has trailing
  slashes. */
static void ThrowIfUrlInvalid(const std::string& url) {
  // Validate what can be validated about the provided url.
  if (url.empty()) {
    throw std::logic_error("HttpService: url parameter may not be empty.");
  }
  if (url.back() == '/') {
    throw std::logic_error("HttpService: url may not end with '/'.");
  }
}

/* Throws an std::exception if `endpoint` starts or ends with a '/'. */
static void ThrowIfEndpointInvalid(const std::string& endpoint) {
  if (endpoint.size() >= 1) {
    if (endpoint.front() == '/' || endpoint.back() == '/') {
      throw std::runtime_error(fmt::format(
          "Provided endpoint='{}' is not valid, it may not start or end with "
          "a '/'.",
          endpoint));
    }
  }
}

static void ThrowIfFilesMissing(
    const std::map<std::string,
                   std::pair<std::string, std::optional<std::string>>>&
        file_fields) {
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
}
//@}

/* A simple wrapper struct to encapsulate an HTTP server response. */
struct HttpResponse {
  /* The HTTP response code from the server.  Note that in the special case of
   an HttpService not being able to connect to a server (e.g., the wrong url
   was provided or the server is not running), this field will often have a
   value of `0`.  Use HttpResponse::Good() to validate server responses. */
  int http_code{0};

  /* In the event that the server has provided a text or binary response in
   addition to its HTTP response code, the file path described by this attribute
   will contain the response.  An HttpService will populate this field with a
   non-null value if and only if a response from the server was provided.
   @note
     This is **not** a response loaded into a string, it is a path to a
     non-empty file that contains the response.  The **caller** is responsible
     for loading this file as the appropriate file type, **and** for deleting
     the file when it is no longer needed. */
  std::optional<std::string> data_path{std::nullopt};

  /* If the underlying HttpService encountered an error, this will be set to
   true and any additional information provided in
   HttpResponse::service_error_message. */
  bool service_error{false};

  /* In the event that there was an error with the HttpService processing the
   request, HttpResponse::service_error will be set to `true` and any additional
   information that can be provided will be in this string. */
  std::optional<std::string> service_error_message{std::nullopt};

  /* Whether or not the HTTP transaction was successful.  This includes
   verifying that the HttpResponse::http_code indicates success, but also
   whether or not the underlying HttpService marked it for failure via
   HttpResponse::service_error.  Note that if a server has not been
   properly implemented, it may always return `200` even if the response text
   indicates failure.  This is an error with the server, and there is nothing
   this framework can do to validate such a scenario. */
  bool Good() const {
    return !service_error && (http_code >= 200 && http_code < 400);
  }
};

/* An HTTP service API, used by a RenderClient, to facilitate server
 communications.  This class is not intended to be used on its own. It is a
 helper class to perform server communications, but does not have any specific
 knowledge of a given client-server API.  The owning entity, such as
 RenderClient, is responsible for adhering to the server API. */
class HttpService {
 public:
  HttpService() {}
  virtual ~HttpService() {}
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HttpService);

  /* @name Server Interaction Interface */
  //@{
  /* Posts an HTML `<form>` to the specified `endpoint`.

   An HTML `<form>` may allow for a wide range of different kinds of `<input>`
   data.  For the full listing of available input `type`'s, see the
   <a href="https://developer.mozilla.org/en-US/docs/Web/HTML/Element/input">
   <tt>&lt;input&gt;</tt> documentation</a>.  For many of these types, the way
   an %HttpService will actually POST the data to the server will be the same, a
   `name` and `value` are provided and the server takes care of validating the
   `type`.  For example, an `<input type="number" name="age" min="0" max="150">`
   on the server will be posted to with simply `age={value}`.  If the `{value}`
   is invalid, the server will respond indicating so.  Similarly, an
   `<input type="checkbox" name="check">` would have a submission of just
   `check=on` or `check=off`.  Implementations are allowed to make the following
   assumptions:

   1. The server `<form>` is _valid_, namely that every `<input>` has a `name`
      attribute.
   2. Every input `value` being submitted corresponds appropriately with the
      provided field `name`.  That is, no validation of the provided `<input>`s
      is performed -- the `<form>` is simply submitted.  In the event that the
      caller has provided invalid data for a `name={value}` pair, the server
      is expected to respond with an error code and message indicating this.  If
      it does, this will be provided in the returned HttpResponse.

   @note
     Callers are encouraged to provide the `<input type="submit">` entry in the
     provided `data_fields`.  Although providing this entry is often not
     required, it is encouraged.

   Since file uploads (`<input type="file">`) are a special case, often
   requiring the underlying implementation to wield different machinery, these
   are required as a separate parameter.  This also enables the implementation
   to validate that the file path(s) provided exist.

   Finally, the %HttpService has no way of knowing what kind of a response to
   expect from the server.  A server may only provide an HTTP response code,
   a response code and an error message, or a response code and a binary file
   response.  As such, the %HttpService will expect to receive a response in
   addition to an HTTP return code.  In the event that the server responds with
   data of any kind (text or binary), the returned HttpResponse::data_path will
   have the path to the file where this information was saved.

   Some examples for filling out simple `<form>`s:
   @code{.cpp}
   // A simple form that does not upload any files nor return anything.
   // <form>
   //   <input type="text" name="name">
   //   <input type="number" name="age">
   //   <!-- NOTE: often, the `value` is irrelevant. -->
   //   <input type="submit" name="submit" value="Submit">
   // <form>
   // Submit form to /hello:
   auto response_1 = http_service_->PostForm(
       "hello",
       {{"name", "Sue"}, {"age", "34"}, {"submit", "Submit"}},
       {});  // No file fields.
   // Suppose `<input type="file" name="photo">` and
   // `<input type="file" name="id">` were added.
   auto response_2 = http_service->PostForm(
       "hello",
       {{"name", "Bo"}, {"age", "49"}, {"submit", "Submit"}},
       {{"photo", {"/path/to/bo-profile.jpg", "image/jpeg"}},
        // No mime-type for this file, std::nullopt indicates this.
        {"id", {"/path/to/bo-id.bin", std::nullopt}}});
   @endcode
   Make sure to check HttpResponse::Good() before continuing, or trying to load
   the HttpResponse::data_path as any specific kind of file.

   @param temp_directory
     The (shared) temporary directory, e.g., as created from
     drake::temp_directory().  File responses from a server will be stored here.
     The %HttpService does **not** own this directory, and is not responsible
     for deleting it.  No validity checks on this directory are performed.
   @param url
     The url this HTTP service will communicate with.  May **not** be the empty
     string.  May **not** have any trailing slashes.  Communications are
     typically constructed as  `url() + "/" + endpoint`.  For
     example, `https://drake.mit.edu` is acceptable, but
     `https://drake.mit.edu/` is not.
   @param port
     The TCP port this HTTP service will communicate on.  A value less than or
     equal to `0` implies no port level communication is needed.
   @param endpoint
     The endpoint the `<form>` should be posted to, e.g., `render`.  May **not**
     have a leading or trailing `/`, the post is sent to
     `{url() + "/" + endpoint}`.  If the `<form>` should be posted to the server
     root, then `endpoint` should be the empty string `""`.
   @param data_fields
     The entries for the `<input>` elements of the form.  Keys are the field
     name, and values are the field values.  For example:
     @code{.cpp}
     std::map<std::string, std::string> data_fields;
     // <input type="text" name="scene_sha256">
     data_fields["scene_sha256"] = "... hash value ...";
     // <input type="number" name="width">
     data_fields["width"] = "640";
     @endcode
     Do **not** zero pad numeric values, a given server may have trouble parsing
     unanticipated numeric values.  Supply the empty map if no data fields
     should be posted.
   @param file_fields
     Any entries for `<input type="file">` elements of the form.  Keys are the
     field name, and values are the (file path, optional mimetype) pair.  For
     example:
     @code{.cpp}
     std::map<std::string,
              std::pair<std::string, std::optional<std::string>>> file_fields;
     // <input type="file" name="scene">, known mimetype
     file_fields["scene"] = {"/path/to/scene_file.gltf", "model/gltf+json"};
     // <input type="file" name="id">, unknown mimetype
     file_fields["id"] = {"/path/to/id.bin", std::nullopt};
     @endcode
     Supply the empty map if no files are to be uploaded with the `<form>`.
   @param verbose
     Whether or not client/server communications should be logged to the
     console.
   @return HttpResponse
     The response from the server, including an HTTP code, and any potential
     additional server response text or data in HttpResponse::data_path.
   @throws std::exception
     An exception may be thrown in the event that the provided `endpoint` starts
     or ends with a `/` character.  An exception may also be thrown if an
     irrecoverable error is encountered, e.g., a file path provided in one of
     the values of `file_fields` does not exist.  Failed communications such as
     an inability to connect, or an invalid HTTP response code, should **not**
     produce an exception but rather encode this information in the
     returned HttpResponse for the caller to determine how to proceed. */
  HttpResponse PostForm(
      const std::string& temp_directory, const std::string& url, int port,
      const std::string& endpoint,
      const std::map<std::string, std::string>& data_fields,
      const std::map<std::string,
                     std::pair<std::string, std::optional<std::string>>>&
          file_fields,
      bool verbose = false) {
    ThrowIfUrlInvalid(url);
    ThrowIfEndpointInvalid(endpoint);
    ThrowIfFilesMissing(file_fields);
    return DoPostForm(temp_directory, url, port, endpoint, data_fields,
                      file_fields, verbose);
  }

 protected:
  /* The NVI-function for posting an HTML form to a render server. When
   PostForm calls this, it has already validated the url, the endpoint, and the
   existence of the files in `file_fields`. */
  virtual HttpResponse DoPostForm(
      const std::string& temp_directory, const std::string& url, int port,
      const std::string& endpoint,
      const std::map<std::string, std::string>& data_fields,
      const std::map<std::string,
                     std::pair<std::string, std::optional<std::string>>>&
          file_fields,
      bool verbose = false) = 0;
};

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
