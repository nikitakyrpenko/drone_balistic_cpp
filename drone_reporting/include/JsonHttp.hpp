#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "HttpRequest.hpp"
#include "HttpResponce.hpp"
#include "TcpLink.hpp"

#include "json.hpp"

using HttpHeaders = std::vector<std::pair<std::string, std::string>>;

template <typename T>
concept JsonSerializable = requires(const T& value) { nlohmann::ordered_json(value); };

inline void apply_headers(HttpRequest& request, const HttpHeaders& headers)
{
  for (const auto& [key, value] : headers) {
    request.add_header(key, value);
  }
}

template <JsonSerializable T>
std::optional<HttpResponse> post_json(
  const TcpLink& link, std::string host, std::string path, const T& value, const HttpHeaders& headers = {})
{
  HttpRequest request(HttpRequest::Method::Post, std::move(host), std::move(path));
  apply_headers(request, headers);

  request.add_header("Content-Type", "application/json").set_body(nlohmann::ordered_json(value).dump());

  return link.dispatch(request);
}

inline std::optional<HttpResponse> get_json(const TcpLink& link, std::string host, std::string path, const HttpHeaders& headers = {})
{
  HttpRequest request(HttpRequest::Method::Get, std::move(host), std::move(path));
  apply_headers(request, headers);

  request.add_header("Accept", "application/json");

  return link.dispatch(request);
}
