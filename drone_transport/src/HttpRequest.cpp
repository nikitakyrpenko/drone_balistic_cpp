#include "HttpRequest.hpp"

#include <algorithm>
#include <cctype>
#include <ostream>

namespace {

const char* method_name(HttpRequest::Method method)
{
  switch (method) {
    case HttpRequest::Method::Get:
      return "GET";
    case HttpRequest::Method::Post:
      return "POST";
  }
  return "GET";
}

bool normalize_keys(const std::string& name)
{
  std::string lowered = name;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  return lowered == "host" || lowered == "content-length" || lowered == "connection";
}

}  // namespace

HttpRequest::HttpRequest(Method method, std::string host, std::string path)
  : method_(method)
  , host_(std::move(host))
  , path_(std::move(path))
{
  if (path_.empty()) {
    path_ = "/";
  }
}

HttpRequest& HttpRequest::add_header(std::string key, std::string value)
{
  headers_.emplace_back(std::move(key), std::move(value));
  return *this;
}

HttpRequest& HttpRequest::set_body(std::string body)
{
  body_ = std::move(body);
  return *this;
}

std::string HttpRequest::serialize_header() const
{
  std::string header;
  header.reserve(256);

  header += method_name(method_);
  header += ' ';
  header += path_;
  header += " HTTP/1.1\r\n";

  header += "Host: ";
  header += host_;
  header += "\r\n";

  if (method_ == Method::Post || !body_.empty()) {
    header += "Content-Length: ";
    header += std::to_string(body_.size());
    header += "\r\n";
  }

  header += "Connection: close\r\n";

  for (const auto& [key, value] : headers_) {
    if (normalize_keys(key)) {
      continue;
    }
    header += key;
    header += ": ";
    header += value;
    header += "\r\n";
  }

  header += "\r\n";

  return header;
}

std::ostream& operator<<(std::ostream& os, const HttpRequest& request)
{
  return os << request.serialize_header() << request.body();
}
