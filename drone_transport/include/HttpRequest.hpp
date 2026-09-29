#pragma once

#include <cstddef>
#include <iosfwd>
#include <string>
#include <utility>
#include <vector>

class HttpRequest {
public:
  enum class Method { Get, Post };

  HttpRequest(Method method, std::string host, std::string path = "/");

  HttpRequest& add_header(std::string key, std::string value);
  HttpRequest& set_body(std::string body);

  Method method() const { return method_; }
  const std::string& host() const { return host_; }
  const std::string& path() const { return path_; }
  const std::string& body() const { return body_; }
  size_t content_length() const { return body_.size(); }

  std::string serialize_header() const;

private:
  Method method_;
  std::string host_;
  std::string path_;
  std::string body_;
  std::vector<std::pair<std::string, std::string>> headers_;
};

std::ostream& operator<<(std::ostream& os, const HttpRequest& request);
