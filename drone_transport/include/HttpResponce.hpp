#pragma once

#include <string>
#include <string_view>
#include <unordered_map>

struct HttpResponse {
  std::string raw;

  std::string_view version;
  std::string_view status_code;
  std::string_view status_text;
  std::unordered_map<std::string_view, std::string_view> headers;
  std::string_view body;
};
