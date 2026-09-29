#pragma once

#include "JsonHttp.hpp"
#include "ResultReporter.hpp"
#include "TcpLink.hpp"

#include <string>

class HttpResultReporter : public ResultReporter {
public:
  HttpResultReporter(std::string host, std::string path, std::string api_key);

  bool save(const std::vector<SimulationStep>& steps, const std::string& student_id, const std::string& test_id) override;
  bool check(const std::string& student_id, const std::string& test_id) const override;

private:
  HttpHeaders auth_headers() const;

  TcpLink link_;
  std::string host_;
  std::string path_;
  std::string api_key_;
};
