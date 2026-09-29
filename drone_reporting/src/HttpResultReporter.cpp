#include "HttpResultReporter.hpp"

#include <iostream>

HttpResultReporter::HttpResultReporter(std::string host, std::string path, std::string api_key)
  : link_()
  , host_(std::move(host))
  , path_(std::move(path))
  , api_key_(std::move(api_key))
{
}

HttpHeaders HttpResultReporter::auth_headers() const
{
  return {//{"User-Agent",
          //"Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
          //"(KHTML, like Gecko) Chrome/150.0.0.0 Safari/537.36"},
          //{"Cookie", "wssplashchk=be3231d61d41ce1c28f3c7eda37d1c262adca86d.1785923713.1"},
          {"x-api-key", api_key_}};
}

bool HttpResultReporter::save(const std::vector<SimulationStep>& steps, const std::string& student_id, const std::string& test_id)
{
  const nlohmann::ordered_json report = to_json(steps, student_id, test_id);
  const std::optional<HttpResponse> response = post_json(link_, host_, path_, report, auth_headers());

  if (!response) {
    std::cerr << "reporting failed: no answer from " << host_ << path_ << "\n";
    return false;
  }

  std::cout << "reported to " << host_ << path_ << " -> " << response->status_code << " " << response->status_text << "\n";

  return response->status_code.starts_with("2");
}

bool HttpResultReporter::check(const std::string& student_id, const std::string& test_id) const
{
  const std::string check_path = path_ + "/" + test_id + "/" + student_id;
  const std::optional<HttpResponse> response = get_json(link_, host_, check_path, auth_headers());

  if (!response) {
    std::cerr << "check failed: no answer from " << host_ << check_path << "\n";
    return false;
  }
  if (response->status_code.starts_with("4") || response->status_code.starts_with("5")) {
    std::cerr << "Status report failed with status code: " << response->status_code;
    return false;
  }

  return response->status_code.starts_with("2");
}
