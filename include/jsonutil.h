// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INCLUDE_JSONUTIL_H_
#define INCLUDE_JSONUTIL_H_

#include <fstream>
#include <iostream>
#include <string>
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/writer.h"

namespace hobot {
namespace centerpoint {

typedef uint64_t TimeStamp;

class JSONUtil {
 public:
  static int ParseJson(const std::string& config_file, rapidjson::Document& document) {
    std::ifstream cfg_file(config_file);
    if (!cfg_file.is_open()) {
      std::cout << "open cfg_file:" << config_file << " failed!" << std::endl;
      return -1;
    }

    rapidjson::IStreamWrapper isw(cfg_file);
    document.ParseStream(isw);
    if (document.HasParseError()) {
      std::cout << "Parsing config file " << config_file << " failed!" << std::endl;
      return -1;
    }

    return 0;
  }

  static std::string GetWorkingDir(const std::string& config_file) {
    std::string config_dir;
    size_t pos = config_file.rfind('/');
    if (pos == std::string::npos) {
      pos = config_file.rfind('\\');
    }
    if (pos == std::string::npos) {
      config_dir = "./";
    } else {
      config_dir = config_file.substr(0, pos);
    }
    return config_dir;
  }
};

}  // namespace centerpoint
}  // namespace hobot

#endif  // INCLUDE_JSONUTIL_H_

