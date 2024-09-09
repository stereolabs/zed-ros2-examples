// Copyright 2024 Stereolabs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <string>
#include <iostream>
#include <fstream>
#include <vector>

// Check CUDA Status
#ifndef CUDA_CHECK
#define CUDA_CHECK(status) \
  do { \
    auto ret = (status); \
    if (ret != 0) { \
      std::cerr << "Cuda failure: " << cudaGetErrorString(ret) << std::endl; \
      abort(); \
    } \
  } while (0)
#endif


inline std::vector<std::string> split_str(const std::string & str, const std::string & delimiter)
{
  size_t pos_start = 0, pos_end, delim_len = delimiter.length();
  std::string token;
  std::vector<std::string> res;

  while ((pos_end = str.find(delimiter, pos_start)) != std::string::npos) {
    token = str.substr(pos_start, pos_end - pos_start);
    pos_start = pos_end + delim_len;
    res.push_back(token);
  }

  res.push_back(str.substr(pos_start));
  return res;
}

inline bool readFile(std::string filename, std::vector<uint8_t> & file_content)
{
  // open the file:
  std::ifstream instream(filename, std::ios::in | std::ios::binary);
  if (!instream.is_open()) {return true;}
  file_content = std::vector<uint8_t>(
    (std::istreambuf_iterator<char>(
      instream)), std::istreambuf_iterator<char>());
  return false;
}

#endif // UTILS_HPP_
