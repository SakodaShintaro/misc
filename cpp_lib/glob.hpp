#ifndef UTILS_GLOB_HPP
#define UTILS_GLOB_HPP

#include <glob.h>

#include <algorithm>
#include <string>
#include <vector>

namespace utils
{

std::vector<std::string> glob(const std::string & input_dir)
{
  glob_t buffer;
  std::vector<std::string> files;
  glob((input_dir + "*").c_str(), 0, NULL, &buffer);
  for (size_t i = 0; i < buffer.gl_pathc; i++) {
    files.push_back(buffer.gl_pathv[i]);
  }
  globfree(&buffer);
  std::sort(files.begin(), files.end());
  return files;
}

} // namespace utils

#endif
