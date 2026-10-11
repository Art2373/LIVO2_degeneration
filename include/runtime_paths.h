#pragma once

// Runtime products are separate from the source/install tree. Configure once
// on the mapper thread before opening logs; callers do not mutate this later.
#include <string>
#include <stdexcept>
#include <boost/filesystem.hpp>

namespace runtime_paths
{
inline std::string &outputDirectoryStorage()
{
  static std::string path = "/tmp/livo2_degeneration/";
  return path;
}
inline const std::string &outputDirectory() { return outputDirectoryStorage(); }
inline void configureOutputDirectory(const std::string &directory)
{
  if (directory.empty()) throw std::invalid_argument("runtime/output_directory must not be empty");
  const boost::filesystem::path path = boost::filesystem::absolute(directory);
  const char *subdirectories[] = {"", "result", "pcd", "image", "Colmap/sparse/0", "Colmap/images", "ref_cur_combine"};
  for (const char *subdirectory : subdirectories)
    boost::filesystem::create_directories(path / subdirectory);
  outputDirectoryStorage() = path.string();
  if (outputDirectoryStorage().back() != '/') outputDirectoryStorage() += '/';
}
}
