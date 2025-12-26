/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *  Copyright (c) 2024, Smart Robotics Lab / Technical University of Munich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab, ETH Zurich, Smart Robotics Lab,
 *     Imperial College London, Technical University of Munich, nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************/

/**
 * @file DatasetReader.cpp
 * @brief Source file for the DatasetReader class.
 * @author Stefan Leutenegger
 */
 
#include <limits>

#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <okvis/ViInterface.hpp>
#include <okvis/DatasetReader.hpp>

namespace okvis {

DatasetReader::DatasetReader(
  const std::string& path, 
  const std::string& rgb_csv, const std::string& settings_yaml, 
  size_t numCameras, const std::set<size_t> &syncCameras,
  const Duration & deltaT) :
  numCameras_(numCameras), syncCameras_(syncCameras), deltaT_(deltaT), 
  rgb_csv_(rgb_csv), settings_yaml_(settings_yaml) {
  streaming_ = false;
  setDatasetPath(path);
  counter_ = 0;
}

DatasetReader::~DatasetReader() {
  stopStreaming();
}

bool DatasetReader::setDatasetPath(const std::string & path) {
  path_ = path;
  return true;
}

bool DatasetReader::setStartingDelay(const Duration &deltaT)
{
  if(streaming_) {
    LOG(WARNING)<< "starting delay ignored, because streaming already started";
    return false;
  }
  deltaT_ = deltaT;
  return true;
}

bool DatasetReader::isStreaming()
{
  return streaming_;
}

double DatasetReader::completion() const {
  if(streaming_) {
    return double(counter_)/double(numImages_);
  }
  return 0.0;
}

bool DatasetReader::startStreaming() {
  OKVIS_ASSERT_TRUE(Exception, !imagesCallbacks_.empty(), "no add image callback registered")
  OKVIS_ASSERT_TRUE(Exception, !imuCallbacks_.empty(), "no add IMU callback registered")

  YAML::Node settings = YAML::LoadFile(settings_yaml_);
  std::string cam_name = settings["cam_mono"].as<std::string>();
  std::string imu_name = settings["imu"].as<std::string>();

  // open the IMU file
  std::string line;
  imuFile_.open(path_ + "/" + imu_name + ".csv");
  OKVIS_ASSERT_TRUE(Exception, imuFile_.good(), "no imu file found at " << path_+"/imu.csv");
  int number_of_lines = 0;
  while (std::getline(imuFile_, line))
    ++number_of_lines;
  LOG(INFO)<< "No. IMU measurements: " << number_of_lines-1;
  if (number_of_lines - 1 <= 0) {
    LOG(ERROR)<< "no imu messages present in " << path_+"/imu0/data.csv";
    return -1;
  }
  // set reading position to second line
  imuFile_.clear();
  imuFile_.seekg(0, std::ios::beg);
  std::getline(imuFile_, line);

  // now open camera files
  std::vector<okvis::Time> times;
  okvis::Time latest(0);

  for(size_t i=0; i<numCameras_; ++i) {
    std::vector < std::pair<std::string, std::string> > imageNames;
    // first try gray
    int num_camera_images = readCameraImageCsv(cam_name, i, imageNames);
    if (num_camera_images <= 0) {
      // try also RGB
      num_camera_images = readCameraImageCsv("rgb", i, imageNames);
      OKVIS_ASSERT_TRUE(Exception, num_camera_images>0, "no images found for camera " << i)
      LOG(INFO)<< "No. cam " << i << " RGB images: " << num_camera_images;
    } else {
      LOG(INFO)<< "No. cam " << i << " images: " << num_camera_images;
    }
    if(i==0) {
      numImages_ = num_camera_images;
    }
    allImageNames_[i] = imageNames;

    // now also see if there might be depth images
    std::vector < std::pair<std::string, std::string> > depthImageNames;
    int num_depth_camera_images = 0;//readCameraImageCsv("depth", i, depthImageNames);
    if(num_depth_camera_images>0) {
      allDepthImageNames_[i] = depthImageNames;
      LOG(INFO)<< "No. cam " << i << " depth images: " << num_depth_camera_images;
    }
  }

  counter_ = 0;
  streaming_ = true;
  processingThread_ = std::thread(&DatasetReader::processing, this);

  return true;
}

std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        // Simple trim for leading/trailing whitespace, often needed in real-world CSVs
        token.erase(0, token.find_first_not_of(" \t\n\r"));
        token.erase(token.find_last_not_of(" \t\n\r") + 1);
        tokens.push_back(token);
    }
    return tokens;
}

int DatasetReader::readCameraImageCsv(const std::string& cam_name, size_t camIdx,
    std::vector < std::pair<std::string, std::string> >& imageFilenames) const
{
    imageFilenames.clear();
    
    std::ifstream in(rgb_csv_);
    std::string line;

    // Read and map the header row to find indices
    if (!std::getline(in, line)) return 0; 
    if (!line.empty() && line.back() == '\r') line.pop_back();

    std::vector<std::string> headers = split(line, ',');
    std::map<std::string, int> col_map;
    for (size_t i = 0; i < headers.size(); ++i) {
        col_map[headers[i]] = i;
    }

    // Required headers
    const std::string header_ts ="ts_" + cam_name + " (ns)";
    const std::string header_rgb0 = "path_" + cam_name;

    // Safely get indices
    auto get_index = [&](const std::string& key) -> int {
        return col_map[key];
    };

    int ts_idx = get_index(header_ts);
    int rgb0_idx = get_index(header_rgb0);   

    // Read and process data lines using fixed indices
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        if (!line.empty() && line.back() == '\r') line.pop_back();

        std::vector<std::string> tokens = split(line, ',');
        
        // Assign variables using indices, regardless of column order
        std::string nanoseconds = tokens[ts_idx];
        std::string rel_rgb0_path = tokens[rgb0_idx];

        imageFilenames.push_back(
          std::make_pair(nanoseconds, path_ + "/" + rel_rgb0_path));
    }

    return (int) imageFilenames.size();
}

bool DatasetReader::stopStreaming() {
  // Stop the pipeline
  if(processingThread_.joinable()) {
    processingThread_.join();
    streaming_ = false;
  }
  return true;
}

/// \brief Helper struct for image iterators.
struct ImageIterators {
  /// \brief Camera iterators.
  std::vector<std::vector<std::pair<std::string, std::string>>::iterator> cam_iterators;
  /// \brief Depth camera iterators.
  std::map<size_t, std::vector<std::pair<std::string, std::string>>::iterator> depthCam_iterators;
  /// \brief Camera iterator ends.
  std::vector<std::vector<std::pair<std::string, std::string>>::iterator> cam_ends;
  /// \brief Depth camera iterator ends.
  std::map<size_t, std::vector<std::pair<std::string, std::string>>::iterator> depthCam_ends;
  const uint64_t tolNSec = 10000000; ///< Sync time tolerance in nano-seconds.

  /// \brief Arg min of the timestamps in current iterators.
  /// @param[out] timestamp The smallest timestamp in current iterators.
  /// @param[out] isDepth If it is a depth camera.
  /// \return The arg min (idx).
  int argMinTime(uint64_t& timestamp, bool& isDepth) {
    timestamp = std::numeric_limits<uint64_t>::max();
    int i_min = -1;
    for(size_t i = 0; i<cam_iterators.size(); ++i) {
      if(cam_iterators.at(i) == cam_ends.at(i)) continue;
      uint64_t timestamp_tmp = std::atol(cam_iterators.at(i)->first.c_str());
      if(timestamp_tmp < timestamp) {
        i_min = i;
        timestamp = timestamp_tmp;
        isDepth = false;
      }
      /// \todo Fixme: reading async depth only frames is broken with the following:
      /*if (depthCam_iterators.count(i)) {
        if (depthCam_iterators.at(i) != depthCam_ends.at(i)) {
          uint64_t timestamp_tmp = std::atol(depthCam_iterators.at(i)->first.c_str());
          if (timestamp_tmp < timestamp) {
            i_min = i;
            timestamp = timestamp_tmp;
            isDepth = true;
          }
        }
      }*/
    }
    return i_min;
  }

  /// \brief Arg min of the timestamps in current iterators.
  /// @param[in] syncGroup Set of camera indices to treat as synced.
  /// @param[out] timestamp The smallest timestamp in current iterators.
  /// @param[out] isDepth If it is a depth camera.
  /// \return The arg min (idx).
  int argMinTime(const std::set<size_t>& syncGroup, uint64_t& timestamp, bool& isDepth) {
    timestamp = std::numeric_limits<uint64_t>::max();
    size_t i_min = *syncGroup.begin();
    for(size_t i = 0; i<cam_iterators.size(); ++i) {
      if(!syncGroup.count(i)) continue;
      if(cam_iterators.at(i) == cam_ends.at(i)) return -1;
      uint64_t timestamp_tmp = std::atol(cam_iterators.at(i)->first.c_str());
      if(timestamp_tmp < timestamp) {
        i_min = i;
        timestamp = timestamp_tmp;
        isDepth = false;
      }
    }
    return i_min;
  }

  /// \brief Check if sync group is indeed synced.
  /// @param[in] syncGroup Set of camera indices to treat as synced.
  /// \return True if synced.
  bool isSynched(const std::set<size_t>& syncGroup) {
    uint64_t timestamp_min;
    bool isDepth_min;
    int i_min = argMinTime(syncGroup, timestamp_min, isDepth_min);
    if(i_min == -1) return false;
    for(size_t i : syncGroup) {
      if(cam_iterators.at(i) == cam_ends.at(i)) return false;
      uint64_t timestamp = std::atol(cam_iterators.at(i)->first.c_str());
      if(timestamp-timestamp_min > tolNSec) {
        return false;
      }
    }
    return true;
  }
};

void  DatasetReader::processing() {
  std::string line;
  okvis::Time start(0.0);
  const size_t numCameras = allImageNames_.size();
  ImageIterators iterators;
  iterators.cam_iterators.resize(numCameras);
  iterators.cam_ends.resize(numCameras);
  for (size_t i = 0; i < numCameras; ++i) {
    iterators.cam_iterators.at(i) = allImageNames_.at(i).begin();
    iterators.cam_ends.at(i) = allImageNames_.at(i).end();
    if(allDepthImageNames_.count(i)){
      iterators.depthCam_iterators[i] = allDepthImageNames_.at(i).begin();
      iterators.depthCam_ends[i] = allDepthImageNames_.at(i).end();
    }
  }

  while (streaming_) {

    // outer sync loop: add whatever has the smallest timestamp, unless sync needed
    int i_min = 0;
    bool needSync = false;
    uint64_t timestamp_min = 0;
    bool isDepthCam = false;
    do {
      i_min = iterators.argMinTime(timestamp_min, isDepthCam);
      if(i_min == -1) {
        streaming_ = false;
        return; // all processed, finished
      }

      if(syncCameras_.count(i_min)) {
        needSync = !iterators.isSynched(syncCameras_);
      }
      
      if(allDepthImageNames_.count(i_min)) {
        std::set<size_t> depthGroup;
        depthGroup.insert(i_min);
        needSync |= !iterators.isSynched(depthGroup);
      }

      if(needSync) {
        if(isDepthCam) {
          LOG(WARNING)
              << "depth image at t=" << timestamp_min << " without correspondence -- dropping";
          iterators.depthCam_iterators.at(i_min)++;
          if(iterators.depthCam_iterators.at(i_min) == allDepthImageNames_.at(i_min).end()) {
            streaming_ = false;
            return; // finished!
          }
        } else {
          LOG(WARNING)
              << "image at t=" << timestamp_min << " without correspondence -- dropping";
          iterators.cam_iterators.at(i_min)++;
          if(iterators.cam_iterators.at(i_min) == allImageNames_.at(i_min).end()) {
            streaming_ = false;
            return; // finished!
          }
        }
      }
    } while (needSync);

    // add (unless at the end)...
    std::map<size_t, cv::Mat> images;
    std::map<size_t, cv::Mat> depthImages;
    if(syncCameras_.count(i_min)) {
      if(iterators.cam_iterators.at(i_min) == allImageNames_.at(i_min).end()) {
        streaming_ = false;
        return; // finished!
      }
      for(size_t i : syncCameras_) {
        const std::string & filename = iterators.cam_iterators.at(i)->second;
        cv::Mat filtered;
        boost::filesystem::path p(filename);
        std::string directory = p.parent_path().parent_path().filename().string();
        if(directory.size() > 3 && directory.substr(0,3).compare("cam") == 0) {
          filtered = cv::imread(filename, cv::IMREAD_GRAYSCALE); // force gray already here
        } else {
          filtered = cv::imread(filename); // colour
        }
        OKVIS_ASSERT_TRUE(
              Exception, !filtered.empty(),
              "cam " << i << " missing image :" << std::endl << filename)
        images[i] = filtered;
        iterators.cam_iterators.at(i)++; // advance to next
      }
    } else {
      const std::string & filename = iterators.cam_iterators.at(i_min)->second;
      cv::Mat filtered;
      boost::filesystem::path p(filename);
      std::string directory = p.parent_path().parent_path().filename().string();
      if(directory.size() > 3 && directory.substr(0,3).compare("cam") == 0) {
        filtered = cv::imread(filename, cv::IMREAD_GRAYSCALE); // force gray already here
      } else {
        filtered = cv::imread(filename); // colour
      }
      OKVIS_ASSERT_TRUE(
        Exception, !filtered.empty(),
        "cam " << i_min << " missing image :" << std::endl << filename)
      images[i_min] = filtered;
      iterators.cam_iterators.at(i_min)++; // advance to next
    }
    if(allDepthImageNames_.count(i_min)) {
      if(iterators.depthCam_iterators.at(i_min) == allDepthImageNames_.at(i_min).end()) {
        streaming_ = false;
        return; // finished!
      }
      const std::string & depthFilename = iterators.depthCam_iterators.at(i_min)->second;
      cv::Mat depthImg = cv::imread(depthFilename, cv::IMREAD_UNCHANGED);
      OKVIS_ASSERT_TRUE(
            Exception, !depthImg.empty(),
            "cam " << i_min << " missing depth image :" << std::endl << depthFilename)

      // convert back
      if(depthImg.type() == CV_32FC1) {
        logImageTransform(depthImg);
      } else {
        depthImg.convertTo(depthImg,  CV_32F, 0.001);
      }

      depthImages[i_min] = depthImg;
      iterators.depthCam_iterators.at(i_min)++; // advance to next

      // next also read synced image, if not already added from sync group above.
      if(!syncCameras_.count(i_min)) {
        const std::string & filename = iterators.cam_iterators.at(i_min)->second;
        cv::Mat filtered;
        if(filename.substr(filename.size()-3,filename.size()).compare("png") == 0) {
          filtered = cv::imread(filename, cv::IMREAD_GRAYSCALE); // force gray already here
        } else {
          filtered = cv::imread(filename); // colour
        }

        OKVIS_ASSERT_TRUE(
            Exception, !filtered.empty(),
            "cam " << i_min << " missing image :" << std::endl << filename)
        images[i_min] = filtered;
        iterators.cam_iterators.at(i_min)++; // advance to next
      }
    }

    // time
    Time t;
    t.fromNSec(timestamp_min);
    if (start == okvis::Time(0.0)) {
      start = t;
    }

    // get all IMU measurements till then
    okvis::Time t_imu = start;
    do {
      if (!std::getline(imuFile_, line)) {
        streaming_ = false;
        return;
      }

      std::stringstream stream(line);
      std::string s;
      std::getline(stream, s, ',');
      uint64_t nanoseconds = std::stol(s.c_str());
      //uint64_t nanoseconds = uint64_t(std::stod(s.c_str()) * 10e9);
      //double seconds = std::stod(s.c_str());

      Eigen::Vector3d gyr;
      for (int j = 0; j < 3; ++j) {
        std::getline(stream, s, ',');
        gyr[j] = std::stof(s);
      }

      Eigen::Vector3d acc;
      for (int j = 0; j < 3; ++j) {
        std::getline(stream, s, ',');
        acc[j] = std::stof(s);
      }

      t_imu.fromNSec(nanoseconds);
      //t_imu.fromSec(seconds);

      // add the IMU measurement for (blocking) processing
      if (t_imu - start + okvis::Duration(1.0) > deltaT_) {
        for (auto &imuCallback : imuCallbacks_) {
          imuCallback(t_imu, acc, gyr);
        }
      }

    } while (t_imu <= t + okvis::Duration(0.021));

    // finally we are ready to call the callback
    for (auto& imagesCallback : imagesCallbacks_) {
      imagesCallback(t, images, depthImages);
    }
    if(images.count(0) && !images.at(0).empty()) {
      ++counter_; // reference for counter is always image 0.
    }
  }

  return;
}

}
