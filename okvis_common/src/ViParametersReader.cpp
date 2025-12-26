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
 * @file ViParametersReader.cpp
 * @brief Source file for the VioParametersReader class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <algorithm>

#include <glog/logging.h>

#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>

#include <opencv2/core/core.hpp>

#include <okvis/ViParametersReader.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

// The default constructor.
ViParametersReader::ViParametersReader()
    : readConfigFile_(false) {
}

// The constructor. This calls readConfigFile().
ViParametersReader::ViParametersReader(const std::string& calibration_yaml, const std::string& settings_yaml) {
  // reads
  readConfigFile(calibration_yaml, settings_yaml);
}

// Read and parse a config file.
void ViParametersReader::readConfigFile(const std::string& calibration_yaml, const std::string& settings_yaml) {

  // reads
  cv::FileStorage file(settings_yaml, cv::FileStorage::READ);
  OKVIS_ASSERT_TRUE(Exception, file.isOpened(),
                    "Could not open config file: " << settings_yaml)
  LOG(INFO) << "Opened configuration file: " << settings_yaml;

  ///////////// VSLAM-LAB
  YAML::Node calibration = YAML::LoadFile(calibration_yaml);
  YAML::Node settings = YAML::LoadFile(settings_yaml);
  std::string cam_name = settings["cam_mono"].as<std::string>();
  std::string imu_name = settings["imu"].as<std::string>();

  const YAML::Node& cameras = calibration["cameras"];
  YAML::Node cam;
  for (int i{0}; i < cameras.size(); ++i){
    if (cameras[i]["cam_name"].as<std::string>() == cam_name){
      cam = cameras[i];
      break;
      }
  }

  const YAML::Node& imus = calibration["imus"];
  YAML::Node imu;
  for (int i{0}; i < imus.size(); ++i){
    if (imus[i]["imu_name"].as<std::string>() == imu_name){
      imu = imus[i];
      break;
      }
  }

  // camera calibration
  std::vector<CameraCalibration,Eigen::aligned_allocator<CameraCalibration>> calibrations;
  if(!getCameraCalibration(calibrations, cam)) {
    LOG(FATAL) << "Did not find any calibration!";
  }

  size_t camIdx = 0;
  for (size_t i = 0; i < calibrations.size(); ++i) {

    std::shared_ptr<const kinematics::Transformation> T_SC_okvis_ptr(
          new kinematics::Transformation(calibrations[i].T_SC.r(),
                                                calibrations[i].T_SC.q().normalized()));
    if (strcmp(calibrations[i].distortionType.c_str(), "equidistant") == 0) {
      std::shared_ptr<cameras::PinholeCamera<cameras::EquidistantDistortion>> cam;
      cam.reset(new cameras::PinholeCamera<
                  cameras::EquidistantDistortion>(
                  calibrations[i].imageDimension[0],
                  calibrations[i].imageDimension[1],
                  calibrations[i].focalLength[0],
                  calibrations[i].focalLength[1],
                  calibrations[i].principalPoint[0],
                  calibrations[i].principalPoint[1],
                  cameras::EquidistantDistortion(
                    calibrations[i].distortionCoefficients[0],
                    calibrations[i].distortionCoefficients[1],
                    calibrations[i].distortionCoefficients[2],
                    calibrations[i].distortionCoefficients[3])/*, id ?*/));
      cam->initialiseUndistortMaps(); // set up undistorters
      cam->initialiseCameraAwarenessMaps();
      viParameters_.nCameraSystem.addCamera(
          T_SC_okvis_ptr,
          std::static_pointer_cast<const cameras::CameraBase>(cam),
          cameras::NCameraSystem::Equidistant, true,
          calibrations[i].cameraType);
      std::stringstream s;
      s << calibrations[i].T_SC.T();
      LOG(INFO) << "Equidistant pinhole camera " << camIdx
                << " with T_SC=\n" << s.str();
    } else if (strcmp(calibrations[i].distortionType.c_str(), "radialtangential") == 0
               || strcmp(calibrations[i].distortionType.c_str(), "plumb_bob") == 0) {
      std::shared_ptr<cameras::PinholeCamera<cameras::RadialTangentialDistortion>> cam;
      cam.reset(new cameras::PinholeCamera<
                  cameras::RadialTangentialDistortion>(
                  calibrations[i].imageDimension[0],
                  calibrations[i].imageDimension[1],
                  calibrations[i].focalLength[0],
                  calibrations[i].focalLength[1],
                  calibrations[i].principalPoint[0],
                  calibrations[i].principalPoint[1],
                  cameras::RadialTangentialDistortion(
                    calibrations[i].distortionCoefficients[0],
                    calibrations[i].distortionCoefficients[1],
                    calibrations[i].distortionCoefficients[2],
                    calibrations[i].distortionCoefficients[3])/*, id ?*/));
      cam->initialiseUndistortMaps(); // set up undistorters
      cam->initialiseCameraAwarenessMaps();
      viParameters_.nCameraSystem.addCamera(
          T_SC_okvis_ptr,
          std::static_pointer_cast<const cameras::CameraBase>(cam),
          cameras::NCameraSystem::RadialTangential, true,
          calibrations[i].cameraType);
      std::stringstream s;
      s << calibrations[i].T_SC.T();
      LOG(INFO) << "Radial tangential pinhole camera " << camIdx
                << " with T_SC=\n" << s.str();
    } else if (strcmp(calibrations[i].distortionType.c_str(), "radialtangential8") == 0
               || strcmp(calibrations[i].distortionType.c_str(), "plumb_bob8") == 0) {
      std::shared_ptr<cameras::PinholeCamera<cameras::RadialTangentialDistortion8>> cam;
      cam.reset(new cameras::PinholeCamera<cameras::RadialTangentialDistortion8>(
                  calibrations[i].imageDimension[0],
                  calibrations[i].imageDimension[1],
                  calibrations[i].focalLength[0],
                  calibrations[i].focalLength[1],
                  calibrations[i].principalPoint[0],
                  calibrations[i].principalPoint[1],
                  cameras::RadialTangentialDistortion8(
                    calibrations[i].distortionCoefficients[0],
                    calibrations[i].distortionCoefficients[1],
                    calibrations[i].distortionCoefficients[2],
                    calibrations[i].distortionCoefficients[3],
                    calibrations[i].distortionCoefficients[4],
                    calibrations[i].distortionCoefficients[5],
                    calibrations[i].distortionCoefficients[6],
                    calibrations[i].distortionCoefficients[7])/*, id ?*/));
      cam->initialiseUndistortMaps(); // set up undistorters
      cam->initialiseCameraAwarenessMaps();
      viParameters_.nCameraSystem.addCamera(
          T_SC_okvis_ptr,
          std::static_pointer_cast<const cameras::CameraBase>(cam),
          cameras::NCameraSystem::RadialTangential8, true,
          calibrations[i].cameraType);
      std::stringstream s;
      s << calibrations[i].T_SC.T();
      LOG(INFO) << "Radial tangential 8 pinhole camera " << camIdx
                << " with T_SC=\n" << s.str();
    } else {
      LOG(ERROR) << "unrecognized distortion type " << calibrations[i].distortionType;
    }
    ++camIdx;
  }

  //camera parameters.
  parseEntry(file["camera_parameters"], "timestamp_tolerance",
             viParameters_.camera.timestamp_tolerance);
  cv::FileNode T = file["camera_parameters"]["sync_cameras"];
  OKVIS_ASSERT_TRUE(
    Exception, T.isSeq(),
    "missing real array parameter " << "camera_parameters" << ": " << "sync_cameras")
  for(auto iter = T.begin(); iter != T.end(); ++iter) {
    viParameters_.camera.sync_cameras.insert(int(*iter));
  }
  parseEntry(file["camera_parameters"], "image_delay",
             viParameters_.camera.image_delay);
  parseEntry(file["camera_parameters"]["online_calibration"], "do_extrinsics",
             viParameters_.camera.online_calibration.do_extrinsics);
  parseEntry(file["camera_parameters"]["online_calibration"], "do_extrinsics_final_ba",
             viParameters_.camera.online_calibration.do_extrinsics_final_ba);
  parseEntry(file["camera_parameters"]["online_calibration"], "sigma_r",
             viParameters_.camera.online_calibration.sigma_r);
  parseEntry(file["camera_parameters"]["online_calibration"], "sigma_alpha",
             viParameters_.camera.online_calibration.sigma_alpha);
  parseEntry(file["camera_parameters"]["online_calibration"], "sigma_r_final_ba",
             viParameters_.camera.online_calibration.sigma_r_final_ba);
  parseEntry(file["camera_parameters"]["online_calibration"], "sigma_alpha_final_ba",
             viParameters_.camera.online_calibration.sigma_alpha_final_ba);

  // IMU parameters.
  viParameters_.imu.use = true;
  viParameters_.imu.a_max = imu["a_max"].as<double>();
  viParameters_.imu.g_max = imu["g_max"].as<double>();
  viParameters_.imu.sigma_g_c = imu["sigma_g_c"].as<double>();
  viParameters_.imu.sigma_bg = imu["sigma_bg"].as<double>();
  viParameters_.imu.sigma_a_c = imu["sigma_a_c"].as<double>();

  viParameters_.imu.sigma_ba = imu["sigma_ba"].as<double>();
  viParameters_.imu.sigma_gw_c = imu["sigma_gw_c"].as<double>();
  viParameters_.imu.sigma_aw_c = imu["sigma_aw_c"].as<double>();
  std::vector<double> a0 = imu["a0"].as<std::vector<double>>();
  std::vector<double> g0 = imu["g0"].as<std::vector<double>>();
  viParameters_.imu.a0 = Eigen::Vector3d(a0[0],a0[1],a0[2]);
  viParameters_.imu.g0 = Eigen::Vector3d(g0[0],g0[1],g0[2]);
  viParameters_.imu.g = imu["g"].as<double>();

  std::vector<double> T_SC_imu_data = imu["T_SC"].as<std::vector<double>>();
  Eigen::Matrix4d T_SC_imu = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(T_SC_imu_data.data());  
  viParameters_.imu.T_BS = kinematics::Transformation(T_SC_imu);

  // Parameters for detection etc.
  parseEntry(file["frontend_parameters"], "detection_threshold",
             viParameters_.frontend.detection_threshold);
  parseEntry(file["frontend_parameters"], "absolute_threshold",
             viParameters_.frontend.absolute_threshold);
  parseEntry(file["frontend_parameters"], "matching_threshold",
             viParameters_.frontend.matching_threshold);
  parseEntry(file["frontend_parameters"], "octaves",
             viParameters_.frontend.octaves);
  parseEntry(file["frontend_parameters"], "max_num_keypoints",
             viParameters_.frontend.max_num_keypoints);
  parseEntry(file["frontend_parameters"], "keyframe_overlap",
             viParameters_.frontend.keyframe_overlap);
  parseEntry(file["frontend_parameters"], "use_cnn",
             viParameters_.frontend.use_cnn);
  parseEntry(file["frontend_parameters"], "parallelise_detection",
             viParameters_.frontend.parallelise_detection);
  parseEntry(file["frontend_parameters"], "num_matching_threads",
             viParameters_.frontend.num_matching_threads);

  // Parameters regarding the estimator.
  parseEntry(file["estimator_parameters"], "num_keyframes",
             viParameters_.estimator.num_keyframes);
  parseEntry(file["estimator_parameters"], "num_loop_closure_frames",
             viParameters_.estimator.num_loop_closure_frames);
  parseEntry(file["estimator_parameters"], "num_imu_frames",
             viParameters_.estimator.num_imu_frames);
  parseEntry(file["estimator_parameters"], "do_loop_closures",
             viParameters_.estimator.do_loop_closures);
  parseEntry(file["estimator_parameters"], "do_final_ba",
             viParameters_.estimator.do_final_ba);
  parseEntry(file["estimator_parameters"], "enforce_realtime",
             viParameters_.estimator.enforce_realtime);
  parseEntry(file["estimator_parameters"], "realtime_min_iterations",
             viParameters_.estimator.realtime_min_iterations);
  parseEntry(file["estimator_parameters"], "realtime_max_iterations",
             viParameters_.estimator.realtime_max_iterations);
  parseEntry(file["estimator_parameters"], "realtime_time_limit",
             viParameters_.estimator.realtime_time_limit);
  parseEntry(file["estimator_parameters"], "realtime_num_threads",
             viParameters_.estimator.realtime_num_threads);
  parseEntry(file["estimator_parameters"], "full_graph_iterations",
             viParameters_.estimator.full_graph_iterations);
  parseEntry(file["estimator_parameters"], "full_graph_num_threads",
             viParameters_.estimator.full_graph_num_threads);
  parseEntry(file["estimator_parameters"], "p_dbow",
             viParameters_.estimator.p_dbow);
  parseEntry(file["estimator_parameters"], "drift_percentage_heuristic",
             viParameters_.estimator.drift_percentage_heuristic);

  // Some options for how and what to output.
  parseEntry(file["output_parameters"], "display_matches",
             viParameters_.output.display_matches);
  parseEntry(file["output_parameters"], "display_overhead",
             viParameters_.output.display_overhead);

  // done!
  readConfigFile_ = true;
}

void ViParametersReader::parseEntry(const cv::FileNode& file, std::string name, int& readValue) {
  OKVIS_ASSERT_TRUE(Exception, file[name].isInt(),
                    "missing integer parameter " << file.name() << ": " << name)
  file[name] >> readValue;
}

void ViParametersReader::parseEntry(const cv::FileNode &file,
                                    std::string name, double& readValue) {
  OKVIS_ASSERT_TRUE(Exception, file[name].isReal(),
                    "missing real parameter " << file.name() << ": " << name)
  file[name] >> readValue;
}

// Parses booleans from a cv::FileNode. OpenCV sadly has no implementation like this.
void ViParametersReader::parseEntry(const cv::FileNode& file, std::string name, bool& readValue) {
  OKVIS_ASSERT_TRUE(Exception, file[name].isInt() || file[name].isString(),
                    "missing boolean parameter " << file.name() << ": " << name)
  if (file[name].isInt()) {
    readValue = int(file[name]) != 0;
    return;
  }
  if (file[name].isString()) {
    std::string str = std::string(file[name]);
    // cut out first word. str currently contains everything including comments
    str = str.substr(0,str.find(" "));
    // transform it to all lowercase
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    /* from yaml.org/type/bool.html:
     * Booleans are formatted as English words
     * (“true”/“false”, “yes”/“no” or “on”/“off”)
     * for readability and may be abbreviated as
     * a single character “y”/“n” or “Y”/“N”. */
    if (str.compare("false")  == 0
        || str.compare("no")  == 0
        || str.compare("n")   == 0
        || str.compare("off") == 0) {
      readValue = false;
      return;
    }
    if (str.compare("true")   == 0
        || str.compare("yes") == 0
        || str.compare("y")   == 0
        || str.compare("on")  == 0) {
      readValue = true;
      return;
    }
    OKVIS_THROW(Exception, "Boolean with uninterpretable value " << str)
  }
  return;
}

void ViParametersReader::parseEntry(const cv::FileNode &file, std::string name,
                                    Eigen::Matrix4d& readValue) {
  cv::FileNode T = file[name];
  OKVIS_ASSERT_TRUE(Exception, T.isSeq(),
                    "missing real array parameter " << file.name() << ": " << name)
  readValue << T[0], T[1], T[2], T[3],
               T[4], T[5], T[6], T[7],
               T[8], T[9], T[10], T[11],
               T[12], T[13], T[14], T[15];
}

void ViParametersReader::parseEntry(const cv::FileNode& file, std::string name,
                                    Eigen::Vector3d& readValue) {
  cv::FileNode T = file[name];
  OKVIS_ASSERT_TRUE(Exception, T.isSeq(),
                    "missing real array parameter " << file.name() << ": " << name)
  readValue << T[0], T[1], T[2];
}

bool ViParametersReader::getCameraCalibration(
    std::vector<CameraCalibration,Eigen::aligned_allocator<CameraCalibration>> & calibrations,
    const YAML::Node& cam) {

  bool success = getCalibrationViaConfig(calibrations, cam);
  return success;
}

// Get the camera calibration via the configuration file.
bool ViParametersReader::getCalibrationViaConfig(
  std::vector<CameraCalibration,Eigen::aligned_allocator<CameraCalibration>> & calibrations,
  const YAML::Node& cam) const {
    
    calibrations.clear();
    CameraCalibration calib;  
    calib.imageDimension << cam["image_dimension"][0].as<int>(), cam["image_dimension"][1].as<int>();
    calib.focalLength << cam["focal_length"][0].as<double>(), cam["focal_length"][1].as<double>();
    calib.principalPoint << cam["principal_point"][0].as<double>(), cam["principal_point"][1].as<double>();

    if (cam["distortion_type"] && cam["distortion_coefficients"]) {
      std::vector<double> dist = cam["distortion_coefficients"].as<std::vector<double>>(); 
      if (cam["distortion_type"].as<std::string>() == "radtan4"){
        calib.distortionType = "radialtangential";
        calib.distortionCoefficients.resize(4);
        calib.distortionCoefficients << dist[0], dist[1], dist[2], dist[3];
      }
      if (cam["distortion_type"].as<std::string>() == "radtan5"){
        calib.distortionType = "radialtangential8";
        calib.distortionCoefficients.resize(8);
        calib.distortionCoefficients << dist[0], dist[1], dist[2], dist[3], dist[4], 0.0, 0.0, 0.0;
      }
      if (cam["distortion_type"].as<std::string>() == "radtan8"){
        calib.distortionType = "radialtangential8";
        calib.distortionCoefficients.resize(8);
        calib.distortionCoefficients << dist[0], dist[1], dist[2], dist[3], dist[4], dist[5], dist[6], dist[7];
      }
      if (cam["distortion_type"].as<std::string>() == "equid4"){
        calib.distortionType = "equidistant";
        calib.distortionCoefficients.resize(4);
        calib.distortionCoefficients << dist[0], dist[1], dist[2], dist[3];
      }
    }
    else{
      calib.distortionType = "radialtangential";
      calib.distortionCoefficients.resize(4);
      calib.distortionCoefficients << 0.0, 0.0, 0.0, 0.0;
    }

    std::vector<double> T_SC_cam_data = cam["T_SC"].as<std::vector<double>>();
    Eigen::Matrix4d T_SC_cam = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(T_SC_cam_data.data());
    calib.T_SC = kinematics::Transformation(T_SC_cam.cast<double>());

    calib.cameraType.isColour = cam["cam_type"].as<std::string>().find("gray") == std::string::npos;
    calib.cameraType.depthType.isDepthCamera = false;
    calib.cameraType.depthType.createDepth = false;
    calib.cameraType.depthType.createVirtual = false;
    calib.cameraType.isUsed = true;
    calib.cameraType.depthType.sigmaPixels = 0.0;
    calibrations.push_back(calib);

    return true;
    
}

}  // namespace okvis
