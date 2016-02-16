/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libelas.
Authors: Andreas Geiger

libelas is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 3 of the License, or any later version.

libelas is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libelas; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA
*/

// Demo program showing how libelas can be used, try "./elas -h" for help

#include <iostream>

#include <Eigen/Core>
#include <HAL/Utils/GetPot>
#include <HAL/Camera/CameraDevice.h>
#include <HAL/Camera/Drivers/Rectify/RectifyDriver.h>
#include <calibu/Calibu.h>
#include <kangaroo/ImageIntrinsics.h>
#include <kangaroo/kangaroo.h>

#include "elas.h"
#include "image.h"

using namespace std;



inline Sophus::SE3d T_rlFromCamModelRDF(const calibu::CameraModelAndTransform& lcmod, const calibu::CameraModelAndTransform& rcmod, const Eigen::Matrix3d& targetRDF)
{
  // Transformation matrix to adjust to target RDF
  Eigen::Matrix4d Tadj[2] = {Eigen::Matrix4d::Identity(),Eigen::Matrix4d::Identity()};
  Tadj[0].block<3,3>(0,0) = targetRDF.transpose() * lcmod.camera.RDF();
  Tadj[1].block<3,3>(0,0) = targetRDF.transpose() * rcmod.camera.RDF();

  // Computer Poses in our adjust coordinate system
  const Eigen::Matrix4d T_lw_ = Tadj[0] * lcmod.T_wc.matrix().inverse();
  const Eigen::Matrix4d T_rw_ = Tadj[1] * rcmod.T_wc.matrix().inverse();

  // Computer transformation to right camera frame from left
  const Eigen::Matrix4d T_rl = T_rw_ * T_lw_.inverse();

  return Sophus::SE3d(T_rl.block<3,3>(0,0), T_rl.block<3,1>(0,3) );
}


// compute disparities of pgm image input pair file_1, file_2
void process (const char* file_1,const char* file_2) {


  // load images
  image<uchar> *I1,*I2;
  I1 = loadPGM(file_1);
  I2 = loadPGM(file_2);

  // check for correct size
  if (I1->width()<=0 || I1->height() <=0 || I2->width()<=0 || I2->height() <=0 ||
      I1->width()!=I2->width() || I1->height()!=I2->height()) {
    cout << "ERROR: Images must be of same size, but" << endl;
    cout << "       I1: " << I1->width() <<  " x " << I1->height() <<
            ", I2: " << I2->width() <<  " x " << I2->height() << endl;
    delete I1;
    delete I2;
    return;
  }

  // get image width and height
  int32_t width  = I1->width();
  int32_t height = I1->height();

  // allocate memory for disparity images
  const int32_t dims[3] = {width,height,width}; // bytes per line = width
  float* D1_data = (float*)malloc(width*height*sizeof(float));
  float* D2_data = (float*)malloc(width*height*sizeof(float));

  // process
  Elas::parameters param;
  param.postprocess_only_left = false;
  Elas elas(param);
  elas.process(I1->data,I2->data,D1_data,D2_data,dims);

  // find maximum disparity for scaling output disparity images to [0..255]
  float disp_max = 0;
  for (int32_t i=0; i<width*height; i++) {
    if (D1_data[i]>disp_max) disp_max = D1_data[i];
    if (D2_data[i]>disp_max) disp_max = D2_data[i];
  }

  // copy float to uchar
  image<uchar> *D1 = new image<uchar>(width,height);
  image<uchar> *D2 = new image<uchar>(width,height);
  for (int32_t i=0; i<width*height; i++) {
    D1->data[i] = (uint8_t)max(255.0*D1_data[i]/disp_max,0.0);
    D2->data[i] = (uint8_t)max(255.0*D2_data[i]/disp_max,0.0);
  }

  // save disparity images
  char output_1[1024];
  char output_2[1024];
  strncpy(output_1,file_1,strlen(file_1)-4);
  strncpy(output_2,file_2,strlen(file_2)-4);
  output_1[strlen(file_1)-4] = '\0';
  output_2[strlen(file_2)-4] = '\0';
  strcat(output_1,"_disp.pgm");
  strcat(output_2,"_disp.pgm");
  savePGM(D1,output_1);
  savePGM(D2,output_2);

  // free memory
  delete I1;
  delete I2;
  delete D1;
  delete D2;
  free(D1_data);
  free(D2_data);
}

int main (int argc, char** argv) {

  GetPot clArgs( argc, argv );

  int skip_frames   = clArgs.follow(0, "-skip_frames");
  bool export_time  = clArgs.search("-export_time");

  // Setup camera.
  hal::Camera camera = hal::Camera(clArgs.follow("", "-cam"));

  const unsigned int width = camera.Width();
  const unsigned int height = camera.Height();


  // Check if camera is being rectified. If so, use that camera model.
  // Otherwise, load a camera model from file.
  // Load Camera intrinsics from file.
  calibu::CameraRig rig;
  hal::RectifyDriver *driver_ptr = camera.GetDriver<hal::RectifyDriver>();
  if (driver_ptr) {
    std::cerr << "Rectified driver detected. Extracting new camera model." << std::endl;
    calibu::CameraRig driver_rig;
    // get camera models from driver
    calibu::CameraModelGeneric<double> cmod;
    cmod = driver_ptr->CameraModel();
    driver_rig.Add(cmod, Sophus::SE3Group<double>());
    driver_rig.Add(cmod, driver_ptr->T_rl().inverse());
    rig = driver_rig;
  } else {
    const std::string filename = clArgs.follow("","-cmod");
    if( filename.empty() ) {
      std::cerr << "Camera models file is required!" << std::endl;
      exit(EXIT_FAILURE);
    }
    rig = calibu::ReadXmlRig(filename);
  }



  if (rig.cameras.size() != 2) {
    std::cerr << "Two camera models are required to run this program!" << std::endl;
    exit(1);
  }

  Eigen::Matrix3f CamModel0 = rig.cameras[0].camera.K().cast<float>();
  Eigen::Matrix3f CamModel1 = rig.cameras[1].camera.K().cast<float>();

  roo::ImageIntrinsics camMod[] = {
    {CamModel0(0,0),CamModel0(1,1),CamModel0(0,2),CamModel0(1,2)},
    {CamModel1(0,0),CamModel1(1,1),CamModel1(0,2),CamModel1(1,2)}
  };

  const Eigen::Matrix3d& Kl = camMod[0][0].Matrix();

  // print selected camera model
  std::cout << "Camera Model used: " << std::endl << camMod[0][0].Matrix() << std::endl;

  Eigen::Matrix3d RDFvision;RDFvision<< 1,0,0,  0,1,0,  0,0,1;
  Eigen::Matrix3d RDFrobot; RDFrobot << 0,1,0,  0,0, 1,  1,0,0;
  Eigen::Matrix4d T_vis_ro = Eigen::Matrix4d::Identity();
  T_vis_ro.block<3,3>(0,0) = RDFvision.transpose() * RDFrobot;
  Eigen::Matrix4d T_ro_vis = Eigen::Matrix4d::Identity();
  T_ro_vis.block<3,3>(0,0) = RDFrobot.transpose() * RDFvision;

  const Sophus::SE3d T_rl = T_rlFromCamModelRDF(rig.cameras[0], rig.cameras[1], RDFvision);
  const double baseline = T_rl.translation().norm();

  std::cout << "Baseline is: " << baseline << std::endl;

  roo::Image<float, roo::TargetDevice, roo::Manage> dDisparity(width, height);
  roo::Image<float, roo::TargetDevice, roo::Manage> dDepth(width, height);
  cv::Mat hDisparity1 = cv::Mat(height, width, CV_32FC1);
  cv::Mat hDisparity2 = cv::Mat(height, width, CV_32FC1);
  cv::Mat hDepth = cv::Mat(height, width, CV_32FC1);

  std::shared_ptr<pb::ImageArray> images = pb::ImageArray::Create();

  // ELAS image format
  image<uchar> *I1 = new image<uchar>(width, height);
  image<uchar> *I2 = new image<uchar>(width, height);

  // allocate memory for disparity images
  int32_t dims[3];
  dims[0] = width; // bytes per line = width
  dims[1] = height; // bytes per line = width
  dims[2] = width; // bytes per line = width

  // set up ELAS process
  Elas::parameters param;
  param.postprocess_only_left = false;
  Elas elas(param);

  cout << "Processing ... " << endl;

  for(size_t frame = 0;; ++frame) {

    for (int ii = 0; ii < skip_frames; ++ii) {
      camera.Capture(*images);
    }
    if (camera.Capture(*images) == false ) {
      break;
    }


    ///----- Main ELAS Code.

    // repack images
    memcpy(imPtr(I1, 0, 0), images->at(0)->data(), width * height);
    memcpy(imPtr(I2, 0, 0), images->at(1)->data(), width * height);

    // process
    elas.process(I1->data,I2->data,(float*)hDisparity1.data,(float*)hDisparity2.data,dims);

    // upload disparity to GPU
    dDisparity.MemcpyFromHost((float*)hDisparity1.data);

    // convert disparity to depth
    roo::Disp2Depth(dDisparity, dDepth, Kl(0,0), baseline);

    // download depth from GPU
    dDepth.MemcpyToHost(hDepth.data);

    // save depth image
    const double    timestamp = images->SystemTime();
    int             frame_i = static_cast<int>(frame);
    char            index[30];

    if (export_time) {
      sprintf(index, "%015.10f", timestamp);
    } else {
      sprintf(index, "%05d", frame_i);
    }
    std::string DepthPrefix = "ELAS-";
    std::string DepthFile;
    DepthFile = DepthPrefix + index + ".pdm";
    std::cout << "Depth File: " << DepthFile << std::endl;
    std::ofstream pDFile( DepthFile.c_str(), std::ios::out | std::ios::binary );
    pDFile << "P7" << std::endl;
    pDFile << hDepth.cols << " " << hDepth.rows << std::endl;
    unsigned int Size = hDepth.elemSize1() * hDepth.rows * hDepth.cols;
    pDFile << 4294967295 << std::endl;
    pDFile.write((const char*)hDepth.data, Size);
    pDFile.close();

    // save grey image
    std::string GreyPrefix = "Grey-";
    std::string GreyFile;
    GreyFile = GreyPrefix + index + ".pgm";
    std::cout << "Grey File: " << GreyFile << std::endl;
    cv::imwrite( GreyFile, images->at(0)->Mat() );
  }

  cout << "... done!" << endl;

  return 0;
}
