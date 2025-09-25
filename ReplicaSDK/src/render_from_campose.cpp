// Copyright (c) Facebook, Inc. and its affiliates. All Rights Reserved

#include <EGL.h>
#include <PTexLib.h>
#include <pangolin/image/image_convert.h>
#include <fstream>
#include <experimental/filesystem>

#include "GLCheck.h"
#include "MirrorRenderer.h"

namespace fs = std::experimental::filesystem;

int main(int argc, char* argv[]) {
  ASSERT(argc == 3 || argc == 4, "Usage: ./ReplicaRenderer mesh.ply /path/to/atlases [mirrorFile]");

  const std::string meshFile(argv[1]);
  const std::string atlasFolder(argv[2]);
  ASSERT(pangolin::FileExists(meshFile));
  ASSERT(pangolin::FileExists(atlasFolder));

  std::ifstream camPoses("/home/robotlab/dataset/debug/camPose.txt"); //相机位姿
  if (!camPoses.is_open()) {
        std::cerr << "Failed to open camPose.txt" << std::endl;
        exit(0);
  }
  std::string line; std::getline(camPoses, line);  // 跳过第一行（注释行）
  std::string outFolder = "/home/robotlab/dataset/debug";     // 输出图片

  fs::create_directory(outFolder+"/rgb");
  fs::create_directory(outFolder+"/depth");
  
  
  std::string surfaceFile;
  if (argc == 4) {
    surfaceFile = std::string(argv[3]);
    ASSERT(pangolin::FileExists(surfaceFile));
  }

  const int width = 1280;  // FixMe
  const int height = 960;
  bool renderDepth = true;
  float depthScale = 65535.0f * 0.1f;

  // Setup EGL
  EGLCtx egl;

  egl.PrintInformation();
  
  if(!checkGLVersion()) {
    return 1;
  }

  //Don't draw backfaces
  const GLenum frontFace = GL_CCW;
  glFrontFace(frontFace);

  // Setup a framebuffer
  pangolin::GlTexture render(width, height);
  pangolin::GlRenderBuffer renderBuffer(width, height);
  pangolin::GlFramebuffer frameBuffer(render, renderBuffer);

  pangolin::GlTexture depthTexture(width, height, GL_R32F, false, 0, GL_RED, GL_FLOAT, 0);
  pangolin::GlFramebuffer depthFrameBuffer(depthTexture, renderBuffer);

  // Setup a camera
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrixRDF_BottomLeft(
          width,
          height,
          width / 2.0f,
          width / 2.0f,
          (width - 1.0f) / 2.0f,
          (height - 1.0f) / 2.0f,
          0.1f,
          100.0f),
      pangolin::ModelViewLookAtRDF(0, 0, 4, 0, 0, 0, 0, 1, 0));

  // // Start at some origin
  // Eigen::Matrix4d T_camera_world = s_cam.GetModelViewMatrix();

  // // And move to the left
  // Eigen::Matrix4d T_new_old = Eigen::Matrix4d::Identity();

  // T_new_old.topRightCorner(3, 1) = Eigen::Vector3d(0.025, 0, 0);

  // load mirrors
  std::vector<MirrorSurface> mirrors;
  if (surfaceFile.length()) {
    std::ifstream file(surfaceFile);
    picojson::value json;
    picojson::parse(json, file);

    for (size_t i = 0; i < json.size(); i++) {
      mirrors.emplace_back(json[i]);
    }
    std::cout << "Loaded " << mirrors.size() << " mirrors" << std::endl;
  }

  const std::string shadir = STR(SHADER_DIR);
  MirrorRenderer mirrorRenderer(mirrors, width, height, shadir);

  // load mesh and textures
  PTexMesh ptexMesh(meshFile, atlasFolder);

  pangolin::ManagedImage<Eigen::Matrix<uint8_t, 3, 1>> image(width, height);
  pangolin::ManagedImage<float> depthImage(width, height);
  pangolin::ManagedImage<uint16_t> depthImageInt(width, height);

  // Render some frames
  // const size_t numFrames = 100;
  // for (size_t i = 0; i < numFrames; i++) {
  std::string frameId;
  double m00, m01, m02, m03;
  double m10, m11, m12, m13;
  double m20, m21, m22, m23;
  double m30, m31, m32, m33;
  std::cout<<"Start rendering"<<std::endl;

  while (camPoses >> frameId >> m00 >> m01 >> m02 >> m03 >> m10 >> m11 >> m12 >> m13 >> m20 >> m21 >> m22 >> m23 >> m30 >> m31 >> m32 >> m33) {
    std::cout << "\rRendering frame" << frameId << std::endl;  //std::cout << "\rRendering frame " << i + 1 << "/" << numFrames << "... ";
    // std::cout.flush();
    Eigen::Matrix4d world2cam = Eigen::Matrix4d::Identity();
    world2cam(0, 0) = m00;
    world2cam(0, 1) = m01;
    world2cam(0, 2) = m02;
    world2cam(0, 3) = m03;
    world2cam(1, 0) = m10;
    world2cam(1, 1) = m11;
    world2cam(1, 2) = m12;
    world2cam(1, 3) = m13;
    world2cam(2, 0) = m20;
    world2cam(2, 1) = m21;
    world2cam(2, 2) = m22;
    world2cam(2, 3) = m23;
    world2cam(3, 0) = m30;
    world2cam(3, 1) = m31;
    world2cam(3, 2) = m32;
    world2cam(3, 3) = m33;
    
    Eigen::Matrix4d cam2world = world2cam.inverse();
    s_cam.GetModelViewMatrix() = cam2world;
    // Render
    frameBuffer.Bind();
    glPushAttrib(GL_VIEWPORT_BIT);
    glViewport(0, 0, width, height);
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    glEnable(GL_CULL_FACE);

    ptexMesh.Render(s_cam);

    glDisable(GL_CULL_FACE);

    glPopAttrib(); //GL_VIEWPORT_BIT
    frameBuffer.Unbind();

    for (size_t i = 0; i < mirrors.size(); i++) {
      MirrorSurface& mirror = mirrors[i];
      // capture reflections
      mirrorRenderer.CaptureReflection(mirror, ptexMesh, s_cam, frontFace);

      frameBuffer.Bind();
      glPushAttrib(GL_VIEWPORT_BIT);
      glViewport(0, 0, width, height);

      // render mirror
      mirrorRenderer.Render(mirror, mirrorRenderer.GetMaskTexture(i), s_cam);

      glPopAttrib(); //GL_VIEWPORT_BIT
      frameBuffer.Unbind();
    }

    // Download and save
    render.Download(image.ptr, GL_RGB, GL_UNSIGNED_BYTE);

    char filename[1000];
    snprintf(filename, 1000, "%s/rgb/%s.png", outFolder.c_str(), frameId.c_str());   // snprintf(filename, 1000, "frame%06zu.jpg", i);

    pangolin::SaveImage(
        image.UnsafeReinterpret<uint8_t>(),
        pangolin::PixelFormatFromString("RGB24"),
        std::string(filename));

    if (renderDepth) {
      // render depth
      depthFrameBuffer.Bind();
      glPushAttrib(GL_VIEWPORT_BIT);
      glViewport(0, 0, width, height);
      glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

      glEnable(GL_CULL_FACE);

      ptexMesh.RenderDepth(s_cam, depthScale);

      glDisable(GL_CULL_FACE);

      glPopAttrib(); //GL_VIEWPORT_BIT
      depthFrameBuffer.Unbind();

      depthTexture.Download(depthImage.ptr, GL_RED, GL_FLOAT);

      // convert to 16-bit int
      for(size_t i = 0; i < depthImage.Area(); i++)
          depthImageInt[i] = static_cast<uint16_t>(depthImage[i] + 0.5f);

      snprintf(filename, 1000, "%s/depth/%s.png", outFolder.c_str(), frameId.c_str());   //snprintf(filename, 1000, "depth%06zu.png", i);
      pangolin::SaveImage(
          depthImageInt.UnsafeReinterpret<uint8_t>(),
          pangolin::PixelFormatFromString("GRAY16LE"),
          std::string(filename), true, 34.0f);
    }

  }

  return 0;
}

