// Copyright (c) Facebook, Inc. and its affiliates. All Rights Reserved
// #include <EGL.h>
#include <PTexLib.h>
#include <fstream>
#include <string>
#include <iomanip>
#include <iostream>
#include <experimental/filesystem>

#include <pangolin/display/display.h>
#include <pangolin/display/widgets/widgets.h>
#include <pangolin/var/var.h>
#include <pangolin/var/varextra.h>
#include <pangolin/image/image_convert.h>
#include <pangolin/handler/handler.h>

#include "GLCheck.h"
#include "MirrorRenderer.h"

namespace fs = std::experimental::filesystem;

// code from ORB-SLAM2: 
// https://github.com/raulmur/ORB_SLAM2/blob/master/src/MapDrawer.cc#L179
// input: w2c cam pose
void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = 0.15;
    const float h = w*0.75;
    const float z = w*0.5;

    glPushMatrix();
    // Twc = Twc.Inverse();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(2);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

// Adapted from ewfuentes's fork.
// https://github.com/ewfuentes/Replica-Dataset/blob/main/ReplicaSDK/src/viewer.cpp#L28
struct FloatingHandler : pangolin::Handler {
  FloatingHandler(pangolin::OpenGlRenderState &render_state, std::ofstream& camPoseFile): 
    render_state(&render_state){
      mpCamPoseFile = &camPoseFile;
  }
  
  struct MouseOnState {
    Eigen::Vector3d start_axis;
    Eigen::Affine3d camera_from_world;
  };

  MouseOnState* last_mouse_on;
  std::ofstream* mpCamPoseFile;
  int frameCnt = 0;

  pangolin::OpenGlRenderState *render_state;
  // std::optional<std::filesystem::path> output_dir;

  void Keyboard(pangolin::View &, unsigned char key, int x, int y, bool pressed) {
    // const int key_int = static_cast<int>(key);
    if (!pressed) {
      return;
    }
    constexpr double STEP = 0.2;
    const Eigen::Translation3d new_camera_from_old_camera([&]() -> Eigen::Vector3d{
      // 相机是右手坐标系，x轴正向向右，y轴正向向上，z轴正向向后
      if (key == 'w') {
        return -Eigen::Vector3d::UnitZ() * STEP;
      } else if (key == 's') {
        return Eigen::Vector3d::UnitZ() * STEP;   // z轴正向，向后
      } else if (key == 'a') {
        return Eigen::Vector3d::UnitX() * STEP;  // x轴正向，向右
      } else if (key == 'd') {
        return -Eigen::Vector3d::UnitX() * STEP;
      } else if (key == 'e') {
        return Eigen::Vector3d::UnitY() * STEP;  // y轴正向，向上
      } else if (key == 'q') {
        return -Eigen::Vector3d::UnitY() * STEP;
      } else {
        return Eigen::Vector3d::Zero();
      }
    }());

    constexpr double ROT_ANGLE_RAD = 5.0 * M_PI / 180.0;
    const Eigen::Affine3d rotation_transform([&]() -> Eigen::Affine3d {
      switch (key) {
        case 'i':  // ↑ UP arrow key (X 轴旋转)
          return Eigen::Affine3d(Eigen::AngleAxisd(-ROT_ANGLE_RAD, Eigen::Vector3d::UnitX()));
        case 'k':  // ↓ DOWN arrow key
          return Eigen::Affine3d(Eigen::AngleAxisd(ROT_ANGLE_RAD, Eigen::Vector3d::UnitX()));
        case 'j':  // ← LEFT arrow key (Y 轴旋转)
          return Eigen::Affine3d(Eigen::AngleAxisd(ROT_ANGLE_RAD, Eigen::Vector3d::UnitY()));
        case 'l':  // → RIGHT arrow key
          return Eigen::Affine3d(Eigen::AngleAxisd(-ROT_ANGLE_RAD, Eigen::Vector3d::UnitY()));
        default:
          return Eigen::Affine3d::Identity();  // 不旋转
      }
    }());

    const Eigen::Affine3d new_camera_from_world = new_camera_from_old_camera * rotation_transform * static_cast<Eigen::Affine3d>(render_state->GetModelViewMatrix()) ;

    render_state->SetModelViewMatrix(new_camera_from_world);

    Eigen::Matrix4d world2camPose = render_state->GetModelViewMatrix().Inverse();  // world. to camera
    SaveCameraPose(world2camPose);
  }

  void Mouse(pangolin::View &view, const pangolin::MouseButton button, const int x, const int y, const bool pressed, int button_state) {
    Eigen::Vector3d pt_in_cam = Eigen::Vector3d::Zero();
    view.GetCamCoordinates(*render_state, x, y, 0.1, pt_in_cam.x(), pt_in_cam.y(), pt_in_cam.z());
    if (!pressed && ((button_state & 0x01) == 0)) {
      // delete last_mouse_on;
      last_mouse_on = nullptr;
    }
    else if (button == pangolin::MouseButton::MouseButtonLeft) {
      last_mouse_on = new MouseOnState{
        pt_in_cam.normalized(),
        static_cast<Eigen::Affine3d>(render_state->GetModelViewMatrix())
      };
    }

  }

  void MouseMotion(pangolin::View &view, const int x, const int y, int button_state) {
    if (last_mouse_on) {
      Eigen::Vector3d pt_in_cam = Eigen::Vector3d::Zero();
      view.GetCamCoordinates(*render_state, x, y, 0.1, pt_in_cam.x(), pt_in_cam.y(), pt_in_cam.z());
      Eigen::Vector3d end_axis = pt_in_cam.normalized();

      const Eigen::Vector3d cross = last_mouse_on->start_axis.cross(end_axis);
      const double angle_rad = std::asin(cross.norm());
      const Eigen::Vector3d axis_in_camera = cross.normalized();
      if (angle_rad < 1e-3) {
        return;
      }

      const Eigen::AngleAxis<double> new_camera_from_old_camera(angle_rad, axis_in_camera);

      const Eigen::Affine3d new_camera_from_world = 
        new_camera_from_old_camera * last_mouse_on->camera_from_world;
      render_state->SetModelViewMatrix(new_camera_from_world);
    }

  }

  void SaveCameraPose(Eigen::Matrix4d& world2camPose){
        *mpCamPoseFile << std::setfill('0') << std::setw(6) << frameCnt++ << " ";
        *mpCamPoseFile << world2camPose(0, 0) << " " << world2camPose(0, 1) << " " << world2camPose(0, 2) << " " << world2camPose(0, 3) << " "
                    << world2camPose(1, 0) << " " << world2camPose(1, 1) << " " << world2camPose(1, 2) << " " << world2camPose(1, 3) << " "
                    << world2camPose(2, 0) << " " << world2camPose(2, 1) << " " << world2camPose(2, 2) << " " << world2camPose(2, 3) << " "
                    << world2camPose(3, 0) << " " << world2camPose(3, 1) << " " << world2camPose(3, 2) << " " << world2camPose(3, 3) << std::endl;
  }
};


int main(int argc, char* argv[]) {

  ASSERT(argc == 4 || argc == 5, "Usage: ./ReplicaCapturer mesh.ply textures [glass.sur] outFolder");

  const std::string meshFile(argv[1]);
  const std::string atlasFolder(argv[2]);
  ASSERT(pangolin::FileExists(meshFile));
  ASSERT(pangolin::FileExists(atlasFolder));

  std::string surfaceFile;
  std::string outFolder;
  if (argc == 5) {
    surfaceFile = std::string(argv[3]);
    ASSERT(pangolin::FileExists(surfaceFile));
    outFolder = std::string(argv[4]);
  }
  else {
    outFolder = std::string(argv[3]);
  }

  if (!fs::exists(outFolder)) {
    fs::create_directory(outFolder);
    fs::create_directory(outFolder+"/rgb");
    fs::create_directory(outFolder+"/depth");
  }

  const int uiWidth = 180;
  const int width = 1280;
  const int height = 960;
  bool renderDepth = true;
  // float depthScale = 65535.0f * 0.1f;
  
  // Setup OpenGL Display (based on GLUT)
  pangolin::CreateWindowAndBind("ReplicaViewer", uiWidth + width, height);

  // debug
  // std::cout << "DEBUG: CreateWindowAndBind done" << std::endl;

  if (glewInit() != GLEW_OK) {
    pango_print_error("Unable to initialize GLEW.");
  }

  if(!checkGLVersion()) {
    return 1;
  }

  // Setup default OpenGL parameters
  glEnable(GL_DEPTH_TEST);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  const GLenum frontFace = GL_CW;
  glFrontFace(frontFace);
  glLineWidth(1.0f);

  //Don't draw backfaces
  // const GLenum frontFace = GL_CCW;
  // glFrontFace(frontFace);

  // debug
  // std::cout << "DEBUG: setup default OpenGL parameters done" << std::endl;

  // Setup a framebuffer
  pangolin::GlTexture render(width, height);
  pangolin::GlRenderBuffer renderBuffer(width, height);
  pangolin::GlFramebuffer frameBuffer(render, renderBuffer);

  pangolin::GlTexture depthTexture(width, height, GL_R32F, false, 0, GL_RED, GL_FLOAT, 0);
  pangolin::GlFramebuffer depthFrameBuffer(depthTexture, renderBuffer);

  // debug
  // std::cout << "DEGUB: setup framebuffer done" << std::endl;

  // Tell the base view to arrange its children equally
  if (uiWidth != 0) {
    pangolin::CreatePanel("ui").SetBounds(0, 1.0f, 0, pangolin::Attach::Pix(uiWidth));
  }

  pangolin::View& container =
      pangolin::CreateDisplay().SetBounds(0, 1.0f, pangolin::Attach::Pix(uiWidth), 1.0f);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrixRDF_TopLeft(  // right hand coordinate
          width,
          height,
          width / 2.0f,
          width / 2.0f,
          width / 2.0f,
          height / 2.0f,
          0.1f,
          100.0f),
      pangolin::ModelViewLookAtRDF(0, 0, 0, 1, 0, 0, 0, 0, 1));
  
  
  // 3. 自定义交互控制器
  std::cout << "Cam Pose(W2C):\n" << s_cam.GetModelViewMatrix().Inverse() << std::endl;
  // 用于存储相机位姿的文件
  // load cam pose write file
  std::ofstream camPoseFile;
  camPoseFile.open(outFolder + "/camPose.txt");
  if (!camPoseFile.is_open()) {
    std::cout << "Failed to open cam pose file: " << outFolder + "/camPose.txt" << std::endl;
    return 1;
  }
  else{
    // write comments: # frameId world2camPose
    camPoseFile << "# frameId World2CamPose(4x4 matrix, row order)" << std::endl;
  }
  int frameCnt = 0;
  FloatingHandler s_handler(s_cam,camPoseFile);

  pangolin::View& meshView = pangolin::Display("MeshView")
                                 .SetBounds(0, 1.0f, 0, 1.0f, (double)width / (double)height)
                                 .SetHandler(&s_handler);

  container.AddDisplay(meshView);

  std::cout << "DEBUG: setup meshView done" << std::endl;

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

  std::cout << "DEBUG: setup mirrorRenderer done" << std::endl;



  // 4. 加载场景网格和镜面
  // load mesh and textures
  PTexMesh ptexMesh(meshFile, atlasFolder);
  std::cout << "DEBUG: setup ptexMesh done" << std::endl;

  // 5. UI 控件面板
  pangolin::ManagedImage<Eigen::Matrix<uint8_t, 3, 1>> image(width, height);
  pangolin::ManagedImage<float> depthImage(width, height);
  pangolin::ManagedImage<uint16_t> depthImageInt(width, height);

  pangolin::Var<float> exposure("ui.Exposure", 0.01, 0.0f, 0.1f);
  pangolin::Var<float> gamma("ui.Gamma", ptexMesh.Gamma(), 1.0f, 3.0f);
  pangolin::Var<float> saturation("ui.Saturation", ptexMesh.Saturation(), 0.0f, 2.0f);
  pangolin::Var<float> depthScale("ui.Depth_scale", 0.1f, 0.0f, 1.0f);

  pangolin::Var<bool> wireframe("ui.Wireframe", false, true);
  pangolin::Var<bool> drawBackfaces("ui.Draw_backfaces", false, true);
  pangolin::Var<bool> drawMirrors("ui.Draw_mirrors", true, true);
  pangolin::Var<bool> drawDepth("ui.Draw_depth", false, true);
  pangolin::Var<bool> captureBtn("ui.Capture", false, false);

  ptexMesh.SetExposure(exposure);
  std::cout << "DEBUG: setup pangolin done" << std::endl;


  // 6. 主渲染循环
  while (!pangolin::ShouldQuit()) {
    
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);


    if (exposure.GuiChanged()) {
      ptexMesh.SetExposure(exposure);
    }

    if (gamma.GuiChanged()) {
      ptexMesh.SetGamma(gamma);
    }

    if (saturation.GuiChanged()) {
      ptexMesh.SetSaturation(saturation);
    }

    if (meshView.IsShown()) {
      meshView.Activate(s_cam);

      if (drawBackfaces) {
        glDisable(GL_CULL_FACE);
      } else {
        glEnable(GL_CULL_FACE);
      }

      if (wireframe) {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0, 1.0);
        ptexMesh.Render(s_cam);
        glDisable(GL_POLYGON_OFFSET_FILL);
        // render wireframe on top
        ptexMesh.RenderWireframe(s_cam);
      } else if (drawDepth) {
        ptexMesh.RenderDepth(s_cam, depthScale);
      } else {
        ptexMesh.Render(s_cam);
      }


      glDisable(GL_CULL_FACE);

      if (drawMirrors) {
        for (size_t i = 0; i < mirrors.size(); i++) {
          MirrorSurface& mirror = mirrors[i];
          // capture reflections
          mirrorRenderer.CaptureReflection(mirror, ptexMesh, s_cam, frontFace, drawDepth, depthScale);

          // render mirror
          mirrorRenderer.Render(mirror, mirrorRenderer.GetMaskTexture(i), s_cam, drawDepth);
        }
      }
    }

    pangolin::FinishFrame();
  }

  return 0;
}
