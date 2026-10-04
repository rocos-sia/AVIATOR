#include "camera.hpp"
#include <EGL/eglext.h>
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace simulation {
Camera::Camera(Simulation& simulation)
    : width_(simulation.cameraWidth()), height_(simulation.cameraHeight()), rgb_(width_*height_*3) {
    try {
        // Mesa surfaceless EGL works on Linux without a desktop/X server.
        auto platform = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
        if (platform) display_ = platform(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
        if (display_ == EGL_NO_DISPLAY || !eglInitialize(display_, nullptr, nullptr)) {
            display_ = eglGetDisplay(EGL_DEFAULT_DISPLAY);
            if (display_ == EGL_NO_DISPLAY || !eglInitialize(display_, nullptr, nullptr))
                throw std::runtime_error("EGL initialization failed; use --no-camera for physics only");
        }
        if (!eglBindAPI(EGL_OPENGL_API)) throw std::runtime_error("EGL OpenGL unavailable");
        const EGLint attributes[] = {EGL_SURFACE_TYPE,EGL_PBUFFER_BIT,EGL_RENDERABLE_TYPE,EGL_OPENGL_BIT,
            EGL_RED_SIZE,8,EGL_GREEN_SIZE,8,EGL_BLUE_SIZE,8,EGL_DEPTH_SIZE,24,EGL_NONE};
        EGLConfig config; EGLint count=0;
        if (!eglChooseConfig(display_,attributes,&config,1,&count) || count != 1) throw std::runtime_error("EGL config unavailable");
        const EGLint size[] = {EGL_WIDTH,width_,EGL_HEIGHT,height_,EGL_NONE};
        surface_ = eglCreatePbufferSurface(display_,config,size);
        context_ = eglCreateContext(display_,config,EGL_NO_CONTEXT,nullptr);
        if (surface_ == EGL_NO_SURFACE || context_ == EGL_NO_CONTEXT || !eglMakeCurrent(display_,surface_,surface_,context_))
            throw std::runtime_error("EGL context creation failed");
        mjv_defaultScene(&scene_); mjv_defaultCamera(&camera_); mjv_defaultOption(&option_); mjr_defaultContext(&render_);
        auto* m = simulation.model();
        snapshot_.reset(mj_makeData(m));
        if (!snapshot_) throw std::runtime_error("Camera snapshot allocation failed");
        m->vis.global.offwidth=width_; m->vis.global.offheight=height_;
        mjv_makeScene(m,&scene_,4000); mjr_makeContext(m,&render_,mjFONTSCALE_100);
        mjr_setBuffer(mjFB_OFFSCREEN,&render_);
        if (render_.currentBuffer != mjFB_OFFSCREEN) throw std::runtime_error("offscreen framebuffer unavailable");
        int left=mj_name2id(m,mjOBJ_SITE,"left_handle"), right=mj_name2id(m,mjOBJ_SITE,"right_handle");
        for (int i=0;i<3;++i) camera_.lookat[i]=(simulation.data()->site_xpos[3*left+i]+simulation.data()->site_xpos[3*right+i])/2;
        camera_.distance=1.5; camera_.azimuth=90; camera_.elevation=-15;
        // Models with a tag provide a fixed pilot-side view; retain the legacy
        // free camera for older/custom models that do not define this camera.
        const int tag_camera = mj_name2id(m, mjOBJ_CAMERA, "cockpit_apriltag");
        if (tag_camera >= 0) {
            camera_.type = mjCAMERA_FIXED;
            camera_.fixedcamid = tag_camera;
            // D436 optical centers are inside the case. Group 1 is its own
            // opaque CAD housing, still visible in the independent viewer.
            if (mj_name2id(m, mjOBJ_BODY, "realsense_d436_link") >= 0)
                option_.geomgroup[1] = 0;
        }
        // EGL and GLFW/GLX must never both own this thread's current context.
        eglMakeCurrent(display_,EGL_NO_SURFACE,EGL_NO_SURFACE,EGL_NO_CONTEXT);
    } catch (...) { cleanup(); throw; }
}
void Camera::cleanup() {
    if (display_ != EGL_NO_DISPLAY) {
        if (context_ != EGL_NO_CONTEXT) {
            eglMakeCurrent(display_,surface_,surface_,context_);
            mjr_freeContext(&render_); mjv_freeScene(&scene_);
        }
        eglMakeCurrent(display_,EGL_NO_SURFACE,EGL_NO_SURFACE,EGL_NO_CONTEXT);
        if (context_ != EGL_NO_CONTEXT) eglDestroyContext(display_,context_);
        if (surface_ != EGL_NO_SURFACE) eglDestroySurface(display_,surface_);
        eglTerminate(display_);
    }
}
Camera::~Camera() { cleanup(); }
aviator::Message Camera::detection(bool in_roi) const {
    auto result = detection_;
    result.header.timestamp = aviator::utc_us();
    if (!in_roi) {
        result.header.valid = false;
        result.body["status"] = "SEARCHING";
        result.body["confidence"] = 0.0;
        result.body["pose"] = nullptr;
        result.body["steering_wheel"].update({{"valid", false}, {"reason", "target_not_tracking"},
            {"theta_rad", nullptr}, {"translation_along_axis_m", nullptr}});
        result.body["yoke"] = {{"detected", false}, {"roll", nullptr}, {"pitch", nullptr}};
        result.body.erase("tag_id");
        result.body.erase("target_frame");
    }
    return result;
}
bool Camera::capture(Simulation& simulation, std::mutex* physics_mutex) {
    Json command;
    {
        std::unique_lock<std::mutex> lock;
        if (physics_mutex) lock = std::unique_lock<std::mutex>(*physics_mutex);
        mj_copyData(snapshot_.get(), simulation.model(), simulation.data());
        command = simulation.camera_command();
        sample_time_ = aviator::monotonic_us();
        // Snapshot the accepted command/reference and its watchdog at acquisition,
        // too: commands arriving during rendering must not invalidate an older frame.
        detection_ = simulation.detection(sample_time_, true, true, snapshot_.get());
    }
    if (!eglMakeCurrent(display_,surface_,surface_,context_)) throw std::runtime_error("EGL make current failed");
    struct ReleaseContext {
        EGLDisplay display;
        ~ReleaseContext() { eglMakeCurrent(display,EGL_NO_SURFACE,EGL_NO_SURFACE,EGL_NO_CONTEXT); }
    } release{display_};
    mjv_updateScene(simulation.model(),snapshot_.get(),&option_,nullptr,&camera_,mjCAT_ALL,&scene_);
    mjr_render({0,0,width_,height_},&scene_,&render_);
    mjr_readPixels(rgb_.data(),nullptr,{0,0,width_,height_},&render_);
    // OpenGL reads bottom-up. All consumers (window, JPEG, Logger) receive RGB8
    // with the same top-left origin as the real camera node.
    const auto stride = width_ * 3;
    for (int y = 0; y < height_/2; ++y)
        std::swap_ranges(rgb_.begin() + y*stride, rgb_.begin() + (y+1)*stride,
                         rgb_.begin() + (height_-1-y)*stride);
    const auto lens = mjv_averageCamera(&scene_.camera[0], &scene_.camera[1]);
    const double fh = lens.frustum_top - lens.frustum_bottom;
    const double fw = lens.frustum_width > 0 ? 2*lens.frustum_width : fh*width_/height_;
    calibration_ = {{"rgb", {{"serial", "simulation"}, {"width", width_}, {"height", height_},
        {"fx", width_*lens.frustum_near/fw}, {"fy", height_*lens.frustum_near/fh},
        {"ppx", width_*(.5-lens.frustum_center/fw)}, {"ppy", height_*lens.frustum_top/fh},
        {"distortion_model", "none"}, {"coeffs", {0,0,0,0,0}},
        {"to_color_rotation", {1,0,0,0,1,0,0,0,1}}, {"to_color_translation_m", {0,0,0}}}}};
    // Ground-truth candidate center projected into the original image coordinates.
    // This sensor deliberately does not claim image-based recognition/occlusion handling.
    mjvGLCamera c = mjv_averageCamera(&scene_.camera[0],&scene_.camera[1]);
    int left=mj_name2id(simulation.model(),mjOBJ_SITE,"left_handle");
    int right=mj_name2id(simulation.model(),mjOBJ_SITE,"right_handle");
    double offset[3], horizontal[3];
    const int tag = mj_name2id(simulation.model(),mjOBJ_SITE,"yoke_apriltag_center");
    for (int i=0;i<3;++i) {
        const double target = tag >= 0 ? snapshot_->site_xpos[3*tag+i] :
            (snapshot_->site_xpos[3*left+i]+snapshot_->site_xpos[3*right+i])/2;
        offset[i] = target - c.pos[i];
    }
    horizontal[0]=c.forward[1]*c.up[2]-c.forward[2]*c.up[1];
    horizontal[1]=c.forward[2]*c.up[0]-c.forward[0]*c.up[2];
    horizontal[2]=c.forward[0]*c.up[1]-c.forward[1]*c.up[0];
    double depth=0,x=0,y=0;
    for(int i=0;i<3;++i) { depth+=offset[i]*c.forward[i]; x+=offset[i]*horizontal[i]; y+=offset[i]*c.up[i]; }
    if (depth <= c.frustum_near || depth >= c.frustum_far) return false;
    const double height=c.frustum_top-c.frustum_bottom;
    // Explicit fx/fy intrinsics can produce a non-square-pixel frustum.
    const double width=c.frustum_width > 0 ? 2*c.frustum_width : height*width_/height_;
    x=width_*(.5+(x*c.frustum_near/depth-c.frustum_center)/width);
    y=height_*(1-(y*c.frustum_near/depth-c.frustum_bottom)/height);
    if (!std::isfinite(x) || !std::isfinite(y) || x<0 || x>=width_ || y<0 || y>=height_) return false;
    if (!command.contains("roi") || command.at("roi").is_null()) return true;
    const auto& roi=command.at("roi");
    double rx=roi.at("x"), ry=roi.at("y"), rw=roi.at("width"), rh=roi.at("height");
    return x>=rx && x<rx+rw && y>=ry && y<ry+rh;
}
}
