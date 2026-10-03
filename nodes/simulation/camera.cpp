#include "camera.hpp"
#include <EGL/eglext.h>
#include <cmath>
#include <stdexcept>

namespace simulation {
Camera::Camera(Simulation& simulation) : rgb_(640*480*3) {
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
        const EGLint size[] = {EGL_WIDTH,640,EGL_HEIGHT,480,EGL_NONE};
        surface_ = eglCreatePbufferSurface(display_,config,size);
        context_ = eglCreateContext(display_,config,EGL_NO_CONTEXT,nullptr);
        if (surface_ == EGL_NO_SURFACE || context_ == EGL_NO_CONTEXT || !eglMakeCurrent(display_,surface_,surface_,context_))
            throw std::runtime_error("EGL context creation failed");
        mjv_defaultScene(&scene_); mjv_defaultCamera(&camera_); mjv_defaultOption(&option_); mjr_defaultContext(&render_);
        auto* m = simulation.model();
        snapshot_.reset(mj_makeData(m));
        if (!snapshot_) throw std::runtime_error("Camera snapshot allocation failed");
        m->vis.global.offwidth=640; m->vis.global.offheight=480;
        mjv_makeScene(m,&scene_,4000); mjr_makeContext(m,&render_,mjFONTSCALE_100);
        mjr_setBuffer(mjFB_OFFSCREEN,&render_);
        if (render_.currentBuffer != mjFB_OFFSCREEN) throw std::runtime_error("offscreen framebuffer unavailable");
        int left=mj_name2id(m,mjOBJ_SITE,"left_handle"), right=mj_name2id(m,mjOBJ_SITE,"right_handle");
        for (int i=0;i<3;++i) camera_.lookat[i]=(simulation.data()->site_xpos[3*left+i]+simulation.data()->site_xpos[3*right+i])/2;
        camera_.distance=1.5; camera_.azimuth=90; camera_.elevation=-15;
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
bool Camera::capture(Simulation& simulation, std::mutex* physics_mutex) {
    Json command;
    {
        std::unique_lock<std::mutex> lock;
        if (physics_mutex) lock = std::unique_lock<std::mutex>(*physics_mutex);
        mj_copyData(snapshot_.get(), simulation.model(), simulation.data());
        command = simulation.camera_command();
    }
    if (!eglMakeCurrent(display_,surface_,surface_,context_)) throw std::runtime_error("EGL make current failed");
    struct ReleaseContext {
        EGLDisplay display;
        ~ReleaseContext() { eglMakeCurrent(display,EGL_NO_SURFACE,EGL_NO_SURFACE,EGL_NO_CONTEXT); }
    } release{display_};
    mjv_updateScene(simulation.model(),snapshot_.get(),&option_,nullptr,&camera_,mjCAT_ALL,&scene_);
    mjr_render({0,0,640,480},&scene_,&render_);
    mjr_readPixels(rgb_.data(),nullptr,{0,0,640,480},&render_);
    // Ground-truth candidate center projected into the original image coordinates.
    // This sensor deliberately does not claim image-based recognition/occlusion handling.
    mjvGLCamera c = mjv_averageCamera(&scene_.camera[0],&scene_.camera[1]);
    int left=mj_name2id(simulation.model(),mjOBJ_SITE,"left_handle");
    int right=mj_name2id(simulation.model(),mjOBJ_SITE,"right_handle");
    double offset[3], horizontal[3];
    for (int i=0;i<3;++i) offset[i]=(snapshot_->site_xpos[3*left+i]+snapshot_->site_xpos[3*right+i])/2-c.pos[i];
    horizontal[0]=c.forward[1]*c.up[2]-c.forward[2]*c.up[1];
    horizontal[1]=c.forward[2]*c.up[0]-c.forward[0]*c.up[2];
    horizontal[2]=c.forward[0]*c.up[1]-c.forward[1]*c.up[0];
    double depth=0,x=0,y=0;
    for(int i=0;i<3;++i) { depth+=offset[i]*c.forward[i]; x+=offset[i]*horizontal[i]; y+=offset[i]*c.up[i]; }
    if (depth <= c.frustum_near || depth >= c.frustum_far) return false;
    const double height=c.frustum_top-c.frustum_bottom, width=height*640.0/480;
    x=640*(.5+(x*c.frustum_near/depth-c.frustum_center)/width);
    y=480*(1-(y*c.frustum_near/depth-c.frustum_bottom)/height);
    if (!std::isfinite(x) || !std::isfinite(y) || x<0 || x>=640 || y<0 || y>=480) return false;
    if (!command.contains("roi") || command.at("roi").is_null()) return true;
    const auto& roi=command.at("roi");
    double rx=roi.at("x"), ry=roi.at("y"), rw=roi.at("width"), rh=roi.at("height");
    return x>=rx && x<rx+rw && y>=ry && y<ry+rh;
}
}
