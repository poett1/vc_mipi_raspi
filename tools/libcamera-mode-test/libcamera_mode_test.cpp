// libcamera sensor-mode test for VC MIPI cameras.
//
// `cam` cannot choose the sensor bit depth (the Raspberry Pi pipeline rewrites raw
// stream formats to 16-bit before the final mode pass), so this tool selects the
// sensor mode through a SensorConfiguration and reports the delivered frame rate.
//
//   usage: libcamera_mode_test <bitdepth> <frames> [frame_duration_us|0] [out.pgm|-] [exposure_us]
//   build: g++ -std=c++20 -O1 libcamera_mode_test.cpp -o libcamera_mode_test $(pkg-config --cflags --libs libcamera)
//          (inside `meson devenv -C <libcamera build dir>` for an uninstalled libcamera)
//
// Saves the last frame as a 16-bit PGM when an output path is given. Frame rate is
// measured from SensorTimestamp; the last-20 figure is the steady-state value.
#include <libcamera/libcamera.h>
#include <sys/mman.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <thread>
#include <vector>
using namespace libcamera;
static std::atomic<int> done{0};
static std::vector<int64_t> ts;
static std::shared_ptr<Camera> cam;
static int nframes, fd_us, exp_us; static FrameBuffer *lastbuf;
static void setCtrls(Request *r){ if (fd_us>0){ int64_t v[2]={fd_us,fd_us}; r->controls().set(controls::FrameDurationLimits, Span<const int64_t,2>(v)); } if (exp_us>0){ r->controls().set(controls::ExposureTimeMode, controls::ExposureTimeModeManual); r->controls().set(controls::ExposureTime, exp_us); r->controls().set(controls::AnalogueGainMode, controls::AnalogueGainModeManual); r->controls().set(controls::AnalogueGain, 1.0f); } }
static void onDone(Request *r){
  if (r->status()==Request::RequestCancelled) return;
  auto t=r->metadata().get(controls::SensorTimestamp); ts.push_back(t?*t:0);
  lastbuf=r->buffers().begin()->second;
  if ((int)ts.size()>=nframes){ done=1; return; }
  r->reuse(Request::ReuseBuffers); setCtrls(r); cam->queueRequest(r);
}
int main(int argc,char**argv){
  int bd=atoi(argv[1]); nframes=atoi(argv[2]); fd_us=argc>3?atoi(argv[3]):0; const char*out=(argc>4 && argv[4][0]!='-')?argv[4]:nullptr; exp_us=argc>5?atoi(argv[5]):0;
  auto cm=std::make_unique<CameraManager>(); cm->start();
  cam=cm->cameras()[0]; cam->acquire();
  auto cfg=cam->generateConfiguration({StreamRole::Raw});
  auto &sc=cfg->at(0); sc.size={2048,1536}; sc.pixelFormat=formats::R16; sc.bufferCount=6;
  SensorConfiguration scfg; scfg.outputSize={2048,1536}; scfg.bitDepth=bd; cfg->sensorConfig=scfg;
  if (cfg->validate()==CameraConfiguration::Invalid){ fprintf(stderr,"invalid config\n"); return 1; }
  printf("stream: %s %s\n", sc.toString().c_str(), cfg->sensorConfig? "sensorConfig set":"no sensorConfig");
  if (cam->configure(cfg.get())){ fprintf(stderr,"configure failed\n"); return 1; }
  FrameBufferAllocator alloc(cam); alloc.allocate(sc.stream());
  std::vector<std::unique_ptr<Request>> reqs;
  for (auto &b: alloc.buffers(sc.stream())){ auto r=cam->createRequest(); r->addBuffer(sc.stream(), b.get()); setCtrls(r.get()); reqs.push_back(std::move(r)); }
  cam->requestCompleted.connect(onDone);
  cam->start(); for (auto &r: reqs) cam->queueRequest(r.get());
  auto t0=std::chrono::steady_clock::now();
  while(!done){ std::this_thread::sleep_for(std::chrono::milliseconds(20)); if (std::chrono::steady_clock::now()-t0>std::chrono::seconds(30)){ fprintf(stderr,"timeout after %zu frames\n", ts.size()); break; } }
  if (out && lastbuf){ auto &p=lastbuf->planes()[0]; void *m=mmap(nullptr,p.length,PROT_READ,MAP_SHARED,p.fd.get(),0); std::ofstream f(out,std::ios::binary); f<<"P5\n2048 1536\n65535\n"; auto *s=(const uint16_t*)((const char*)m+p.offset); for(size_t i=0;i<2048u*1536u;i++){ uint16_t v=s[i]; f.put(v>>8); f.put(v&0xff);} munmap(m,p.length); }
  cam->stop();
  int n=ts.size(); if(n>20){ double fps_all=(n-1)*1e9/(double)(ts[n-1]-ts[0]); double fps_tail=(19)*1e9/(double)(ts[n-1]-ts[n-20]); int64_t mn=1e12,mx=0; for(int i=1;i<n;i++){int64_t d=ts[i]-ts[i-1]; if(d<mn)mn=d; if(d>mx)mx=d;} printf("frames %d fps_all %.2f fps_last20 %.2f interval min %.3f ms max %.3f ms\n",n,fps_all,fps_tail,mn/1e6,mx/1e6); }
  cam->release(); cam.reset(); cm->stop(); return 0;
}
