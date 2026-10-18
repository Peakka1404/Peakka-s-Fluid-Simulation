#include <chrono>
#include <cstdio>
#include <algorithm>
#include <numeric>
#include "../src/sph_solver.h"
#include "../src/euler_solver.h"
static void stats(const char* tag, const std::vector<float>& v){
  float mn=*std::min_element(v.begin(),v.end()), mx=*std::max_element(v.begin(),v.end());
  double s=std::accumulate(v.begin(),v.end(),0.0)/v.size();
  printf("   %-9s min %9.2f  mean %9.2f  max %9.2f\n",tag,mn,s,mx);
}
template<class S> void run(const char* nm,int frames,int every){
  S s; auto t0=std::chrono::steady_clock::now();
  printf("== %s, %zu particles\n",nm,s.particles().size());
  for(int f=1;f<=frames;++f){ s.step();
    if(f%every==0){ auto&p=s.particles(); bool bad=false; for(size_t i=0;i<p.size();++i) if(!std::isfinite(p.x[i]+p.y[i]+p.speed[i]+p.pressure[i]+p.heat[i]+p.accel[i])) bad=true;
      float ymax=0; std::vector<float> ys=p.y; std::sort(ys.begin(),ys.end()); ymax=ys[(size_t)(0.98*ys.size())];
      printf(" frame %d  NaN:%d  y98%%=%.3f\n",f,bad,ymax);
      stats("speed",p.speed); stats("heat",p.heat); stats("pressure",p.pressure); stats("accel",p.accel);}
  }
  double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count();
  printf(" avg %.2f ms/frame\n",ms/frames);
}
int main(int argc,char**argv){
  if(argc>1&&argv[1][0]=='e') run<EulerSolver>("Euler",300,100); else run<SphSolver>("SPH",600,150);
}
