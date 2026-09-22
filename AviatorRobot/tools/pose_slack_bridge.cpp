// Offline-only C ABI for whole-trajectory optimization; reuses the study's model and clearance.
#define main clearance_study_original_main
#include "clearance_trajectory.cpp"
#undef main

namespace {
std::vector<std::unique_ptr<ClearanceTrajectory>> workers;
Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
    Eigen::Matrix3d a; a << 0,-v.z(),v.y(),v.z(),0,-v.x(),-v.y(),v.x(),0; return a;
}
Eigen::Vector3d rotation_log(const KDL::Rotation &R) {
    Eigen::Matrix3d A;for(int i=0;i<3;i++)for(int j=0;j<3;j++)A(i,j)=R(i,j);
    Eigen::AngleAxisd aa(A);return aa.angle()*aa.axis();
}
void fast_set(ClearanceTrajectory &r, int side, const double *q, double th, double s) {
    for(int j=0;j<7;j++) r.d->qpos[r.m->jnt_qposadr[r.joint[side][j]]]=q[j];
    r.d->qpos[r.m->jnt_qposadr[r.wheel[0]]]=th;
    r.d->qpos[r.m->jnt_qposadr[r.wheel[1]]]=s;
    mj_kinematics(r.m.get(),r.d.get()); mj_comPos(r.m.get(),r.d.get());
}
// Closest sampled mesh point, with the same box distance as wall_clearance().
double clearance(ClearanceTrajectory &r,int side,double *grad) {
    double best=1e9; Eigen::Vector3d bpbest=Eigen::Vector3d::Zero(), normal=bpbest; int body=-1;
    for(const auto &cl:r.cloud[side]) {
        Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> B(r.d->xmat+9*cl.body);
        Eigen::Map<const Eigen::Vector3d> p(r.d->xpos+3*cl.body);
        for(const auto &pt:cl.pts) {
            Eigen::Vector3d w=p+B*Eigen::Vector3d(pt[0],pt[1],pt[2]);
            for(int wi=0;wi<std::min(2,(int)r.wall_geoms.size());wi++) {
                Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> R(r.wall_xmat[wi].data());
                Eigen::Map<const Eigen::Vector3d> c(r.wall_center[wi].data()),h(r.wall_half[wi].data());
                Eigen::Vector3d l=R.transpose()*(w-c),z=l.cwiseAbs()-h,a=z.cwiseMax(0.);
                double out=a.norm(),dist=out>0?out:r.wall_inner_sign[wi]*l.z()-h.z();
                if(dist<best) {
                    best=dist;body=cl.body;bpbest=w;
                    Eigen::Vector3d n=Eigen::Vector3d::Zero();
                    if(out>0) for(int j=0;j<3;j++) n[j]=a[j]/out*(l[j]>=0?1:-1);
                    else n.z()=r.wall_inner_sign[wi];
                    normal=R*n;
                }
            }
        }
    }
    if(grad) {
        std::vector<double> J(3*r.m->nv);
        mj_jac(r.m.get(),r.d.get(),J.data(),nullptr,bpbest.data(),body);
        for(int j=0;j<7;j++) { int col=r.m->jnt_dofadr[r.joint[side][j]]; grad[j]=0;
            for(int a=0;a<3;a++) grad[j]+=normal[a]*J[a*r.m->nv+col]; }
    }
    return best;
}
void evaluate(ClearanceTrajectory &r,int side,const double *q,const double *x,double *e,double *jac) {
    fast_set(r,side,q,x[0],x[1]);
    auto p=r.site_pos(r.tcp_site[side])-r.site_pos(r.handle_site[side]);
    auto v=rotation_log(r.site_rot(r.tcp_site[side])*r.site_rot(r.handle_site[side]).Inverse());
    for(int a=0;a<3;a++){e[a]=p[a];e[a+3]=v[a];}
    e[6]=clearance(r,side,jac?jac+42:nullptr);
    if(jac) {
        auto J=r.arm_jac(side);
        Eigen::Vector3d rv(v[0],v[1],v[2]); double t=rv.norm(); auto K=skew(rv);
        double c=t<1e-5?1./12.:(1.-0.5*t/std::tan(0.5*t))/(t*t);
        Eigen::Matrix3d Ji=Eigen::Matrix3d::Identity()-0.5*K+c*K*K;
        Eigen::Matrix<double,3,7> Jr=Ji*J.bottomRows<3>();
        for(int a=0;a<3;a++) for(int j=0;j<7;j++){jac[a*7+j]=J(a,j);jac[(a+3)*7+j]=Jr(a,j);}
    }
}
}
extern "C" {
int ps_init(const char *config,int nt,double *bounds) {
    try {
        workers.clear();
        for(int i=0;i<nt;i++) workers.emplace_back(new ClearanceTrajectory(fs::absolute(config),.005,.03,10,.1));
        for(int s=0;s<2;s++)for(int j=0;j<7;j++){bounds[s*14+j]=workers[0]->lower[s](j);bounds[s*14+7+j]=workers[0]->upper[s](j);}
        return 0;
    } catch(const std::exception &e){std::cerr<<e.what()<<std::endl;return -1;}
}
void ps_stream(int n,double dt,double *x) {
    std::mt19937 rng(987654321u);std::uniform_real_distribution<double> U01(0.,1.),Uph(0.,2.*M_PI);
    double a[5],p[5],b[5],r[5];const double f[]={.10,.25,.40,.70,1.10},g[]={.08,.20,.35,.60,.90};
    for(int i=0;i<5;i++){a[i]=.35+.65*U01(rng);p[i]=Uph(rng);b[i]=.35+.65*U01(rng);r[i]=Uph(rng);}
    for(int k=0;k<n;k++){double th=0,s=-.08;for(int i=0;i<5;i++){th+=a[i]*std::sin(2*M_PI*f[i]*k*dt+p[i]);s+=.04*b[i]*std::sin(2*M_PI*g[i]*k*dt+r[i]);}
        x[k*2]=std::clamp(.5*th,-.87266,.87266);x[k*2+1]=std::clamp(s,-.16,0.);}
}
void ps_eval(int n,int side,const double *q,const double *x,double *e,double *jac) {
    std::vector<std::thread> ts;
    for(int w=0;w<(int)workers.size();w++) ts.emplace_back([&,w]{
        for(int k=w;k<n;k+=workers.size()) evaluate(*workers[w],side,q+7*k,x+2*k,e+7*k,jac?jac+49*k:nullptr);
    }); for(auto &t:ts)t.join();
}
void ps_validate(int n,const double *q,const double *x,double *out) {
    std::vector<std::thread> ts;
    for(int w=0;w<(int)workers.size();w++)ts.emplace_back([&,w]{auto &r=*workers[w];
        for(int k=w;k<n;k+=workers.size()) {
            Q a(7),b(7);for(int j=0;j<7;j++){a(j)=q[k*14+j];b(j)=q[k*14+7+j];}
            r.set_config(a,b,x[k*2],x[k*2+1]);
            double *o=out+8*k;o[0]=r.task_error();o[1]=r.wall_clearance(0);o[2]=r.wall_clearance(1);o[3]=r.collision()?1:0;
            for(int s=0;s<2;s++){o[4+s]=(r.site_rot(r.tcp_site[s])*r.site_rot(r.handle_site[s]).Inverse()).GetRot().Norm();o[6+s]=r.arm_margin(s,s?b:a);}
        }
    });for(auto &t:ts)t.join();
}
}

// Re-evaluate sampled set separation and candidate DP after adding an entire
// validated continuous rigid trajectory. This cannot be worse than that seed path.
extern "C" void ps_seeded_dp(int n,const double *q,const double *x,double dt,double *summary,double *path) {
    for(int side=0;side<2;side++) {
        std::vector<std::vector<Q>> candidates(n);
        std::vector<std::thread> ts;
        for(int w=0;w<(int)workers.size();w++)ts.emplace_back([&,w]{auto &r=*workers[w];
            for(int k=w;k<n;k+=workers.size()) {
                Q a(7),b(7);for(int j=0;j<7;j++){a(j)=q[14*k+7*side+j];b(j)=q[14*k+7*(1-side)+j];}
                std::vector<Q> c;std::vector<double> cd;Q qs(7),qm(7);double ds,dm;bool hs;
                r.ik[side]->setSeed(atlas_seed(0,side,1000));
                r.collect_boundary_candidates(side,a,b,x[2*k],x[2*k+1],64,c,cd,qs,ds,hs,qm,dm);
                // The seed is a separately validated rigid, clearance-safe path.
                candidates[k].push_back(a);
                for(size_t j=0;j<c.size();j++)if(cd[j]>=.005 && r.arm_margin(side,c[j])>=-1e-8)candidates[k].push_back(c[j]);
            }
        });for(auto &t:ts)t.join();
        double sep=0;int sepk=0;
        std::vector<double> previous(candidates[0].size(),0),current;
        std::vector<std::vector<int>> back(n);
        for(int k=1;k<n;k++) {
            current.assign(candidates[k].size(),1e99);back[k].resize(current.size());double sep_k=1e99;
            for(size_t b=0;b<candidates[k].size();b++)for(size_t a=0;a<candidates[k-1].size();a++) {
                double step=0;for(int j=0;j<7;j++)step=std::max(step,std::abs(candidates[k][b](j)-candidates[k-1][a](j)));
                sep_k=std::min(sep_k,step);double v=std::max(previous[a],step);
                if(v<current[b]){current[b]=v;back[k][b]=a;}
            }
            if(sep_k>sep){sep=sep_k;sepk=k;}previous.swap(current);
        }
        int idx=std::min_element(previous.begin(),previous.end())-previous.begin();
        summary[side*3]=previous[idx]/dt;summary[side*3+1]=sep/dt;summary[side*3+2]=sepk*dt;
        for(int k=n-1;k>=0;k--){for(int j=0;j<7;j++)path[14*k+7*side+j]=candidates[k][idx](j);if(k>0)idx=back[k][idx];}
    }
}
