// 复用B/C的无界面运行器，读取统一的manifest.csv与imu.csv。
// D增加System::GetDImuState接口，逐帧导出实际位姿、速度和偏置。
//
// run_orbslam MODE VOCAB CALIB SEQ_DIR OUT_DIR [--rate R] [--imu FILE] [--max-frames N] [--start S]
//   --rate 1 (default) plays back in real time; 0 = as fast as possible.
//   --start 0.2 (default): skip frames and IMU samples earlier than first_timestamp + S seconds
//     (TartanAir IMU has spline end effects in the first ~0.1 s).
//
// tracking.csv map_id: index of the tracking segment (one world frame lineage). Incremented on the first OK
// frame after (a) a frame in NO_IMAGES_YET/NOT_INITIALIZED, or (b) a re-initialisation that is not visible in
// the state sequence: RGB-D can go LOST(4) -> ResetActiveMap/CreateMapInAtlas -> one-frame StereoInitialization
// -> OK(2). (b) is detected read-only: none of the keyframes of the previous OK frame's map is in the current
// map (reset in place or a fresh map). -1 before the first initialisation.
// tracking.csv frame_epoch: within one map_id, incremented whenever ORB-SLAM3 re-defines the world frame
// (0 at the start of each map_id). Checked read-only on every OK frame against the previous OK frame:
//   imu_init / viba1 / viba2 : Map::isImuInitialized / GetIniertialBA1 / GetIniertialBA2 false -> true
//   loop_gba                 : Map::GetLastBigChangeIdx increased (loop correction, end of global BA)
//   map_switch               : the tracked points belong to another map that shares keyframes (map merge)
//   frame_jump               : the recent keyframes (up to 20, by id) moved together: a similarity fitted
//                              (Umeyama) between their old and new positions has rotation > --jump-rot-deg (1),
//                              |scale-1| > --jump-scale (0.03), or the median keyframe displacement is
//                              > --jump-disp (0.2 m). Covers ApplyScaledRotation (IMU init, ScaleRefinement,
//                              merge) and loop/GBA corrections, whose flags are set before/after the poses move.
// Every increment is logged in frame_events.csv. tracking.csv imu_init: active map IMU-initialised (0/1).
//
#include <sys/wait.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <algorithm>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <System.h>
#include <ImuTypes.h>

using namespace std;
typedef ORB_SLAM3::System Sys;

struct Row { double t; string rgb, depth; };

static vector<string> split(string s) {
    while (!s.empty() && (s.back() == '\r' || s.back() == '\n')) s.pop_back();
    vector<string> v; string c; stringstream ss(s);
    while (getline(ss, c, ',')) v.push_back(c);
    return v;
}

static void writeTum(ostream &f, double t, const Sophus::SE3f &T) {
    Eigen::Vector3f p = T.translation();
    Eigen::Quaternionf q = T.unit_quaternion();
    f << fixed << setprecision(6) << t << setprecision(9) << " " << p.x() << " " << p.y() << " " << p.z()
      << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
}

// Read-only watcher for world-frame re-definitions (see map_id / frame_epoch above).
struct FrameWatch {
    enum { NONE = 0, EPOCH = 1, REINIT = 2 };
    ORB_SLAM3::Map *map = nullptr;
    bool imu = false, ba1 = false, ba2 = false;
    int big = 0;
    std::map<ORB_SLAM3::KeyFrame *, Eigen::Vector3f> kfs;  // camera centres at the previous OK frame

    void Snapshot(const vector<ORB_SLAM3::KeyFrame *> &cur) {
        kfs.clear();
        for (auto *kf : cur) if (kf && !kf->isBad()) kfs[kf] = kf->GetPoseInverse().translation();
    }
    int Observe(ORB_SLAM3::Map *m, string &why, double &rotDeg, double &scale, double &disp, int &nCommon,
                double thRot, double thScale, double thDisp) {
        const bool nImu = m->isImuInitialized(), nBa1 = m->GetIniertialBA1(), nBa2 = m->GetIniertialBA2();
        const int nBig = m->GetLastBigChangeIdx();
        const vector<ORB_SLAM3::KeyFrame *> cur = m->GetAllKeyFrames();
        const std::set<ORB_SLAM3::KeyFrame *> curSet(cur.begin(), cur.end());
        vector<ORB_SLAM3::KeyFrame *> common;
        for (auto &e : kfs) if (curSet.count(e.first) && !e.first->isBad()) common.push_back(e.first);
        nCommon = common.size();
        int res = NONE;
        if (map && !kfs.empty() && common.empty()) { why = "reinit"; res = REINIT; }
        else if (map) {
            if (m != map) why += "map_switch;";
            if (nImu && !imu && m == map) why += "imu_init;";
            if (nBa1 && !ba1 && m == map) why += "viba1;";
            if (nBa2 && !ba2 && m == map) why += "viba2;";
            if (nBig > big && m == map) why += "loop_gba;";
            sort(common.begin(), common.end(),
                 [](ORB_SLAM3::KeyFrame *a, ORB_SLAM3::KeyFrame *b) { return a->mnId > b->mnId; });
            if (common.size() > 20) common.resize(20);
            if (common.size() >= 3) {
                Eigen::Matrix3Xf src(3, common.size()), dst(3, common.size());
                vector<float> d;
                for (size_t i = 0; i < common.size(); i++) {
                    src.col(i) = kfs[common[i]];
                    dst.col(i) = common[i]->GetPoseInverse().translation();
                    d.push_back((dst.col(i) - src.col(i)).norm());
                }
                nth_element(d.begin(), d.begin() + d.size() / 2, d.end());
                disp = d[d.size() / 2];
                const Eigen::Matrix4f S = Eigen::umeyama(src, dst, true);
                scale = S.block<3, 3>(0, 0).col(0).norm();
                const Eigen::Matrix3f R = S.block<3, 3>(0, 0) / scale;
                rotDeg = Eigen::AngleAxisf(R).angle() * 180.0 / M_PI;
                if (rotDeg > thRot || fabs(scale - 1.0) > thScale || disp > thDisp) why += "frame_jump;";
            }
            if (!why.empty()) { why.pop_back(); res = EPOCH; }
        }
        map = m; imu = nImu; ba1 = nBa1; ba2 = nBa2; big = nBig;
        Snapshot(cur);
        return res;
    }
};

int main(int argc, char **argv) {
    if (argc < 6) {
        cerr << "usage: run_orbslam MODE VOCAB CALIB SEQ_DIR OUT_DIR [--rate R] [--imu FILE] [--max-frames N]\n";
        return 2;
    }
    const string mode = argv[1], voc = argv[2], calib = argv[3], seq = argv[4], out = argv[5];
    double rate = 1.0, start = 0.2, jumpRotDeg = 1.0, jumpScale = 0.03, jumpDisp = 0.2; string imuFile = seq + "/imu.csv"; size_t maxFrames = 0;
    for (int i = 6; i < argc; i += 2) {
        string a = argv[i];
        if (i + 1 >= argc) { cerr << "missing value for " << a << "\n"; return 2; }
        if (a == "--rate") rate = atof(argv[i + 1]);
        else if (a == "--imu") imuFile = argv[i + 1];
        else if (a == "--max-frames") maxFrames = strtoul(argv[i + 1], nullptr, 10);
        else if (a == "--start") start = atof(argv[i + 1]);
        else if (a == "--jump-rot-deg") jumpRotDeg = atof(argv[i + 1]);
        else if (a == "--jump-scale") jumpScale = atof(argv[i + 1]);
        else if (a == "--jump-disp") jumpDisp = atof(argv[i + 1]);
        else { cerr << "unknown option " << a << "\n"; return 2; }
    }
    const map<string, Sys::eSensor> modes = {{"MONOCULAR", Sys::MONOCULAR}, {"IMU_MONOCULAR", Sys::IMU_MONOCULAR},
                                             {"RGBD", Sys::RGBD}, {"IMU_RGBD", Sys::IMU_RGBD}};
    if (!modes.count(mode)) { cerr << "bad mode " << mode << "\n"; return 2; }
    const bool useImu = mode.compare(0, 3, "IMU") == 0, useDepth = mode.find("RGBD") != string::npos;

    vector<Row> rows; string line;
    ifstream fm(seq + "/manifest.csv"); getline(fm, line);
    while (getline(fm, line)) { auto c = split(line); if (c.size() >= 3) rows.push_back({stod(c[0]), c[1], c[2]}); }
    if (!rows.empty()) {
        const double tStart = rows[0].t + start - 1e-6;
        size_t n0 = 0;
        while (n0 < rows.size() && rows[n0].t < tStart) n0++;
        rows.erase(rows.begin(), rows.begin() + n0);
    }
    if (maxFrames && rows.size() > maxFrames) rows.resize(maxFrames);
    if (rows.empty()) { cerr << "empty manifest\n"; return 2; }

    vector<ORB_SLAM3::IMU::Point> imu; Sophus::SE3f Tbc;
    if (useImu) {
        ifstream fi(imuFile); getline(fi, line);
        while (getline(fi, line)) {
            auto c = split(line); if (c.size() < 7) continue;
            if (!rows.empty() && stod(c[0]) < rows[0].t - 1e-6) continue;
            imu.emplace_back(stof(c[4]), stof(c[5]), stof(c[6]), stof(c[1]), stof(c[2]), stof(c[3]), stod(c[0]));
        }
        if (imu.empty()) { cerr << "empty IMU file " << imuFile << "\n"; return 2; }
        cv::FileStorage fs(calib, cv::FileStorage::READ); cv::Mat M; fs["IMU.T_b_c1"] >> M; M.convertTo(M, CV_32F);
        Eigen::Matrix3f R; Eigen::Vector3f p;
        for (int r = 0; r < 3; r++) { for (int c = 0; c < 3; c++) R(r, c) = M.at<float>(r, c); p(r) = M.at<float>(r, 3); }
        Tbc = Sophus::SE3f(Eigen::Quaternionf(R).normalized(), p);
    }

    Sys slam(voc, calib, modes.at(mode), false);
    ofstream ft(out + "/tracking.csv"), fo(out + "/trajectory_online.tum");
    ofstream fev(out + "/frame_events.csv");
    // D实验新增：逐帧保存本体位置、朝向、速度和两类偏置，原来的B/C输出仍保留。
    ofstream fstate(out + "/slam_states.csv");
    fstate << "timestamp,px,py,pz,qx,qy,qz,qw,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz,valid,imu_initialized,map_id,frame_epoch\n";
    ft << "timestamp,valid,state,quality_rgb,quality_depth,source,latency_ms,map_id,frame_epoch,imu_init\n";
    fev << "timestamp,map_id,frame_epoch,reason,rot_deg,scale,median_disp_m,n_common_kf\n";
    vector<int> states(rows.size(), -1);
    size_t k = 0, nOk = 0; double sumLat = 0;
    int mapId = -1; bool uninit = true;
    int epoch = 0, nEvents = 0, nReinit = 0; FrameWatch w;
    const auto wall0 = chrono::steady_clock::now();
    for (size_t i = 0; i < rows.size(); i++) {
        const Row &r = rows[i];
        cv::Mat rgb = cv::imread(seq + "/" + r.rgb, cv::IMREAD_UNCHANGED), depth;
        if (rgb.empty()) { cerr << "cannot read " << r.rgb << "\n"; return 3; }
        double qDepth = NAN;
        if (useDepth) {
            depth = cv::imread(seq + "/" + r.depth, cv::IMREAD_UNCHANGED);
            if (depth.empty() || depth.type() != CV_16UC1) { cerr << "bad depth " << r.depth << "\n"; return 3; }
            qDepth = double(cv::countNonZero(depth)) / depth.total();
        }
        // IMU samples in (t_prev, t_cur]
        vector<ORB_SLAM3::IMU::Point> meas;
        while (k < imu.size() && imu[k].t <= r.t) meas.push_back(imu[k++]);
        if (rate > 0) {
            auto due = wall0 + chrono::duration_cast<chrono::steady_clock::duration>(
                                   chrono::duration<double>((r.t - rows[0].t) / rate));
            auto now = chrono::steady_clock::now();
            if (due > now) usleep(chrono::duration_cast<chrono::microseconds>(due - now).count());
        }
        auto t0 = chrono::steady_clock::now();
        Sophus::SE3f Tcw = useDepth ? slam.TrackRGBD(rgb, depth, r.t, meas) : slam.TrackMonocular(rgb, r.t, meas);
        double ms = chrono::duration<double, milli>(chrono::steady_clock::now() - t0).count();
        const int st = slam.GetTrackingState(); states[i] = st;
        int nTracked = 0;
        std::map<ORB_SLAM3::Map *, int> votes;
        for (auto *mp : slam.GetTrackedMapPoints())
            if (mp) { nTracked++; if (!mp->isBad()) votes[mp->GetMap()]++; }
        ORB_SLAM3::Map *pMap = nullptr;
        for (auto &v : votes) if (!pMap || v.second > votes[pMap]) pMap = v.first;
        const bool ok = st == ORB_SLAM3::Tracking::OK;
        if (st == ORB_SLAM3::Tracking::NO_IMAGES_YET || st == ORB_SLAM3::Tracking::NOT_INITIALIZED) uninit = true;
        else if (ok && uninit) { mapId++; uninit = false; epoch = 0; w = FrameWatch(); }
        if (ok && pMap) {
            string why; double rotDeg = 0, scale = 1, disp = 0; int nCommon = 0;
            const int ev = w.Observe(pMap, why, rotDeg, scale, disp, nCommon, jumpRotDeg, jumpScale, jumpDisp);
            if (ev == FrameWatch::REINIT) { mapId++; epoch = 0; nReinit++; }
            else if (ev == FrameWatch::EPOCH) { epoch++; nEvents++; }
            if (ev != FrameWatch::NONE)
                fev << fixed << setprecision(6) << r.t << "," << mapId << "," << epoch << "," << why << ","
                    << setprecision(4) << rotDeg << "," << setprecision(6) << scale << "," << setprecision(4) << disp
                    << "," << nCommon << "\n";
        }
        if (ok) { nOk++; writeTum(fo, r.t, Tcw.inverse()); }
        Eigen::Vector3f dp, dv; Eigen::Matrix3f dR; ORB_SLAM3::IMU::Bias db;
        const bool dvalid = useImu && slam.GetDImuState(dp, dR, dv, db);
        fstate << fixed << setprecision(9) << r.t;
        if (dvalid) {
            const Eigen::Quaternionf dq(dR);
            fstate << "," << dp.x() << "," << dp.y() << "," << dp.z()
                   << "," << dq.x() << "," << dq.y() << "," << dq.z() << "," << dq.w()
                   << "," << dv.x() << "," << dv.y() << "," << dv.z()
                   << "," << db.bwx << "," << db.bwy << "," << db.bwz
                   << "," << db.bax << "," << db.bay << "," << db.baz;
        } else { for (int j = 0; j < 16; ++j) fstate << ",0"; }
        fstate << "," << dvalid << "," << (w.map ? int(w.imu) : 0)
               << "," << (uninit ? -1 : mapId) << "," << epoch << "\n";
        sumLat += ms;
        // quality_rgb: number of tracked map points; quality_depth: fraction of non-zero depth pixels.
        // state 3 (RECENTLY_LOST) in IMU modes is IMU prediction, so it is not valid.
        const char *src = ok ? "SLAM" : (useImu && st == ORB_SLAM3::Tracking::RECENTLY_LOST ? "IMU" : "NONE");
        ft << fixed << setprecision(6) << r.t << "," << ok << "," << st << "," << nTracked << ",";
        if (useDepth) ft << setprecision(4) << qDepth;
        ft << "," << src << "," << setprecision(3) << ms << "," << (uninit ? -1 : mapId) << "," << epoch << ","
           << (w.map ? int(w.imu) : 0) << "\n";
    }
    ft.close(); fo.close(); fev.close(); fstate.close();
    cout << "RUN_SUMMARY mode=" << mode << " frames=" << rows.size() << " ok=" << nOk << " map_segments=" << mapId + 1 << " epoch_events=" << nEvents << " silent_reinits=" << nReinit
         << " mean_latency_ms=" << sumLat / rows.size() << endl;

    // Everything above is on disk already: the ORB-SLAM3 save below may crash on an empty atlas.
    slam.Shutdown();
    sleep(5);  // Shutdown() does not join the mapping threads
    ofstream(out + "/trajectory_est.tum").close();
    if (nOk) {
        const string raw = out + "/trajectory_final_raw.txt";
        // SaveTrajectoryEuRoC dereferences an empty map if the last reset cleared it:
        // run it in a forked child so a crash there only means "no final trajectory".
        cout.flush();
        pid_t pid = fork();
        if (pid < 0) { cerr << "fork failed\n"; _exit(4); }
        if (pid == 0) { slam.SaveTrajectoryEuRoC(raw); cout.flush(); _exit(0); }
        int ws = 0; waitpid(pid, &ws, 0);
        const bool saved = WIFEXITED(ws) && WEXITSTATUS(ws) == 0;
        cout << "FINAL_SAVE " << (saved ? "ok" : "crashed") << " wait_status=" << ws << endl;
        if (!saved) unlink(raw.c_str());  // never use a partial file
        map<long long, size_t> idx;
        for (size_t i = 0; i < rows.size(); i++) idx[llround(rows[i].t * 1e5)] = i;
        ifstream fr(raw); ofstream fe(out + "/trajectory_est.tum");
        map<size_t, Sophus::SE3f> poses; double ns, x, y, z, qx, qy, qz, qw;
        while (fr >> ns >> x >> y >> z >> qx >> qy >> qz >> qw) {
            auto it = idx.find(llround(ns * 1e-9 * 1e5));
            if (it == idx.end() || states[it->second] != ORB_SLAM3::Tracking::OK) continue;
            Sophus::SE3f T(Eigen::Quaternionf(qw, qx, qy, qz).normalized(), Eigen::Vector3f(x, y, z));
            poses[it->second] = useImu ? T * Tbc : T;  // IMU modes save T_w_b
        }
        for (auto &p : poses) writeTum(fe, rows[p.first].t, p.second);
        cout << "FINAL_POSES " << poses.size() << endl;
    }
    cout.flush();
    _exit(0);  // skip destructors while ORB-SLAM3 threads may still be alive
}
