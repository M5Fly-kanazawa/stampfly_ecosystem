// baro_offset_eval.cpp — offline evaluation of the firmware's barometric height estimator.
//
// Compiles the SAME header the vehicle runs (sf::BarometricHeightEstimator,
// firmware/vehicle/components/sf_core/include/barometric_height_estimator.hpp; Code Identity, no
// re-implementation). baro_offset_study.py flattens a flight log into an event file; this program
// replays it and, for every start time t0 given in the file, pretends that the ToF goes silent at t0:
// the learned offset is held (no ToF sample is fed any more) and the error
//     error(t) = baro_height(t) - ToF height(t)          t0 < t <= t0 + horizon
// is printed, where the ToF height is the tilt-corrected real ToF of the log (the truth).
//
// 飛行ログを気圧高度推定器に通す。ファームと同じヘッダ（sf::BarometricHeightEstimator）をそのまま
// コンパイルする（再実装ではない）。baro_offset_study.py がログをイベントファイルに平坦化し、本プログラムが
// 再生する。ファイルにある各開始時刻 t0 について「t0 で ToF が出力停止した」と仮定し、学習したオフセットを
// 保持して（以後 ToF サンプルを与えない）、気圧高度 − ToF の高さ（実機の ToF の傾き補正値 = 真値）を出す。
//
// Event file (whitespace separated, sorted by time t [s]):
//   B t altitude_m pressure_pa armed on_ground     barometer sample (as published on sensor_baro)
//   T t height_m tilt_rad usable                   ToF sample fed to the estimator (usable = valid)
//   Q t height_m                                   truth: a VALID tilt-corrected ToF height (not fed)
//   S t0                                           start time of one ToF-silence experiment
//
// Usage: baro_offset_eval <events.txt> <mode> [learn_tau_s] [learning 0|1] [zero_tau_s]
//   mode = series : print "H t height offset valid learned age" for every new baro sample
//   mode = silence: print "E t0 dt error" (error NaN where no truth lies within 0.15 s on both sides)
//                   and "N t0 dt" for every baro sample after t0 within 10.5 s
//
// build: g++ -std=c++17 -O2 -I firmware/vehicle/components/sf_core/include \
//            -I firmware/vehicle/components/sf_math/include baro_offset_eval.cpp -o baro_offset_eval

#include "barometric_height_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr double kMicrosecondsPerSecond = 1.0e6;
constexpr double kHorizonS              = 10.5;   // longest horizon printed / 出力する最長の時間幅
constexpr double kTruthGapS             = 0.15;   // widest ToF gap the truth is interpolated over / 真値を補間してよい ToF の間隔の上限

struct Event {
    char   kind;        // 'B', 'T', 'Q', 'S'
    double t;
    double a, b;        // B: altitude, pressure.  T: height, tilt.  Q: height.  S: -
    int    flag0, flag1;  // B: armed, on_ground.  T: usable
};

std::vector<Event> readEvents(const char* path)
{
    std::vector<Event> events;
    std::ifstream input(path);
    std::string line;
    while (std::getline(input, line)) {
        std::istringstream fields(line);
        Event e{};
        std::string kind;
        fields >> kind >> e.t;
        e.kind = kind[0];
        if (e.kind == 'B') fields >> e.a >> e.b >> e.flag0 >> e.flag1;
        if (e.kind == 'T') fields >> e.a >> e.b >> e.flag0;
        if (e.kind == 'Q') fields >> e.a;
        events.push_back(e);
    }
    return events;
}

uint32_t toMicroseconds(double seconds) { return static_cast<uint32_t>(std::llround(seconds * kMicrosecondsPerSecond)); }

void feedBaro(sf::BarometricHeightEstimator& estimator, const Event& e)
{
    sf::BarometricSample sample{};
    sample.altitude_m   = static_cast<float>(e.a);
    sample.pressure_pa  = static_cast<float>(e.b);
    sample.timestamp_us = toMicroseconds(e.t);
    sf::BarometricContext context{};
    context.armed     = e.flag0 != 0;
    context.on_ground = e.flag1 != 0;
    estimator.onBaroSample(sample, context);
}

void feedToF(sf::BarometricHeightEstimator& estimator, const Event& e)
{
    sf::BarometricToFSample sample{};
    sample.height_m     = static_cast<float>(e.a);
    sample.tilt_rad     = static_cast<float>(e.b);
    sample.timestamp_us = toMicroseconds(e.t);
    sample.usable       = e.flag0 != 0;
    estimator.onToFSample(sample);
}

// Truth ToF height at time t by linear interpolation of the Q events; NaN if a gap is wider than kTruthGapS.
// 時刻 t の真値: Q イベントの線形補間。間隔が kTruthGapS より広ければ NaN。
double truthAt(const std::vector<Event>& truth, double t)
{
    const auto next = std::lower_bound(truth.begin(), truth.end(), t, [](const Event& e, double v) { return e.t < v; });
    if (next == truth.begin() || next == truth.end()) return std::numeric_limits<double>::quiet_NaN();
    const Event& before = *(next - 1);
    if (next->t - before.t > kTruthGapS) return std::numeric_limits<double>::quiet_NaN();
    const double w = (t - before.t) / (next->t - before.t);
    return before.a + w * (next->a - before.a);
}

void runSeries(const std::vector<Event>& events, const sf::BarometricHeightConfig& config)
{
    sf::BarometricHeightEstimator estimator;
    estimator.setConfig(config);
    for (const Event& e : events) {
        if (e.kind == 'T') feedToF(estimator, e);
        if (e.kind != 'B') continue;
        feedBaro(estimator, e);
        const sf::BarometricHeightOutput out = estimator.output(toMicroseconds(e.t));
        std::printf("H %.4f %.4f %.4f %d %d %.2f\n", e.t, out.height_m, out.offset_m, out.valid, out.learned, out.offset_age_s);
    }
}

void runSilence(const std::vector<Event>& events, const std::vector<Event>& truth, const sf::BarometricHeightConfig& config)
{
    for (const Event& start : events) {
        if (start.kind != 'S') continue;
        sf::BarometricHeightEstimator estimator;
        estimator.setConfig(config);
        for (const Event& e : events) {
            const bool silent = e.t > start.t;
            if (e.kind == 'T' && !silent) feedToF(estimator, e);
            if (e.kind != 'B' || e.t > start.t + kHorizonS) continue;
            feedBaro(estimator, e);
            if (!silent) continue;
            const double reference = truthAt(truth, e.t);
            const double height    = estimator.output(toMicroseconds(e.t)).height_m;
            std::printf("E %.3f %.3f %.4f\n", start.t, e.t - start.t, height - reference);
        }
    }
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s events.txt series|silence [learn_tau_s] [learning 0|1] [zero_tau_s]\n", argv[0]);
        return 2;
    }
    sf::BarometricHeightConfig config;
    if (argc > 3) config.learn_tau_s      = static_cast<float>(std::atof(argv[3]));
    if (argc > 4) config.learning_enabled = std::atoi(argv[4]) != 0;
    if (argc > 5) config.zero_tau_s       = static_cast<float>(std::atof(argv[5]));
    const std::vector<Event> events = readEvents(argv[1]);
    std::vector<Event> truth;
    for (const Event& e : events) {
        if (e.kind == 'Q') truth.push_back(e);
    }
    if (std::strcmp(argv[2], "series") == 0) runSeries(events, config);
    else                                     runSilence(events, truth, config);
    return 0;
}
