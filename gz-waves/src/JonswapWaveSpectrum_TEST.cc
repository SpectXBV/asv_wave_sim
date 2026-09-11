// Copyright (C) 2025  AquaFind
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "gz/waves/JonswapWaveSpectrum.hh"
#include "gz/waves/Types.hh"

using gz::waves::Index;
using gz::waves::JonswapWaveSpectrum;

namespace
{
// Independently recover the significant wave height implied by a
// JonswapWaveSpectrum by numerically integrating S(k) dk over a wide
// range of k (equivalent, by change of variables, to integrating S(w) dw
// over angular frequency), using only the public Evaluate(k) interface.
double RecoverHs(const JonswapWaveSpectrum& spectrum,
    double tp, double gravity)
{
  const double omega_p = 2.0 * M_PI / tp;
  const double omega_min = 0.02 * omega_p;
  const double omega_max = 15.0 * omega_p;
  const int n = 20000;

  double m0 = 0.0;
  double k_prev = omega_min * omega_min / gravity;
  double s_prev = spectrum.Evaluate(k_prev);
  for (int i = 1; i <= n; ++i)
  {
    double omega = omega_min + i * (omega_max - omega_min) / n;
    double k = omega * omega / gravity;
    double s = spectrum.Evaluate(k);
    m0 += 0.5 * (s_prev + s) * (k - k_prev);
    k_prev = k;
    s_prev = s;
  }
  return 4.0 * std::sqrt(m0);
}
}  // namespace

//////////////////////////////////////////////////
TEST(JonswapWaveSpectrum, RecoversSignificantWaveHeight)
{
  const double gravity = 9.81;
  const double tolerance = 0.02;  // 2%, accounts for truncated integration

  struct Case { double hs; double tp; double gamma; };
  std::vector<Case> cases = {
    {1.0, 6.0, 3.3},
    {2.0, 8.0, 3.3},
    {3.0, 9.0, 1.0},
    {4.5, 12.0, 5.0},
  };

  for (const auto& c : cases)
  {
    JonswapWaveSpectrum spectrum(c.hs, c.tp, c.gamma, gravity);
    double hs_recovered = RecoverHs(spectrum, c.tp, gravity);
    EXPECT_NEAR(hs_recovered, c.hs, c.hs * tolerance)
        << "hs=" << c.hs << " tp=" << c.tp << " gamma=" << c.gamma;
  }
}

//////////////////////////////////////////////////
TEST(JonswapWaveSpectrum, IndependentOfHs)
{
  // Changing Hs alone should not change Tp (the spectral peak location).
  const double gravity = 9.81;
  const double tp = 10.0;
  const double gamma = 3.3;

  JonswapWaveSpectrum spectrum_a(1.0, tp, gamma, gravity);
  JonswapWaveSpectrum spectrum_b(5.0, tp, gamma, gravity);

  const double omega_p = 2.0 * M_PI / tp;
  const double k_p = omega_p * omega_p / gravity;

  // Ratio of peak spectral density should equal ratio of Hs^2.
  double ratio = spectrum_b.Evaluate(k_p) / spectrum_a.Evaluate(k_p);
  EXPECT_NEAR(ratio, 25.0, 25.0 * 0.02);
}

//////////////////////////////////////////////////
TEST(JonswapWaveSpectrum, ArrayXXd)
{
  double tolerance = 1.0e-16;

  double lx = 200.0;
  double ly = 100.0;
  Index nx = 32;
  Index ny = 16;

  double kx_nyquist = M_PI * nx / lx;
  double ky_nyquist = M_PI * ny / ly;

  Eigen::ArrayXd kx_v(nx);
  Eigen::ArrayXd ky_v(ny);

  for (Index i=0; i < nx; ++i)
  {
    kx_v(i) = (i * 2.0 / nx - 1.0) * kx_nyquist;
  }
  for (Index i=0; i < ny; ++i)
  {
    ky_v(i) = (i * 2.0 / ny - 1.0) * ky_nyquist;
  }

  Eigen::ArrayXXd kx = Eigen::ArrayXXd::Zero(nx, ny);
  kx.colwise() += kx_v;

  Eigen::ArrayXXd ky = Eigen::ArrayXXd::Zero(nx, ny);
  ky.rowwise() += ky_v.transpose();

  Eigen::ArrayXXd kx2 = Eigen::pow(kx, 2.0);
  Eigen::ArrayXXd ky2 = Eigen::pow(ky, 2.0);
  Eigen::ArrayXXd k = Eigen::sqrt(kx2 + ky2);

  JonswapWaveSpectrum spectrum(2.0, 8.0, 3.3);

  Eigen::ArrayXXd cap_s = Eigen::ArrayXXd::Zero(nx, ny);
  spectrum.Evaluate(cap_s, k);

  for (Index i=0; i < nx; ++i)
  {
    for (Index j=0; j < ny; ++j)
    {
      double cap_s_test = spectrum.Evaluate(k(i, j));
      EXPECT_NEAR(cap_s(i, j), cap_s_test, tolerance);
    }
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
