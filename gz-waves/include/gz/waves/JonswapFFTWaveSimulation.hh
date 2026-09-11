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

/// \file JonswapFFTWaveSimulation.hh
///
/// \brief FFT-based wave simulation driven by a JONSWAP spectrum
///        (JonswapWaveSpectrum) with an independently settable dominant
///        wave direction, rather than the ECKV-spectrum-plus-wind model
///        used by LinearRandomFFTWaveSimulation.
///
/// This class is a standalone addition: it duplicates (rather than
/// modifies) the FFT simulation machinery in LinearRandomFFTWaveSimulation
/// so that the existing 'fft' / ECKV algorithm is left completely untouched.

#ifndef GZ_WAVES_JONSWAPFFTWAVESIMULATION_HH_
#define GZ_WAVES_JONSWAPFFTWAVESIMULATION_HH_

#include <Eigen/Dense>

#include <memory>

#include "gz/waves/WaveSimulation.hh"

namespace gz
{
namespace waves
{
class JonswapFFTWaveSimulation :
    public IWaveSimulation
{
 public:
  virtual ~JonswapFFTWaveSimulation();

  JonswapFFTWaveSimulation(double lx, double ly,
      Index nx, Index ny);

  JonswapFFTWaveSimulation(double lx, double ly, double lz,
      Index nx, Index ny, Index nz);

  /// \brief Set lambda which controls the horizontal wave displacement.
  void SetLambda(double lambda);

  /// \brief Set the significant wave height (m).
  void SetHs(double value);

  /// \brief Set the peak period (s).
  void SetTp(double value);

  /// \brief Set the JONSWAP peak-enhancement factor.
  void SetGamma(double value);

  /// \brief Set the dominant wave direction (degrees, counter clockwise
  ///        from the positive x-axis). Independent of wind.
  void SetWaveDirectionDeg(double value);

  /// \brief Set the cosine-2S directional spreading parameter.
  void SetDirectionalSpread(double value);

  /// \todo(deprecated) not used by the JONSWAP spectrum - present only to
  ///       satisfy the IWaveSimulation interface. Use SetWaveDirectionDeg
  ///       to control wave direction instead.
  void SetWindVelocity(double ux, double uy) override;

  void SetSteepness(double value) override;

  void SetTime(double value) override;

  Index SizeX() const override;

  Index SizeY() const override;

  Index SizeZ() const override;

  // lookup interface - array
  void ElevationAt(
      Eigen::Ref<Eigen::ArrayXXd> h) const override;

  void ElevationDerivAt(
      Eigen::Ref<Eigen::ArrayXXd> dhdx,
      Eigen::Ref<Eigen::ArrayXXd> dhdy) const override;

  void DisplacementAt(
      Eigen::Ref<Eigen::ArrayXXd> sx,
      Eigen::Ref<Eigen::ArrayXXd> sy) const override;

  void DisplacementDerivAt(
      Eigen::Ref<Eigen::ArrayXXd> dsxdx,
      Eigen::Ref<Eigen::ArrayXXd> dsydy,
      Eigen::Ref<Eigen::ArrayXXd> dsxdy) const override;

  void DisplacementAndDerivAt(
      Eigen::Ref<Eigen::ArrayXXd> h,
      Eigen::Ref<Eigen::ArrayXXd> sx,
      Eigen::Ref<Eigen::ArrayXXd> sy,
      Eigen::Ref<Eigen::ArrayXXd> dhdx,
      Eigen::Ref<Eigen::ArrayXXd> dhdy,
      Eigen::Ref<Eigen::ArrayXXd> dsxdx,
      Eigen::Ref<Eigen::ArrayXXd> dsydy,
      Eigen::Ref<Eigen::ArrayXXd> dsxdy) const override;

  void PressureAt(
      Index iz,
      Eigen::Ref<Eigen::ArrayXXd> pressure) const override;

  // lookup interface - scalar
  void ElevationAt(
      Index ix, Index iy,
      double& eta) const override;

  void DisplacementAt(
      Index ix, Index iy,
      double& sx, double& sy) const override;

  void PressureAt(
      Index ix, Index iy, Index iz,
      double& pressure) const override;

  // public class declaration - for testing
  class Impl;

 private:
  std::unique_ptr<Impl> impl_;
};

}  // namespace waves
}  // namespace gz

#endif  // GZ_WAVES_JONSWAPFFTWAVESIMULATION_HH_
