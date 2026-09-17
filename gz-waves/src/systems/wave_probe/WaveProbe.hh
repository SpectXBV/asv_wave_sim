// Copyright (C) 2024  Rhys Mainwaring
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

/// \file WaveProbe.hh
/// \brief A system that reports the wave surface elevation at a fixed
/// (x, y) point in the world, acting like a virtual wave buoy.

#ifndef WAVE_PROBE_HH_
#define WAVE_PROBE_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{
// Forward declaration
class WaveProbePrivate;

/// \brief A world plugin that samples the wave field at a fixed
/// horizontal position and publishes the surface elevation.
///
/// # System Parameters
///
/// `<position>`: The (x, y) world-frame position of the probe. Default
/// is (0, 0).
///
/// `<topic>`: The topic on which to publish the elevation
/// (`gz.msgs.Double`). Default is `/wave_probe/<name>/elevation` where
/// `<name>` is the probe's plugin name (or "probe" if unavailable).
///
/// `<update_rate>`: The rate in Hz at which to sample and publish the
/// elevation. Default is 30.
class WaveProbe:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
  /// \brief Constructor
  public: WaveProbe();

  /// \brief Destructor
  public: ~WaveProbe() override;

  // Documentation inherited
  public: void Configure(
              const Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              EntityComponentManager &_ecm,
              EventManager &_eventMgr) final;

  // Documentation inherited
  public: void PreUpdate(
              const UpdateInfo &_info,
              EntityComponentManager &_ecm) override;

  /// \brief Private data pointer
  private: std::unique_ptr<WaveProbePrivate> dataPtr;
};

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif  // WAVE_PROBE_HH_
