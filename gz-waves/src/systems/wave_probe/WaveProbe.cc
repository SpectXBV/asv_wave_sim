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

#include "WaveProbe.hh"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <gz/common/Profiler.hh>
#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/components/Name.hh>
#include <gz/sim/World.hh>

#include <sdf/Element.hh>

#include "gz/waves/Utilities.hh"
#include "gz/waves/Wavefield.hh"
#include "gz/waves/components/Wavefield.hh"

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{

class WaveProbePrivate
{
  /// \brief Locate the wavefield entity and cache a weak reference to it.
  public: bool InitWavefield(EntityComponentManager &_ecm);

  /// \brief Sample the wave elevation at the probe position and publish.
  public: void Update(const UpdateInfo &_info);

  /// \brief Name of the world this probe belongs to.
  public: std::string worldName;

  /// \brief The (x, y) world-frame position to sample.
  public: gz::math::Vector2d position{0.0, 0.0};

  /// \brief Topic on which to publish the surface elevation.
  public: std::string topic{"/wave_probe/elevation"};

  /// \brief Update rate [Hz].
  public: double updateRate{30.0};

  /// \brief Previous update (sim) time [s].
  public: double lastUpdateTime{0.0};

  /// \brief Flag set once the wavefield has been located.
  public: bool validConfig{false};

  /// \brief The name of the wavefield entity to look up.
  public: std::string wavefieldEntityName{"wavefield"};

  /// \brief The wavefield entity.
  public: Entity wavefieldEntity{kNullEntity};

  /// \brief Weak reference to the wavefield.
  public: waves::WavefieldConstWeakPtr wavefield;

  /// \brief Transport node for publishing.
  public: transport::Node node;

  /// \brief Publisher for the surface elevation.
  public: transport::Node::Publisher pub;

  /// \brief Marker shown in the GUI at the probe's (x, y) position,
  /// bobbing at the current surface elevation.
  public: msgs::Marker marker;
};

/////////////////////////////////////////////////
WaveProbe::WaveProbe() : System(),
    dataPtr(std::make_unique<WaveProbePrivate>())
{
}

/////////////////////////////////////////////////
WaveProbe::~WaveProbe()
{
}

/////////////////////////////////////////////////
void WaveProbe::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  GZ_PROFILE("WaveProbe::Configure");

  World world(_entity);
  if (!world.Valid(_ecm))
  {
    gzerr << "The WaveProbe system must be attached to the world entity. "
        << "Failed to initialize." << std::endl;
    return;
  }
  this->dataPtr->worldName = world.Name(_ecm).value_or("");

  this->dataPtr->position = waves::Utilities::SdfParamVector2d(
      *_sdf, "position", this->dataPtr->position);

  this->dataPtr->updateRate = waves::Utilities::SdfParamDouble(
      *_sdf, "update_rate", this->dataPtr->updateRate);

  this->dataPtr->topic = waves::Utilities::SdfParamString(
      *_sdf, "topic", this->dataPtr->topic);

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<msgs::Double>(this->dataPtr->topic);

  // GUI marker: a small sphere bobbing at the probe's (x, y) position so
  // it's visible in the sim, namespaced by topic so multiple probes in
  // the same world don't collide.
  std::string markerNs = this->dataPtr->topic;
  std::replace(markerNs.begin(), markerNs.end(), '/', '_');
  this->dataPtr->marker.set_ns("wave_probe" + markerNs);
  this->dataPtr->marker.set_id(1);
  this->dataPtr->marker.set_action(msgs::Marker::ADD_MODIFY);
  this->dataPtr->marker.set_type(msgs::Marker::SPHERE);
  this->dataPtr->marker.set_visibility(msgs::Marker::GUI);
  msgs::Set(this->dataPtr->marker.mutable_scale(),
      gz::math::Vector3d(0.6, 0.6, 0.6));
  msgs::Set(this->dataPtr->marker.mutable_material()->mutable_ambient(),
      gz::math::Color(1.0, 0.2, 0.0, 1.0));
  msgs::Set(this->dataPtr->marker.mutable_material()->mutable_diffuse(),
      gz::math::Color(1.0, 0.2, 0.0, 1.0));

  gzmsg << "WaveProbe: publishing surface elevation at ("
      << this->dataPtr->position.X() << ", " << this->dataPtr->position.Y()
      << ") on topic [" << this->dataPtr->topic << "]\n";
}

//////////////////////////////////////////////////
void WaveProbe::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("WaveProbe::PreUpdate");

  if (this->dataPtr->worldName.empty())
    return;

  if (!this->dataPtr->validConfig)
  {
    if (!this->dataPtr->InitWavefield(_ecm))
      return;
    this->dataPtr->validConfig = true;
  }

  if (_info.paused)
    return;

  this->dataPtr->Update(_info);
}

//////////////////////////////////////////////////
bool WaveProbePrivate::InitWavefield(EntityComponentManager &_ecm)
{
  this->wavefieldEntity =
      _ecm.EntityByComponents(components::Name(this->wavefieldEntityName));
  if (this->wavefieldEntity == kNullEntity)
  {
    gzwarn << "WaveProbe: no wavefield found, "
        << "no surface elevation will be reported\n";
    return false;
  }

  auto comp = _ecm.Component<waves::components::Wavefield>(
      this->wavefieldEntity);
  if (comp)
  {
    this->wavefield = comp->Data();
  }

  if (!this->wavefield.lock())
  {
    gzwarn << "WaveProbe: invalid wavefield, "
        << "no surface elevation will be reported\n";
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
void WaveProbePrivate::Update(const UpdateInfo &_info)
{
  // Throttle to the configured update rate.
  double simTime = std::chrono::duration<double>(_info.simTime).count();
  double updatePeriod = 1.0 / this->updateRate;
  if ((simTime - this->lastUpdateTime) < updatePeriod)
    return;
  this->lastUpdateTime = simTime;

  auto lockedWavefield = this->wavefield.lock();
  if (!lockedWavefield)
    return;

  Eigen::Vector3d point(this->position.X(), this->position.Y(), 0.0);
  double elevation = 0.0;
  if (!lockedWavefield->Height(point, elevation))
    return;

  msgs::Double msg;
  msg.set_data(elevation);
  this->pub.Publish(msg);

  msgs::Set(this->marker.mutable_pose(),
      gz::math::Pose3d(this->position.X(), this->position.Y(), elevation,
          0, 0, 0));
  this->node.Request("/marker", this->marker);
}

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

//////////////////////////////////////////////////
GZ_ADD_PLUGIN(gz::sim::systems::WaveProbe,
              gz::sim::System,
              gz::sim::systems::WaveProbe::ISystemConfigure,
              gz::sim::systems::WaveProbe::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::WaveProbe,
                   "gz::sim::systems::WaveProbe")
