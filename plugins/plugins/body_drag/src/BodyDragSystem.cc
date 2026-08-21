#include "fire/gz/systems/BodyDragSystem.hh"

#include <cmath>
#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Wind.hh>
#include <sdf/Element.hh>

namespace fire::sim::systems
{
class BodyDragSystemPrivate
{
  public: ::gz::sim::Model model{::gz::sim::kNullEntity};
  public: ::gz::sim::Link link{::gz::sim::kNullEntity};
  public: std::string linkName;
  public: ::gz::math::Vector3d dragCoefficient{0.0, 0.0, 0.0};
  public: ::gz::math::Vector3d referenceArea{0.0, 0.0, 0.0};
  public: ::gz::math::Vector3d centerOfPressure{0.0, 0.0, 0.0};
  public: double airDensity{1.225};
  public: bool useWind{true};
  public: bool valid{false};
  public: bool warnedMissingWind{false};
};

namespace
{
bool NonNegative(const ::gz::math::Vector3d &_value)
{
  return _value.X() >= 0.0 && _value.Y() >= 0.0 && _value.Z() >= 0.0;
}
}  // namespace

BodyDragSystem::BodyDragSystem()
    : dataPtr(std::make_unique<BodyDragSystemPrivate>())
{
}

BodyDragSystem::~BodyDragSystem() = default;

void BodyDragSystem::Configure(
    const ::gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ::gz::sim::EntityComponentManager &_ecm,
    ::gz::sim::EventManager &)
{
  this->dataPtr->model = ::gz::sim::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "BodyDragSystem must be attached to a model entity.\n";
    return;
  }

  if (!_sdf->HasElement("link_name"))
  {
    gzerr << "BodyDragSystem requires <link_name>.\n";
    return;
  }

  this->dataPtr->linkName = _sdf->Get<std::string>("link_name");
  const auto linkEntity = this->dataPtr->model.LinkByName(
      _ecm, this->dataPtr->linkName);
  if (linkEntity == ::gz::sim::kNullEntity)
  {
    gzerr << "BodyDragSystem could not find link ["
          << this->dataPtr->linkName << "].\n";
    return;
  }
  this->dataPtr->link = ::gz::sim::Link(linkEntity);

  this->dataPtr->dragCoefficient = _sdf->Get<::gz::math::Vector3d>(
      "drag_coefficient", this->dataPtr->dragCoefficient).first;
  this->dataPtr->referenceArea = _sdf->Get<::gz::math::Vector3d>(
      "reference_area", this->dataPtr->referenceArea).first;
  this->dataPtr->centerOfPressure = _sdf->Get<::gz::math::Vector3d>(
      "center_of_pressure", this->dataPtr->centerOfPressure).first;
  this->dataPtr->airDensity = _sdf->Get<double>(
      "air_density", this->dataPtr->airDensity).first;
  this->dataPtr->useWind = _sdf->Get<bool>(
      "use_wind", this->dataPtr->useWind).first;

  if (!NonNegative(this->dataPtr->dragCoefficient) ||
      !NonNegative(this->dataPtr->referenceArea) ||
      this->dataPtr->airDensity < 0.0)
  {
    gzerr << "BodyDragSystem requires non-negative drag coefficients, "
          << "reference areas, and air density.\n";
    return;
  }

  if (this->dataPtr->dragCoefficient == ::gz::math::Vector3d::Zero ||
      this->dataPtr->referenceArea == ::gz::math::Vector3d::Zero)
  {
    gzwarn << "BodyDragSystem on link [" << this->dataPtr->linkName
           << "] has a zero drag coefficient or reference area vector.\n";
  }

  // Required for velocity at the configured center of pressure.
  this->dataPtr->link.EnableVelocityChecks(_ecm);
  this->dataPtr->valid = true;
}

void BodyDragSystem::PreUpdate(
    const ::gz::sim::UpdateInfo &_info,
    ::gz::sim::EntityComponentManager &_ecm)
{
  if (!this->dataPtr->valid || _info.paused)
    return;

  const auto worldPose = this->dataPtr->link.WorldPose(_ecm);
  const auto pointVelocity = this->dataPtr->link.WorldLinearVelocity(
      _ecm, this->dataPtr->centerOfPressure);
  if (!worldPose || !pointVelocity)
    return;

  ::gz::math::Vector3d windVelocityWorld = ::gz::math::Vector3d::Zero;
  if (this->dataPtr->useWind)
  {
    const auto windEntity = _ecm.EntityByComponents(
        ::gz::sim::components::Wind());
    const auto windVelocity =
        _ecm.Component<::gz::sim::components::WorldLinearVelocity>(windEntity);
    if (windVelocity)
    {
      windVelocityWorld = windVelocity->Data();
    }
    else if (!this->dataPtr->warnedMissingWind)
    {
      gzdbg << "BodyDragSystem found no wind velocity; using still air.\n";
      this->dataPtr->warnedMissingWind = true;
    }
  }

  const auto relativeVelocityWorld = *pointVelocity - windVelocityWorld;
  const auto relativeVelocityBody =
      worldPose->Rot().RotateVectorReverse(relativeVelocityWorld);

  const auto scale = 0.5 * this->dataPtr->airDensity;
  ::gz::math::Vector3d dragBody(
      -scale * this->dataPtr->dragCoefficient.X() *
          this->dataPtr->referenceArea.X() *
          std::abs(relativeVelocityBody.X()) * relativeVelocityBody.X(),
      -scale * this->dataPtr->dragCoefficient.Y() *
          this->dataPtr->referenceArea.Y() *
          std::abs(relativeVelocityBody.Y()) * relativeVelocityBody.Y(),
      -scale * this->dataPtr->dragCoefficient.Z() *
          this->dataPtr->referenceArea.Z() *
          std::abs(relativeVelocityBody.Z()) * relativeVelocityBody.Z());

  const auto dragWorld = worldPose->Rot().RotateVector(dragBody);
  this->dataPtr->link.AddWorldForce(
      _ecm, dragWorld, this->dataPtr->centerOfPressure);
}
}  // namespace fire::sim::systems

GZ_ADD_PLUGIN(
    fire::sim::systems::BodyDragSystem,
    gz::sim::System,
    fire::sim::systems::BodyDragSystem::ISystemConfigure,
    fire::sim::systems::BodyDragSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    fire::sim::systems::BodyDragSystem,
    "fire::gz::systems::BodyDragSystem")
