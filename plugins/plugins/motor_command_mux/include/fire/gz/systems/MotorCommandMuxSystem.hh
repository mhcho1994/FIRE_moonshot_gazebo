#ifndef FIRE_GZ_SYSTEMS_MOTORCOMMANDMUXSYSTEM_HH_
#define FIRE_GZ_SYSTEMS_MOTORCOMMANDMUXSYSTEM_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace fire::sim::systems
{
class MotorCommandMuxSystemPrivate;

/// \brief Converts per-motor Double commands into an Actuators message.
class MotorCommandMuxSystem final :
    public ::gz::sim::System,
    public ::gz::sim::ISystemConfigure,
    public ::gz::sim::ISystemPreUpdate
{
  public: MotorCommandMuxSystem();
  public: ~MotorCommandMuxSystem() override;

  public: void Configure(
      const ::gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      ::gz::sim::EntityComponentManager &_ecm,
      ::gz::sim::EventManager &_eventMgr) override;

  public: void PreUpdate(
      const ::gz::sim::UpdateInfo &_info,
      ::gz::sim::EntityComponentManager &_ecm) override;

  private: std::unique_ptr<MotorCommandMuxSystemPrivate> dataPtr;
};
}  // namespace fire::sim::systems

#endif  // FIRE_GZ_SYSTEMS_MOTORCOMMANDMUXSYSTEM_HH_
