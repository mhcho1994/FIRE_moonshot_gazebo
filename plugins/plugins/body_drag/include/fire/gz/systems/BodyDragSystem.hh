#ifndef FIRE_GZ_SYSTEMS_BODYDRAGSYSTEM_HH_
#define FIRE_GZ_SYSTEMS_BODYDRAGSYSTEM_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace fire::sim::systems
{
class BodyDragSystemPrivate;

/// \brief Applies axis-dependent quadratic aerodynamic drag to a link.
class BodyDragSystem final :
    public ::gz::sim::System,
    public ::gz::sim::ISystemConfigure,
    public ::gz::sim::ISystemPreUpdate
{
  public: BodyDragSystem();
  public: ~BodyDragSystem() override;

  public: void Configure(
      const ::gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      ::gz::sim::EntityComponentManager &_ecm,
      ::gz::sim::EventManager &_eventMgr) override;

  public: void PreUpdate(
      const ::gz::sim::UpdateInfo &_info,
      ::gz::sim::EntityComponentManager &_ecm) override;

  private: std::unique_ptr<BodyDragSystemPrivate> dataPtr;
};
}  // namespace fire::sim::systems

#endif  // FIRE_GZ_SYSTEMS_BODYDRAGSYSTEM_HH_
