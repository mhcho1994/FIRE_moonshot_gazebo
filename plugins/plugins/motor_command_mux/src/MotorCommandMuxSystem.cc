#include "fire/gz/systems/MotorCommandMuxSystem.hh"

#include <algorithm>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace fire::sim::systems
{
class MotorCommandMuxSystemPrivate
{
  public: void OnCommand(const ::gz::msgs::Double &_msg, std::size_t _index)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->commands[_index] = std::clamp(_msg.data(), 0.0, this->maxVelocity);
  }

  public: ::gz::sim::Model model{::gz::sim::kNullEntity};
  public: ::gz::transport::Node node;
  public: ::gz::transport::Node::Publisher publisher;
  public: std::vector<double> commands;
  public: std::mutex mutex;
  public: double maxVelocity{1000.0};
  public: bool valid{false};
};

MotorCommandMuxSystem::MotorCommandMuxSystem()
    : dataPtr(std::make_unique<MotorCommandMuxSystemPrivate>())
{
}

MotorCommandMuxSystem::~MotorCommandMuxSystem() = default;

void MotorCommandMuxSystem::Configure(
    const ::gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ::gz::sim::EntityComponentManager &_ecm,
    ::gz::sim::EventManager &)
{
  this->dataPtr->model = ::gz::sim::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "MotorCommandMuxSystem must be attached to a model entity.\n";
    return;
  }

  this->dataPtr->maxVelocity = _sdf->Get<double>(
      "max_velocity", this->dataPtr->maxVelocity).first;
  if (this->dataPtr->maxVelocity <= 0.0)
  {
    gzerr << "MotorCommandMuxSystem requires a positive <max_velocity>.\n";
    return;
  }

  std::vector<std::string> inputTopics;
  const auto sdf = _sdf->Clone();
  if (sdf->HasElement("input_topic"))
  {
    auto input = sdf->GetElement("input_topic");
    while (input)
    {
      inputTopics.push_back(input->Get<std::string>());
      input = input->GetNextElement("input_topic");
    }
  }
  if (inputTopics.empty())
  {
    gzerr << "MotorCommandMuxSystem requires at least one <input_topic>.\n";
    return;
  }

  const auto outputSubTopic = _sdf->Get<std::string>(
      "output_sub_topic", "command/motor_speed").first;
  const auto outputTopic = "/" + this->dataPtr->model.Name(_ecm) + "/" +
      outputSubTopic;
  this->dataPtr->publisher =
      this->dataPtr->node.Advertise<::gz::msgs::Actuators>(outputTopic);
  if (!this->dataPtr->publisher)
  {
    gzerr << "MotorCommandMuxSystem failed to advertise ["
          << outputTopic << "].\n";
    return;
  }

  this->dataPtr->commands.assign(inputTopics.size(), 0.0);
  for (std::size_t i = 0; i < inputTopics.size(); ++i)
  {
    std::function<void(const ::gz::msgs::Double &)> callback =
        [this, i](const ::gz::msgs::Double &_msg)
        {
          this->dataPtr->OnCommand(_msg, i);
        };
    const bool subscribed = this->dataPtr->node.Subscribe<::gz::msgs::Double>(
        inputTopics[i], callback);
    if (!subscribed)
    {
      gzerr << "MotorCommandMuxSystem failed to subscribe to ["
            << inputTopics[i] << "].\n";
      return;
    }
  }

  gzmsg << "MotorCommandMuxSystem publishing " << inputTopics.size()
        << " motor commands on [" << outputTopic << "].\n";
  this->dataPtr->valid = true;
}

void MotorCommandMuxSystem::PreUpdate(
    const ::gz::sim::UpdateInfo &_info,
    ::gz::sim::EntityComponentManager &)
{
  if (!this->dataPtr->valid || _info.paused)
    return;

  ::gz::msgs::Actuators message;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    for (const double command : this->dataPtr->commands)
      message.add_velocity(command);
  }
  this->dataPtr->publisher.Publish(message);
}
}  // namespace fire::sim::systems

GZ_ADD_PLUGIN(
    fire::sim::systems::MotorCommandMuxSystem,
    gz::sim::System,
    fire::sim::systems::MotorCommandMuxSystem::ISystemConfigure,
    fire::sim::systems::MotorCommandMuxSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    fire::sim::systems::MotorCommandMuxSystem,
    "fire::gz::systems::MotorCommandMuxSystem")
