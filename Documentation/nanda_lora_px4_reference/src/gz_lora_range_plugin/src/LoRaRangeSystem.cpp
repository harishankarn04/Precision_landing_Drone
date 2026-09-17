#include <gz/sim/System.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <random>
#include <chrono>

namespace gz_lora_range_plugin {

class LoRaRangeSystem : public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPostUpdate {
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &/*_eventMgr*/) override {
    
    tag_entity_ = _entity;
    
    if (_sdf->HasElement("update_rate")) {
      update_rate_ = _sdf->Get<double>("update_rate");
    }
    
    if (_sdf->HasElement("range_noise_sigma")) {
      range_noise_sigma_ = _sdf->Get<double>("range_noise_sigma");
    }
    
    if (_sdf->HasElement("anchor_prefix")) {
      anchor_prefix_ = _sdf->Get<std::string>("anchor_prefix");
    }
    
    topic_name_ = "/lora/ranging";
    pub_ = transport_node_.Advertise<gz::msgs::StringMsg>(topic_name_);
    
    // Seed random generator
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator_.seed(seed);
    
    gzmsg << "LoRaRangeSystem configured for tag entity: " << tag_entity_ << std::endl;
  }

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override {
    if (_info.paused) return;

    auto now = _info.simTime;
    if (update_rate_ > 0) {
      auto dt = now - last_pub_time_;
      if (std::chrono::duration_cast<std::chrono::duration<double>>(dt).count() < (1.0 / update_rate_)) {
        return;
      }
    }
    last_pub_time_ = now;

    // Get tag world pose
    auto tag_pose_comp = _ecm.Component<gz::sim::components::Pose>(tag_entity_);
    if (!tag_pose_comp) return;
    
    // Need to resolve world pose (simplification: assume tag is directly under a model, or we walk up the tree)
    // For proper world pose, we should use the world pose component if available or calculate it.
    // In Gazebo Harmonic, we can use gz::sim::worldPose(tag_entity_, _ecm);
    // But since this is a system, let's just grab the pose of the parent model for now, 
    // or assume tag_pose is relative to parent and parent is relative to world.
    
    // Quick hack for finding anchors (since iterating all entities is slow, we should cache them)
    if (anchors_.empty()) {
      _ecm.Each<gz::sim::components::Name, gz::sim::components::Pose>(
        [&](const gz::sim::Entity &_ent,
            const gz::sim::components::Name *_name,
            const gz::sim::components::Pose */*_pose*/) -> bool {
          if (_name->Data().find(anchor_prefix_) == 0) {
            anchors_.push_back(_ent);
            gzmsg << "Found anchor: " << _name->Data() << " (Entity " << _ent << ")" << std::endl;
          }
          return true;
        });
    }

    // Since worldPose requires extra includes, let's assume the components::Pose of anchors and tags
    // are sufficient if they are attached to the world, OR we just use basic relative math if they are in the same model.
    // For our use case, anchors are on the ground robot, tag is on the drone. 
    // So we MUST get World poses. 
    // We'll use the pose from the ECM, which might be local. We need to accumulate.
    
    auto get_world_pose = [&_ecm](gz::sim::Entity ent) -> gz::math::Pose3d {
      gz::math::Pose3d pose;
      while (ent != gz::sim::kNullEntity) {
        auto p = _ecm.Component<gz::sim::components::Pose>(ent);
        if (p) pose = p->Data() * pose;
        auto parent = _ecm.Component<gz::sim::components::ParentEntity>(ent);
        if (parent) ent = parent->Data();
        else break;
      }
      return pose;
    };

    gz::math::Pose3d tag_world_pose = get_world_pose(tag_entity_);

    std::normal_distribution<double> dist(0.0, range_noise_sigma_);

    for (size_t i = 0; i < anchors_.size(); ++i) {
      gz::math::Pose3d anchor_world_pose = get_world_pose(anchors_[i]);
      
      double true_range = tag_world_pose.Pos().Distance(anchor_world_pose.Pos());
      double noisy_range = true_range + dist(generator_);
      if (noisy_range < 0) noisy_range = 0;
      
      // Simple path loss model for RSSI
      double rssi = -40.0 - 20.0 * std::log10(std::max(true_range, 1.0));
      double snr = 10.0; // dummy SNR

      // Create JSON string
      std::string json = "{";
      json += "\"anchor_id\":" + std::to_string(i) + ",";
      json += "\"tag_id\":0,";
      json += "\"range\":" + std::to_string(noisy_range) + ",";
      json += "\"rssi\":" + std::to_string(rssi) + ",";
      json += "\"snr\":" + std::to_string(snr) + ",";
      json += "\"range_error\":" + std::to_string(range_noise_sigma_);
      json += "}";

      gz::msgs::StringMsg msg;
      msg.set_data(json);
      pub_.Publish(msg);
    }
  }

private:
  gz::sim::Entity tag_entity_{gz::sim::kNullEntity};
  std::string anchor_prefix_ = "lora_anchor";
  double update_rate_ = 10.0;
  double range_noise_sigma_ = 1.5;
  std::vector<gz::sim::Entity> anchors_;
  std::chrono::steady_clock::duration last_pub_time_{0};
  
  std::default_random_engine generator_;
  
  std::string topic_name_;
  gz::transport::Node transport_node_;
  gz::transport::Node::Publisher pub_;
};

} // namespace gz_lora_range_plugin

GZ_ADD_PLUGIN(
  gz_lora_range_plugin::LoRaRangeSystem,
  gz::sim::System,
  gz_lora_range_plugin::LoRaRangeSystem::ISystemConfigure,
  gz_lora_range_plugin::LoRaRangeSystem::ISystemPostUpdate
)
GZ_ADD_PLUGIN_ALIAS(gz_lora_range_plugin::LoRaRangeSystem, "gz_lora_range_plugin::LoRaRangeSystem")
