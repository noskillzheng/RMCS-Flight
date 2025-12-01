#include "./auto_aim_system_v1.hpp"
#include "core/event_bus.hpp"
#include "data/fire_control.hpp"
#include "data/mat_stamped.hpp"
#include "data/sync_data.hpp"
#include "enum/armor_id.hpp"
#include "fire_controller/fire_controller.hpp"
#include "identifier/identifier.hpp"
#include "interfaces/armor_in_camera.hpp"
#include "interfaces/armor_in_image.hpp"
#include "data/predictor_update_package.hpp"
#include "parameters/params_system_v1.hpp"
#include "parameters/profile.hpp"
#include "parameters/rm_parameters.hpp"
#include "predictor/predictor_manager.hpp"

#include "./auto_aim_system_v1.hpp"
#include "./state_machine/state_machine.hpp"

#include <memory>
#include <opencv2/core/mat.hpp>

#include "eigen3/Eigen/Dense"
#include "v1/pnpsolver/armor_pnp_solver.hpp"
#include "v1/sync/syncer.hpp"

#include <cassert>
#include <chrono>

using namespace world_exe;
using namespace v1;
using namespace parameters;
using namespace std::chrono;


class world_exe::v1::SystemV1::Impl{
public:
   explicit Impl(const bool& debug) : debug(debug) {
        std::cerr << "[FORCE] Impl constructor START, debug=" << debug << std::endl;
        time_point_     = std::chrono::steady_clock::now();
        predictor       = std::make_shared<predictor::PredictorManager>(); 
        sync            = std::make_shared<world_exe::v1::Syncer>(seconds(2),6e-6);
        state_machine   = std::make_shared<world_exe::v1::state_machine::StateMachine>();

        std::cerr << "[FORCE] Creating Identifier with model=" << ParamsForSystemV1::szu_model_path()
                  << ", device=" << ParamsForSystemV1::device() << std::endl;
        try {
            identifier      = std::make_shared<identifier::Identifier>(
                                ParamsForSystemV1::szu_model_path(),
                                ParamsForSystemV1::device(),
                                HikCameraProfile::get_width(),
                                HikCameraProfile::get_height());
            std::cerr << "[FORCE] Identifier created successfully" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[FORCE] Identifier creation FAILED: " << e.what() << std::endl;
            throw;
        }

        fire_control    = std::make_shared<fire_control::TracingFireControl>(
                            ParamsForSystemV1::control_delay_in_second(),
                            ParamsForSystemV1::velocity_begin(),
                            ParamsForSystemV1::gravity());
        armor_pnp       = std::make_shared<world_exe::v1::pnpsolver::ArmorIPPEPnPSolver>(
                            Robomaster::LargeArmorObjectPointsOpencv,
                            Robomaster::NormalArmorObjectPointsOpencv);


        fire_control    ->SetTargetCarID(enumeration::CarIDFlag::Base);
        identifier      ->SetTargetColor(ParamsForSystemV1::target_color());
        state_machine   ->SetSwitchFrameCount(4);

        std::cerr << "[SystemV1] Auto-aim system initialized. Target color: "
                  << (ParamsForSystemV1::target_color() ? "RED" : "BLUE") << std::endl;


       core::EventBus::Subscript<data::MatStamped>
       (ParamsForSystemV1::raw_image_event,             [this](const auto& mat_stamped){solve(mat_stamped);});
       core::EventBus::Subscript<data::CameraGimbalMuzzleSyncData>
       (ParamsForSystemV1::camera_capture_transforms,   [this](const auto& pkg){set_transfroms(pkg);});

       std::cerr << "[FORCE] Impl constructor END, subscriptions done" << std::endl;
    }


    
    void solve(const data::MatStamped& mat_stamped){
        const auto& timestamp       = mat_stamped.stamp;
        const auto& raw             = mat_stamped.mat;

        const auto& [armors, flag]  = identifier->identify(raw, timestamp);

        if(flag == enumeration::ArmorIdFlag::Unknow) return;
        if (!armors) return;

        const auto& solved          = armor_pnp->SolvePnp(armors);
        const auto& [pack, check]   = sync->get_data(solved->GetTimeStamp());

        if(!check) [[unlikely]]     return;

        time_point_                 = std::chrono::steady_clock::now();
        state_machine               ->Update(flag);
        const auto& fire_targets    = state_machine->GetAllowdToFires();
        auto combined               = std::make_shared<data::PredictorUpdatePackage>(pack, solved);
        predictor                   ->Update(combined);
        const auto& time            = combined->GetTimeStamp();
        const auto& armor3d         = predictor->Predict(fire_targets,time);
        fire_control                ->set_armor(armor3d);

        if (fire_targets != enumeration::ArmorIdFlag::None)
            fire_control->SetPredictor(predictor->GetPredictor(fire_targets));

        core::EventBus::Publish<data::FireControl>(ParamsForSystemV1::fire_control_event, control());

        if(!debug) [[likely]]       return;
        
        const auto& target_id       = fire_control->GetAttackCarId();
        if (target_id != enumeration::ArmorIdFlag::None)
            predictor->GetPredictor(target_id);

        core::EventBus::Publish<enumeration::CarIDFlag>(
            parameters::ParamsForSystemV1::car_id_identify_event, 
            flag);
        core::EventBus::Publish<std::shared_ptr<interfaces::IArmorInImage>>(
            parameters::ParamsForSystemV1::armors_in_image_identify_event, 
            armors);
        core::EventBus::Publish<std::shared_ptr<world_exe::interfaces::IArmorInCamera>>(
            parameters::ParamsForSystemV1::armors_in_camera_pnp_event, 
            solved);
        core::EventBus::Publish<std::shared_ptr<data::PredictorUpdatePackage>>(
            parameters::ParamsForSystemV1::tracker_update_event, 
            combined);
        core::EventBus::Publish<enumeration::CarIDFlag>(
            parameters::ParamsForSystemV1::car_tracing_event, 
            state_machine->GetAllowdToFires());
    }

    void set_transfroms(const data::CameraGimbalMuzzleSyncData& data){
        sync->set_data(data);
    }

    data::FireControl control(){
        return fire_control->CalculateTarget(std::chrono::duration_cast<seconds>(std::chrono::steady_clock::now() - time_point_));
    }

    Impl(const Impl&)       = delete;
    ~Impl()                         = default;

private:
    std::shared_ptr<predictor::PredictorManager>                    predictor;
    std::shared_ptr<identifier::Identifier>                         identifier;
    std::shared_ptr<fire_control::TracingFireControl>               fire_control;
    std::shared_ptr<world_exe::v1::state_machine::StateMachine>     state_machine;
    std::shared_ptr<world_exe::v1::pnpsolver::ArmorIPPEPnPSolver>   armor_pnp;
    std::shared_ptr<world_exe::v1::Syncer>                          sync;
    time_point<steady_clock, nanoseconds>                           time_point_;
    const bool debug = false;
};

std::unique_ptr<SystemV1> SystemV1::v1;

void world_exe::v1::SystemV1::build(const bool& debug) {
    if (v1 != nullptr) {
        return;
    }
    v1 = std::make_unique<SystemV1>(debug);
    std::cerr << "[FORCE] SystemV1 created, ptr=" << (void*)v1.get() << std::endl;
}
SystemV1::SystemV1(const bool& debug){
    instance_ = std::make_unique<Impl>(debug);
    std::cerr << "[FORCE] SystemV1 constructor finished, Impl created" << std::endl;
}

SystemV1::~SystemV1(){};
