#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/int32.pb.h>
#include <algorithm>
#include <gz/sim/components/JointPositionReset.hh>


#define _USE_MATH_DEFINES
#include <cmath>


using namespace gz;
using namespace sim;

class controller : public System, 
                     public ISystemConfigure,
                     public ISystemPreUpdate 
{
public:
    void Configure(const Entity &_entity,
               const std::shared_ptr<const sdf::Element> &,
               EntityComponentManager &_ecm,
               EventManager &) override
    {
        this->model = Model(_entity);

        this->cartJointName = "cart_linear";
        this->poleJointName = "pole_revolute";

        // Get joint entities
        this->cartJoint = this->model.JointByName(_ecm, this->cartJointName);
        this->poleJoint = this->model.JointByName(_ecm, this->poleJointName);

        if (this->cartJoint == kNullEntity)
        {
            gzerr << "Cart joint not found!\n";
            return;
        }

        if (this->poleJoint == kNullEntity)
        {
            gzerr << "Pole joint not found!\n";
            return;
        }

        if (!_ecm.EntityHasComponentType(
                this->cartJoint,
                components::JointPosition().TypeId()))
        {
            _ecm.CreateComponent(this->cartJoint,
                                components::JointPosition());
        }

        if (!_ecm.EntityHasComponentType(
                this->poleJoint,
                components::JointPosition().TypeId()))
        {
            _ecm.CreateComponent(this->poleJoint,
                                components::JointPosition());
        }
        _ecm.CreateComponent(this->poleJoint, components::JointPositionReset({M_PI}));

        if (!_ecm.EntityHasComponentType(
                this->cartJoint,
                components::JointForceCmd().TypeId()))
        {
            _ecm.CreateComponent(this->cartJoint,
                                components::JointForceCmd({0.0}));
        }

        this->node = std::make_unique<gz::transport::Node>();
        bool ok = this->node->Subscribe(
            "/keyboard/keypress",
            &controller::OnKeyPress,
            this);

        if (ok){
            gzmsg << "Subscribed to /keyboard/keypress\n";
        }
        else{
            gzerr << "FAILED to subscribe\n";
        }

        // Reset controller state
        integrator1 = 0.0;
        integrator2 = 0.0;
        lastError2 = 0.0;
        outerTimer = 0.0;

        gzmsg << "============================ PLUGIN LOADED ============================\n";
    }


    void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override 
    {
        if(_info.paused){
            return;
        }
        double dt = std::chrono::duration<double>(_info.dt).count();
        if(dt <= 0.0){
            return;
        }

        outerTimer += dt;

        auto poleComp = _ecm.Component<components::JointPosition>(this->poleJoint);
        auto posComp = _ecm.Component<components::JointPosition>(this->cartJoint);            
        // Reading Angle
        if(poleComp && !poleComp->Data().empty()){
            theta = wrapToPi(poleComp->Data()[0]);
            theta_raw = poleComp->Data()[0];
        }

        // Reading Position
        if (posComp && !posComp->Data().empty())
        {
            y = -posComp->Data()[0];
        }
        double pole_error = theta_ref - theta;
        double omega = (theta_raw - prev_theta) / dt;
        double pole_d_raw = -omega;
        prev_theta = theta_raw;
        

        double cart_error;
        // Controll Loop 2 (position) @200Hz
        if(outerTimer >= outerDt) {
            cart_error = y_des - y;
            
            double cart_d_raw = (cart_error - lastError2) / outerDt;
            cart_d_filtered = d_alpha_outer * cart_d_filtered + (1.0 - d_alpha_outer) * cart_d_raw;
            lastError2 = cart_error;
            
            if (theta_ref <= theta_clamp && theta_ref >= -theta_clamp){
                integrator2 += cart_error * outerDt;
            }
            double theta_ref_raw = Kp2 * cart_error + Ki2 * integrator2 + Kd2 * cart_d_filtered;
            theta_ref = std::clamp(theta_ref_raw, -theta_clamp, theta_clamp);
            outerTimer -= outerDt;
        }


        double E = 0.5 * (l*l) * omega*omega
         + g*l*(1.0 + std::cos(theta));

        double E_des = 2.0 * g * l;
        
        double u_swing = kE * (E_des - E) * omega * std::cos(theta);

        // BALANCE MODE
        pole_d_filtered = d_alpha_inner * pole_d_filtered 
                        + (1.0 - d_alpha_inner) * pole_d_raw;

        integrator1 += pole_error * dt;
        integrator1 = std::clamp(integrator1, -intClamp1, intClamp1);

        double u_balance = Kp1 * pole_error 
            + Ki1 * integrator1 
            + Kd1 * pole_d_filtered;

  
        output = u_balance;
  
        if(abs(theta) >= M_PI/3){
            double w = std::exp(-(theta*theta)/0.5);
            output = (1.0 - w) * u_swing + w * u_balance;
            static int kick_dir = 1;
            static int in_bottom_prev = false;

            bool in_bottom = ( (M_PI - std::abs(theta)) < 0.08 ) && (std::abs(omega) < 0.2);

            if (!in_bottom_prev && in_bottom)
            {
                kick_dir = (theta >= 0.0) ? 1 : -1;
            }

            if (in_bottom)
                output = 20.0 * kick_dir;

            in_bottom_prev = in_bottom;
        }
        
        if(abs(omega) >= 8.0) output = 0;
        // Applying Force
        output = std::clamp(output, -maxForce, maxForce);
        // output = 40;
        auto forceComp = _ecm.Component<components::JointForceCmd>(cartJoint);
        if(forceComp){
            forceComp->Data()[0] = output;
            _ecm.SetChanged(cartJoint, components::JointForceCmd().TypeId(), ComponentState::OneTimeChange);
        }

        // ======= DEBUG =======
        // if (_info.iterations % 50 == 0){
        //     gzmsg << theta_ref << std::endl;            
        // }
        
    }

private:
    Model model;
    std::string cartJointName;
    std::string poleJointName;
    Entity cartJoint{kNullEntity};
    Entity poleJoint{kNullEntity};
    Entity cartLink{kNullEntity};

    std::unique_ptr<gz::transport::Node> node;
    double y_des = 0.0;
    

    double y = 0.0;
    double LINEAR_LENGTH = 30;
    double theta = 0.0;
    double theta_raw = 0.0;

    double prev_theta = 0.0;
    
    double g = 9.81;
    double l = 0.5;
    double kE = 1; 

    double output = 0.0;
    double Kp1 = 240.0;
    double Ki1 = 0.0;
    double integrator1 = 0.0;
    double Kd1 = 120.0;
    // double lastError1 = 0.0;
    double intClamp1 = 7.0;
    double theta_clamp = 0.22;

    double uprightThreshold = M_PI / 2;

    double pole_d_filtered = 0.0;

    double theta_ref = 0.0;
    double Kp2 = 0.07;
    double Ki2 = 0.0;
    double integrator2 = 0.0;
    double Kd2 = 0.1;


    double lastError2 = 0.0;
    double intClamp2 = 7.0;
    double cart_d_filtered = 0.0;

    double maxForce = 100.0;
    double outerTimer = 0.0;
    double outerDt = 0.005;

    double d_alpha_inner = 0.87; // 1 kHz
    double d_alpha_outer = 0.7; 


    double wrapToPi(double a)
    {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    void OnKeyPress(const gz::msgs::Int32 &_msg)
    {
        int key = _msg.data();

        if (key == 'w' || key == 'W')
        {
            y_des += 1.0;
            y_des = std::clamp(y_des, -5.0, 5.0);
            gzmsg << "y_des increased: " << y_des << "\n";
        }
        else if (key == 's' || key == 'S')
        {
            y_des -= 1.0;
            y_des = std::clamp(y_des, -5.0, 5.0);
            gzmsg << "y_des decreased: " << y_des << "\n";
        }
    }
};

GZ_ADD_PLUGIN(controller, gz::sim::System, controller::ISystemConfigure, controller::ISystemPreUpdate)