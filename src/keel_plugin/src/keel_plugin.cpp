#include <algorithm>
#include <string>

#include <gazebo/common/Assert.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/transport/transport.hh>
#include <keel_plugin/keel_plugin.h>

#include <ros/publisher.h>
#include <sensor_msgs/JointState.h>

using namespace gazebo;

int count(0);


  GZ_REGISTER_MODEL_PLUGIN(Rudderplugin)

/////////////////////////////////////////////////
Rudderplugin::Rudderplugin() : cla(1.0), cda(0.01), cma(0.01), rho(1.2041)
{
  ROS_INFO("------------------------------Rudderplugin OBJECT CREATED!!!!");
  this->cp = math::Vector3(0, 0, 0);
  this->forward = math::Vector3(1, 0, 0);
  this->upward = math::Vector3(0, 0, 1);
  this->area = 1.0;
  this->alpha0 = 0.0;

  // 90 deg stall
  this->alphaStall = 0.5*M_PI;
  this->claStall = 0.0;

  /// \TODO: what's flat plate drag?
  this->cdaStall = 1.0;
  this->cmaStall = 0.0;
}

/////////////////////////////////////////////////
void Rudderplugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROS_INFO("------------------------------Rudderplugin loaded!!!!");

  GZ_ASSERT(_model, "Rudderplugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "Rudderplugin _sdf pointer is NULL");
  this->model = _model;
  this->modelName = _model->GetName();
  this->sdf = _sdf;
  rosnode_ = ros::NodeHandle(modelName);

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "Rudderplugin world pointer is NULL");

  this->physics = this->world->GetPhysicsEngine();
  GZ_ASSERT(this->physics, "Rudderplugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "Rudderplugin _sdf pointer is NULL");

  if (_sdf->HasElement("a0"))
    this->alpha0 = _sdf->Get<double>("a0");

  if (_sdf->HasElement("cla"))
    this->cla = _sdf->Get<double>("cla");

  if (_sdf->HasElement("cda"))
    this->cda = _sdf->Get<double>("cda");

  if (_sdf->HasElement("cma"))
    this->cma = _sdf->Get<double>("cma");

  if (_sdf->HasElement("alpha_stall"))
    this->alphaStall = _sdf->Get<double>("alpha_stall");

  if (_sdf->HasElement("cla_stall"))
    this->claStall = _sdf->Get<double>("cla_stall");

  if (_sdf->HasElement("cda_stall"))
    this->cdaStall = _sdf->Get<double>("cda_stall");

  if (_sdf->HasElement("cma_stall"))
    this->cmaStall = _sdf->Get<double>("cma_stall");

  if (_sdf->HasElement("cp"))
    this->cp = _sdf->Get<math::Vector3>("cp");

  // blade forward (-drag) direction in link frame
  if (_sdf->HasElement("forward"))
    this->forward = _sdf->Get<math::Vector3>("forward");

  // blade upward (+lift) direction in link frame
  if (_sdf->HasElement("upward"))
    this->upward = _sdf->Get<math::Vector3>("upward");

  if (_sdf->HasElement("area"))
    this->area = _sdf->Get<double>("area");

  if (_sdf->HasElement("air_density"))
    this->rho = _sdf->Get<double>("air_density");

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    this->linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(this->linkName);
  }
  waterCurrent = math::Vector3(0,0,0);
}

/////////////////////////////////////////////////
void Rudderplugin::Init()
{
  current_subscriber_ = rosnode_.subscribe("/gazebo/current", 1, &Rudderplugin::ReadWaterCurrent, this);
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&Rudderplugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void Rudderplugin::OnUpdate()
{
//ROS_INFO("------------------------------Rudderplugin OnUpdate!!!!");
	
  // get linear velocity at cp in inertial frame
  //math::Vector3 vel = math::Vector3(1,0,0)- this->link->GetWorldLinearVel(this->cp);
  math::Vector3 vel = this->link->GetWorldLinearVel(this->cp) - waterCurrent;
  // math::Vector3 vel = waterCurrent - this->link->GetWorldLinearVel(this->cp);

//  math::Vector3 vel = this->link->GetWorldLinearVel(this->cp);
//std::cerr<<"\n vel: "<<vel;
  // smoothing
  // double e = 0.8;
  // this->velSmooth = e*vel + (1.0 - e)*velSmooth;
  // vel = this->velSmooth;

  if (vel.GetLength() <= 0.01)
    return;

  // pose of body
  math::Pose pose = this->link->GetWorldPose();

  // rotate forward and upward vectors into inertial frame
  math::Vector3 forwardI = pose.rot.RotateVector(this->forward);
  math::Vector3 upwardI = pose.rot.RotateVector(this->upward);
//std::cerr<<"\n pose: "<<pose<<" forwardI: "<<forwardI<<" upwardI: "<<upwardI;

  // ldNormal vector to lift-drag-plane described in inertial frame
  math::Vector3 ldNormal = forwardI.Cross(upwardI).Normalize();

  // check sweep (angle between vel and lift-drag-plane)
  double sinSweepAngle = ldNormal.Dot(vel) / vel.GetLength();

  // get cos from trig identity
  double cosSweepAngle2 = (1.0 - sinSweepAngle * sinSweepAngle);
  this->sweep = asin(sinSweepAngle);

  // truncate sweep to within +/-90 deg
  while (fabs(this->sweep) > 0.5 * M_PI)
    this->sweep = this->sweep > 0 ? this->sweep - M_PI
                                  : this->sweep + M_PI;

  // angle of attack is the angle between
  // vel projected into lift-drag plane
  //  and
  // forward vector
  //
  // projected = ldNormal Xcross ( vector Xcross ldNormal)
  //
  // so,
  // velocity in lift-drag plane (expressed in inertial frame) is:
  math::Vector3 velInLDPlane = ldNormal.Cross(vel.Cross(ldNormal));
//std::cerr<<"\n velInLDPlane: "<<velInLDPlane;

  // get direction of drag
  math::Vector3 dragDirection = -velInLDPlane;
  dragDirection.Normalize();

  // get direction of lift
  math::Vector3 liftDirection = ldNormal.Cross(velInLDPlane);
  liftDirection.Normalize();
//std::cerr<<"\n liftDirection: "<<liftDirection;

  // get direction of moment
  math::Vector3 momentDirection = ldNormal;

  double cosAlpha = math::clamp(
    forwardI.Dot(velInLDPlane) /
    (forwardI.GetLength() * velInLDPlane.GetLength()), -1.0, 1.0);
  // std::cerr << "ca " << forwardI.Dot(velInLDPlane) /
  //   (forwardI.GetLength() * velInLDPlane.GetLength()) << "\n";

  // get sign of alpha
  // take upwards component of velocity in lift-drag plane.
  // if sign == upward, then alpha is negative
  double alphaSign = -upwardI.Dot(velInLDPlane)/
    (upwardI.GetLength() + velInLDPlane.GetLength());

  // double sinAlpha = sqrt(1.0 - cosAlpha * cosAlpha);
  if (alphaSign > 0.0)
    this->alpha = this->alpha0 + acos(cosAlpha);
  else
    this->alpha = this->alpha0 - acos(cosAlpha);

  // normalize to within +/-90 deg
  while (fabs(this->alpha) > M_PI)
    this->alpha = this->alpha > 0 ? this->alpha - 2*M_PI
                                  : this->alpha + 2*M_PI;

  // compute dynamic pressure
  double speedInLDPlane = velInLDPlane.GetLength();
  double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;

//std::cerr<<"\n speedInLDPlane: "<<speedInLDPlane<<" q: "<<q;
//std::cerr<<"\n alpha: "<<alpha<<" alphaStall: "<<alphaStall;
  // compute cl at cp, check for stall, correct for sweep
  double cl;
  cl = 5 * sin(2*this->alpha);
  // compute lift force at cp
  math::Vector3 lift = cl * q * this->area * liftDirection;

  // compute cd at cp, check for stall, correct for sweep
  double cd;
  // make sure drag is positive
  //cd = fabs(cd);

  cd = 2 * (1-cos(2*this->alpha));
  // drag at cp
  math::Vector3 drag = cd * q * this->area * dragDirection;

  // compute cm at cp, check for stall, correct for sweep
  double cm;
  if (this->alpha > this->alphaStall)
  {
    cm = (this->cma * this->alphaStall +
          this->cmaStall * (this->alpha - this->alphaStall))
         * cosSweepAngle2;
    // make sure cm is still great than 0
    cm = std::max(0.0, cm);
  }
  else if (this->alpha < -this->alphaStall)
  {
    cm = (-this->cma * this->alphaStall +
          this->cmaStall * (this->alpha + this->alphaStall))
         * cosSweepAngle2;
    // make sure cm is still less than 0
    cm = std::min(0.0, cm);
  }
  else
    cm = this->cma * this->alpha * cosSweepAngle2;

  // reset cm to zero, as cm needs testing
  cm = 0.0;

  // compute moment (torque) at cp
  math::Vector3 moment = cm * q * this->area * momentDirection;

  // moment arm from cg to cp in inertial plane
  math::Vector3 momentArm = pose.rot.RotateVector(
    this->cp - this->link->GetInertial()->GetCoG());
  // std::cerr << this->cp << " : " << this->link->GetInertial()->GetCoG() << "\n";

  // force and torque about cg in inertial frame
  math::Vector3 force = lift + drag;
  // + moment.Cross(momentArm);

  math::Vector3 torque = moment;
//std::cerr<<"\n CL: "<<cl<<" CD: "<<cd;
//std::cerr<<"\nlift: "<<lift<<" drag: "<<drag<< "moment: "<<moment;
  // - lift.Cross(momentArm) - drag.Cross(momentArm);

  // debug
  //
  // if ((this->link->GetName() == "wing_1" ||
  //      this->link->GetName() == "wing_2") &&
  //     (vel.GetLength() > 50.0 &&
  //      vel.GetLength() < 50.0))
  //force.z = ;
  ::count++;
  if (::count >= 200 && 1){
    std::cerr << "Link: [" << this->link->GetName() << "  ";
    //std::cerr << "alpha: " << this->alpha*180/3.1415 << "\n";
    //std::cerr << "waterCurrent: " << waterCurrent << "\n";
    //std::cerr << "cl: " << cl << "\n";
    std::cerr << "lift: " << lift << " ";
    std::cerr << "cl: " << cl << " ";
    std::cerr << "drag: " << drag << " cd: "
    << cd << "\n";
    //std::cerr << "force: " << force << "\n";
    ::count = 0;
  }

  if (::count >= 200 && 0){
    std::cerr << "Link: [" << this->link->GetName() << "\n";
    std::cerr << "alpha: " << this->alpha*180/3.1415 << "\n";
    std::cerr << "waterCurrent: " << waterCurrent << "\n";
    std::cerr << "cl: " << cl << "\n";
    std::cerr << "lift: " << lift << "\n";
    std::cerr << "cd: " << cd << "\n";
    std::cerr << "drag: " << drag << " cd: "
    << cd << "\n";
    std::cerr << "force: " << force << "\n";
    ::count = 0;
  }
  if (0)
  {
    std::cerr << "=============================\n";
    std::cerr << "Link: [" << this->link->GetName()
          << "] pose: [" << pose
          << "] dynamic pressure: [" << q << "]\n";
    std::cerr << "spd: [" << vel.GetLength() << "] vel: [" << vel << "]\n";
    std::cerr << "spd sweep: [" << velInLDPlane.GetLength()
          << "] vel in LD: [" << velInLDPlane << "]\n";
    std::cerr << "forward (inertial): " << forwardI << "\n";
    std::cerr << "upward (inertial): " << upwardI << "\n";
    std::cerr << "lift dir (inertial): " << liftDirection << "\n";
    std::cerr << "LD Normal: " << ldNormal << "\n";
    std::cerr << "sweep: " << this->sweep << "\n";
    std::cerr << "alpha: " << this->alpha << "\n";
    std::cerr << "lift: " << lift << "\n";
    std::cerr << "drag: " << drag << " cd: "
    << cd << " cda: " << this->cda << "\n";
    std::cerr << "moment: " << moment << "\n";
    std::cerr << "cp momentArm: " << momentArm << "\n";
    std::cerr << "force: " << force << "\n";
    std::cerr << "torque: " << torque << "\n";
  }

  // apply forces at cg (with torques for position shift)
  this->link->AddForceAtRelativePosition(force, this->cp);
  //this->link->AddTorque(torque);
}

void Rudderplugin::ReadWaterCurrent(const geometry_msgs::Vector3::ConstPtr& _msg)
{
	waterCurrent.x = _msg->x;
	waterCurrent.y = _msg->y;
	waterCurrent.z = _msg->z;
}
