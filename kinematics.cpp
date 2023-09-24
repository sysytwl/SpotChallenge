#include <Arduino.h>

class kinematics{
  public:
    kinematics(float L1, float L2){
      _L1 = L1;
      _L2 = L2;
    };

    struct Position {
      //float Joint0, Joint1, Joint2, Joint3;
      float RightlyJoint0, RightlyJoint1, RightlyJoint2, RightlyJoint3;
      float LeftyJoint0, LeftyJoint1, LeftyJoint2, LeftyJoint3;
      bool Lefty, Rightly;
    } Position_;

    struct ForwardKinematicsOutput {
      float X, Y, Z;
    } ForwardKinematics_, transformed_;

    void InverseKinematics(float targetX, float targetY, float targetZ) {

      float x_y = sqrt(targetX * targetX + targetY * targetY);
      float xy_z = sqrt(x_y * x_y + targetZ * targetZ);

      Position_.Lefty = false;
      Position_.Rightly = false;

      if (xy_z <= _Lsum && xy_z >= _Ldiff) {
        _Alpha = acos((xy_z * xy_z + _L1 * _L1 - _L2 * _L2) / (2 * _L1 * xy_z)) * 180.0f / PI;
        _Beta = acos((_L1 * _L1 + _L2 * _L2 - xy_z * xy_z) / (2 * _L1 * _L2)) * 180.0f / PI;
        _Gamma = atan2(targetZ, x_y) * 180.0f / PI;
        Position_.LeftyJoint0 = Position_.RightlyJoint0 = atan2(targetY, targetX) * 180.0f / PI;
        Position_.RightlyJoint1 = _Gamma - _Alpha;
        Position_.RightlyJoint2 = 180.0f - _Beta;
        Position_.LeftyJoint1 = _Gamma + _Alpha;
        Position_.LeftyJoint2 = _Beta - 180.0f;

        if (-180 <= Position_.LeftyJoint0 <= 180 && 0 <= Position_.LeftyJoint1 <= 180 && 0 <= Position_.LeftyJoint2 <= 180){
          Position_.Lefty = true;
        } 
        if (-180 <= Position_.RightlyJoint0 <= 180 && 0 <= Position_.RightlyJoint1 <= 180 && 0 <= Position_.RightlyJoint2 <= 180){
          Position_.Rightly = true;
        }
      }
    };

    void ForwardKinematics(float Joint0, float Joint1, float Joint2) {
      ForwardKinematics_.Z = _L1 * sin(Joint1) + _L2 * sin(Joint1 + Joint2);
      _xy = _L1 * cos(Joint1) + _L2 * cos(Joint1 + Joint2);
      ForwardKinematics_.X = _xy * cos(Joint0);
      ForwardKinematics_.Y = _xy * sin(Joint0);
    };

    void CoordinateTrans(float originX, float originY, float originZ, float X, float Y, float Z) {
      transformed_.X = X - originX;
      transformed_.Y = Y - originY;
      transformed_.Z = Z - originZ;
    };

  private:
    float _L1, _L2;
    float _Lsum = _L1 + _L2;
    float _Ldiff = abs(_L1 - _L2);
    float _Alpha, _Beta, _Gamma;
    float _xy;
    //float _Theta;
};