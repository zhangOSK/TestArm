#include "Utils.h"

float distance(Vector3d posT,Vector3d posRH)
{
  return sqrt(pow(posRH(0)-posT(0),2)+pow(posRH(1)-posT(1),2));
}

double thetaFct(Quaterniond q)
{
  double theta;
  double theta_z = 2*asin(q.vec()(2));
  double theta_w = 2*acos(q.w()); 

  if (fabs(theta_z - theta_w) > 2)
  {
    double theta_w2 = - theta_w;
    if (fabs(theta_z - theta_w2) > 2)
    {
      theta = theta_w;
    } else {
      theta = theta_z;
    }
  } else {
    theta = theta_z;
  }
  return theta;
}

double nullFunction(double val)
{
  if(fabs(val) < 1e-4)
  {
    val = 0.0;
  }
  return val;
}