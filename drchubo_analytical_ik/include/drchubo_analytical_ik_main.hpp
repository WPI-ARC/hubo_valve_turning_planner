#ifndef DRC_ANALYTICAL_IK_MAIN_HPP
#define DRC_ANALYTICAL_IK_MAIN_HPP

#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <iostream>

using namespace OpenRAVE;

class SkeletonListener;

class DrcHuboAnalyticalIKProblem : public ModuleBase
{
public:
    DrcHuboAnalyticalIKProblem(EnvironmentBasePtr penv);
    virtual ~DrcHuboAnalyticalIKProblem();
    void Destroy();

    virtual int main( const std::string& args );
    virtual bool SendCommand( std::ostream& sout, std::istream& sinput);

    bool NumBodies( std::ostream& sout, std::istream& sinput);

private:
    std::string _strRobotName; ///< name of the active robot
    RobotBasePtr robot;
};

#endif // DRC_ANALYTICAL_IK_MAIN_HPP
