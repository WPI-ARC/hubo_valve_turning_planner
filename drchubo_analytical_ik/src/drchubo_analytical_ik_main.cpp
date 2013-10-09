#include "drchubo_analytical_ik_main.hpp"

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

using std::cout;
using std::endl;

DrcHuboAnalyticalIKProblem::DrcHuboAnalyticalIKProblem(EnvironmentBasePtr penv) : ProblemInstance(penv)
{
    __description = "A very simple plugin.";
    cout << __description << endl;
    RegisterCommand("numbodies",boost::bind(&DrcHuboAnalyticalIKProblem::NumBodies,this,_1,_2),"returns bodies");
}

void DrcHuboAnalyticalIKProblem::Destroy()
{
    RAVELOG_INFO("module unloaded from environment\n");
}

DrcHuboAnalyticalIKProblem::~DrcHuboAnalyticalIKProblem()
{

}

/**
void DrcHuboAnalyticalIKProblem::SetActiveRobots(const std::vector<RobotBasePtr >& robots)
{
    if( robots.size() == 0 ) {
        RAVELOG_WARNA("No robots to plan for\n");
        return;
    }

    vector<RobotBasePtr >::const_iterator itrobot;
    FORIT(itrobot, robots) {
        if( strcmp((*itrobot)->GetName().c_str(), _strRobotName.c_str() ) == 0  ) {
            robot = *itrobot;
            break;
        }
    }

    if( robot == NULL ) {
        RAVELOG_ERRORA("Failed to find %S\n", _strRobotName.c_str());
        return;
    }
}
**/

bool DrcHuboAnalyticalIKProblem::SendCommand(std::ostream& sout, std::istream& sinput)
{
    ProblemInstance::SendCommand(sout,sinput);
    return true;
}

int DrcHuboAnalyticalIKProblem::main(const std::string& cmd)
{
    RAVELOG_DEBUG("env: %s\n", cmd.c_str());

    const char* delim = " \r\n\t";
    std::string mycmd = cmd;
    char* p = strtok(&mycmd[0], delim);
    if( p != NULL )
        _strRobotName = p;

    //std::vector<RobotBasePtr> robots;
    //GetEnv()->GetRobots(robots);
    //SetActiveRobots(robots);
    return 0;
}

bool DrcHuboAnalyticalIKProblem::NumBodies(std::ostream& sout, std::istream& sinput)
{
    std::vector<KinBodyPtr> vbodies;
    GetEnv()->GetBodies(vbodies);
    sout << vbodies.size();     // publish the results
    return true;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_ProblemInstance && interfacename == "kinect" ) {
        return InterfaceBasePtr(new DrcHuboAnalyticalIKProblem(penv));
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_ProblemInstance].push_back("Kinect");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
    RAVELOG_INFO("destroying plugin\n");
}

