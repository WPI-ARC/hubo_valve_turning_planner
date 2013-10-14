#include "drchubo_analytical_ik_main.hpp"
#include "drchubo_analytical_ik_solver.hpp"

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

using std::cout;
using std::endl;

DrcHuboAnalyticalIKModule::DrcHuboAnalyticalIKModule(EnvironmentBasePtr penv) : ModuleBase(penv)
{
    cout << "Initialize DRCHubo IK problem" << endl;
    __description = "A very simple plugin.";
    RegisterCommand("NumBodies",boost::bind(&DrcHuboAnalyticalIKModule::NumBodies,this,_1,_2),"returns bodies");
    RegisterCommand("ComputeArmIK",boost::bind(&DrcHuboAnalyticalIKModule::ComputeArmIK,this,_1,_2),"returns IK solutions");
}

void DrcHuboAnalyticalIKModule::Destroy()
{
    RAVELOG_INFO("module unloaded from environment\n");
}

DrcHuboAnalyticalIKModule::~DrcHuboAnalyticalIKModule()
{

}

/**
void DrcHuboAnalyticalIKModule::SetActiveRobots(const std::vector<RobotBasePtr >& robots)
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

bool DrcHuboAnalyticalIKModule::SendCommand( std::ostream& sout, std::istream& sinput )
{
    ProblemInstance::SendCommand(sout,sinput);
    return true;
}

int DrcHuboAnalyticalIKModule::main(const std::string& cmd)
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

bool DrcHuboAnalyticalIKModule::NumBodies( std::ostream& sout, std::istream& sinput )
{
    std::vector<KinBodyPtr> vbodies;
    GetEnv()->GetBodies(vbodies);
    sout << vbodies.size();     // publish the results
    return true;
}

bool DrcHuboAnalyticalIKModule::ComputeArmIK( std::ostream& sout, std::istream& sinput )
{
    std::string robotname;
    std::string cmd;
    Eigen::Isometry3d B;
    int side;

    while(!sinput.eof())
    {
        sinput >> cmd;
        if( !sinput )
            break;

        if( cmd == "robotname" )
        {
            sinput >> robotname;
        }
        else if( cmd == "side" )
        {
            int temp;
            sinput >> temp;

            if( temp != DrcHuboAnalyticalIK::SIDE_RIGHT && temp != DrcHuboAnalyticalIK::SIDE_LEFT )
                side = -1;
            else
                side = temp;
        }
        else if( cmd == "wristpose" )
        {
            Eigen::Vector3d trans;
            Eigen::Quaterniond rot;
            sinput >> rot.x();
            sinput >> rot.y();
            sinput >> rot.z();
            sinput >> rot.w();
            sinput >> trans[0];
            sinput >> trans[1];
            sinput >> trans[2];
            B.rotation(); // = rot.toRotationMatrix();
            B.translation() = trans;
        }
    }

    cout << "robotname : " << robotname << endl;
    cout << "side : " << side << endl;
    cout << "pose : " << endl << B.matrix() << endl;

    RobotBasePtr robot = GetEnv()->GetRobot( robotname );

    DrcHuboAnalyticalIK::HuboKin kin;
    DrcHuboAnalyticalIK::Vector6d solutions[8];
    DrcHuboAnalyticalIK::Vector6d qPrev;

    bool svalid[8];
    int flags = 0;
    flags |= DrcHuboAnalyticalIK::IK_PREFER_CLOSEST_ANGLES;
    // flags |= DrcHuboAnalyticalIK::IK_IGNORE_LIMITS;

    int best_index = kin.armIK( solutions, svalid, B, qPrev, side, flags );

    sout << solutions[best_index][0] << " ";
    sout << solutions[best_index][1] << " ";
    sout << solutions[best_index][2] << " ";
    sout << solutions[best_index][3] << " ";
    sout << solutions[best_index][4] << " ";
    sout << solutions[best_index][5] << " ";

    return true;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "drchuboik" ) {
        return InterfaceBasePtr(new DrcHuboAnalyticalIKModule(penv));
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Module].push_back("DrcHuboIK");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
    RAVELOG_INFO("destroying plugin\n");
}
