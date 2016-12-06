#include <wmrde/main.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

volatile float speed_msg = 0.0;
void chatterCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->data);
  speed_msg = msg->data;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "listener");

    boost::thread simThread(&simulatorThread);
    // ros::spin();
    simThread.join();

    return 0;
}



void simulatorThread()
{
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    //set simulation options
    bool do_dyn = true; //do dynamic simulation, else kinematic
    bool ideal_actuators = false;
    bool do_anim = true; //do animation

    const int dt_ms = 1;
    const Real dtSim = dt_ms / 1000.0;
    Real time = 0;

    //setup sizes of vectors
    const int MAXNS = NUMSTATE(WmrModel::MAXNF);
    const int MAXNV = NUMQVEL(WmrModel::MAXNF);
    const int MAXNY = MAXNS+MAXNV+WmrModel::MAXNA; //for dynamic sim

    //make WmrModel object
    WmrModel mdl;
    Real state[MAXNS];
    Real qvel[MAXNV]; //for dynamic sim

    /*
     * This function sets up the model. Initializes the state of the model,
     * also uncomment the corresponding scene function below!
     */
    rocky( mdl,state,qvel);
    // zoe( mdl,state,qvel);


    //initialize wheel-ground contact model
    mdl.wheelGroundContactModel(0, mdl.wgc_p, 0, 0, 0, //inputs
                                0, 0); //outputs

    if (ideal_actuators)
        mdl.actuatorModel=0;

    //get from WmrModel
    const int nf = mdl.get_nf();
    const int nw = mdl.get_nw();
    const int nt = mdl.get_nt();
    const int ns = NUMSTATE(nf); //number of states
    //for dynamic sim
    const int nv = NUMQVEL(nf); //size of joint space vel
    const int na = mdl.get_na();
    int ny;

    //define the terrain/map
    SurfaceVector surfs;
    flat(surfs);

    manyRamps(surfs,terrainInfo);
    // ramp(surfs); //must also uncomment flat
    // grid(surfs, ResourceDir() + std::string("gridsurfdata9.txt") );

    //init contact geometry
    WheelContactGeom wcontacts[WmrModel::MAXNW];
    TrackContactGeom tcontacts[WmrModel::MAXNW];
    ContactGeom* contacts =0; //base class

    if (nw > 0)
    {
        contacts = static_cast<ContactGeom*>(wcontacts);
    }
    else if (nt > 0)
    {
        sub_initTrackContactGeom(mdl, tcontacts);
        contacts = static_cast<ContactGeom*>(tcontacts);
    }

    initTerrainContact(mdl, surfs, contacts, state); //DEBUGGING

    //allocate state vector
    Real y[MAXNY];
    Real ydot[MAXNY];

    //init y
    if (do_dyn)   //dynamic sim
    {
        copyVec(ns,state,y);
        copyVec(nv,qvel,y+ns);
        setVec(na,0.0,y+ns+nv); //interr
        ny = ns + nv + na;
    }
    else
    {
        copyVec(ns,state,y);
        ny = ns;
    }

    //backup
    Real y0[MAXNY];
    copyVec(ny,y,y0);

    //allocate the transform objects
    HomogeneousTransform HT_parent[WmrModel::MAXNF];
    HomogeneousTransform HT_world[WmrModel::MAXNF];

    // Initialize the render system
    anim.start();

    //uncomment the scene function that corresponds to the model function above
    rockyScene(mdl, anim);
    // zoeScene(mdl, anim);

    // render the surfaces
    for (int i=0; i<surfs.size(); i++)
        anim.addEntitySurface(surfs[i].get());

    init();


    // Start simulation
    while(true)
    {
        ros::spinOnce();

        odeDyn(time, y, mdl, surfs, contacts, dtSim, ydot, HT_parent, HT_world);

        // Obtain the position and orientation of body frame
        Vec3 pos;
        Vec3 ori;
        HTToPose(HT_world[0],ori,pos);
        updateSimInterface(pos[0],pos[1],pos[2],ori[2],dtSim,time);

        addmVec(ny,ydot,dtSim,y);
        time += dtSim;

        std::cout <<"\n Here\n"<<std::endl;
        // simInterface.setSpeedCmd(0.1);

        // Render the frame
        anim.updateNodesLines(nf, HT_parent, nw + nt, contacts);
        if (!anim.updateRender())
            goto stop;

        // If pause button is pressed, pause the simulation temporarily
        while( true == anim.get_mPause() )
        {
            if (!anim.updateRender())
            {
                goto stop;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
    }

stop:
    cleanup();
    std::cout << "\nExiting simulator\n";
    exit(0);
}
void updateSimInterface(const double x,const double y,const double z,const double heading, const double dt, const double currentTime)
{
    simInterface.setPositionX(x);
    simInterface.setPositionY(y);
    simInterface.setPositionZ(z);
    simInterface.setHeading(heading);

    // Compute robot speed
    const Eigen::Vector3d currentPos(x,y,0); // we only care about 2d projection
    double robotSpeed = (currentPos - oldPos).norm() / dt;
    oldPos = currentPos;
    simInterface.setSpeed(robotSpeed);

    simInterface.setTime(currentTime);

    of << x << " " << y << " " << currentTime << " " << robotSpeed << std::endl;
    of.flush();
}
void init()
{
    anim.drawWorldMap();
}
void cleanup()
{
}

void sub_initTrackContactGeom(const WmrModel& mdl, TrackContactGeom* contacts)
{

    //get from WmrModel
    const int nt = mdl.get_nt();
    const int* sprocketframeinds = mdl.get_sprocketframeinds();
    const Frame* frames = mdl.get_frames();

    for (int tno = 0; tno < nt; tno++)
    {
        int fi = sprocketframeinds[tno];
        initTrackContactGeom(frames[fi].rad, frames[fi].rad2, frames[fi].L, contacts[tno]);
    }
}

