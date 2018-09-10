#include "gamepad.hpp"
#include <rws/RobWorkStudio.hpp>
#include <QMessageBox>

using namespace std;
using namespace rws;
using namespace rw::common;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace boost;
using rw::loaders::WorkCellFactory;

Controller::Controller() : RobWorkStudioPlugin("Controller", QIcon(":/gamepad.png"))
{
    _ui.setupUi(this);

    connect(_ui.pushButton_2, SIGNAL(clicked()), this, SLOT(ObslugaPrzyciskuConnect()));
    connect(_ui.pushButton_Start, SIGNAL(clicked()), this, SLOT(ObslugaPrzyciskuStart()));
    connect(_ui.pushButton_Stop, SIGNAL(clicked()), this, SLOT(ObslugaPrzyciskuStop()));
    // WorkCellScene::setWorkCell(*wc);
    _wc = WorkCellFactory::load("/home/tomek/Documents/RobWork/workcell/Scene.wc.xml");
    // getRobWorkStudio()->postWorkCell(_wc);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Controller::joyCallback, this);
    initRumble();
    robotUR5 = new RobotInterface();
    // std::thread(&Controller::mainLoop, this).detach();
    // ros::spin();
    // Log::infoLog() << "Konstruktor";
}

Controller::~Controller()
{
}

void Controller::initialize()
{

    //getRobWorkStudio()->genericEvent().add(boost::bind(&RWSPlugin::genericEventListener, this, _1), this);
    //getRobWorkStudio()->keyEvent().add(boost::bind(&RWSPlugin::keyEventListener, this, _1, _2), this);

    log().setLevel(Log::Info);
    while (!_wc)
        ;
    getRobWorkStudio()->setWorkcell(_wc);
    std::thread(&Controller::mainLoop, this).detach();
    //   robotUR5 = new RobotInterface();
    //  Log::infoLog() << "Inicjalizacja";
}

void Controller::open(WorkCell *workcell)
{

    try
    {
        // _wc = workcell;
        _state = getRobWorkStudio()->getState();
        _robot = _wc->findDevice("robot");
        if (!_robot)
        {
            INFO << "robot device not found" << endl;
        }

        // find ghost robot device
        _ghost = _wc->findDevice("ghost");
        if (!_ghost)
        {
            INFO << "ghost not found" << endl;
        }
        _sdghost = new rw::models::SerialDevice(_ghost->getBase(), _ghost->getEnd(), _ghost->getName(), _state);
        _invkin = new rw::invkin::ClosedFormIKSolverUR(_sdghost, _wc->getDefaultState());
        _invkin->setCheckJointLimits(true);

        // initialize collision detector
        // _cd = new rw::proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
    }
    catch (const rw::common::Exception &e)
    {
        QMessageBox::critical(NULL, "RW Exception", e.what());
    }
    // Log::infoLog() << "Open";
}

void Controller::close()
{
    /* ... */
}

void Controller::update()
{
    /* ... */
}

void Controller::genericEventListener(const std::string &event)
{

    /*if (event == "DynamicWorkCellLoaded") {
    
  }*/
}

void Controller::keyEventListener(int key, Qt::KeyboardModifiers modifier)
{

    /* ... */
}

// void Controller::ObslugaPrzycisku(){
// 	Log::infoLog() << "Hello world!" << endl;

//   rw::models::Device::Ptr _robot = _wc->findDevice("robot");
//   if(_robot != NULL){
//   // rw::math::Q q = _robot->getQ(_state);
//   // Log::infoLog() << q << endl;
//   // rw::math::Q q1 = rw::math::Q(6, 2.3, -1.1, 1.0, 1.3, 1.1, 1.0);

//   // _robot->setQ(q1, _state);
//   // getRobWorkStudio()->setState(_state);
//   rwhw::URRTData data = robotUR5->getData();
//   rw::math::Q q = data.qActual;
//         rw::math::Q dq = data.dqActual;

//         if (q.size() == 6) {
//         _robot->setQ(q, _state);
//         getRobWorkStudio()->setState(_state);
//         }
//   }

// }

void Controller::ObslugaPrzyciskuConnect()
{
    if (!_connected)
    {
        Log::infoLog() << "Connecting..." << endl;
        if (robotUR5->connect())
        {
            Log::infoLog() << "Connected" << endl;
            _connected = true;
        }
    }
}
void Controller::ObslugaPrzyciskuStart()
{
    if (!_recording)
    {
        Log::infoLog() << "Recording..." << endl;

        std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
        _myfile.open("/home/tomek/Documents/RobWork/measure/" + std::to_string(ms.count()) + ".csv");
        _startTime = std::chrono::system_clock::now();
        // _myfile << "controllerTimeStamp,qTarget,dqTarget,ddqTarget,iTarget,torqueTarget,qActual,dqActual,iActual,accValues,tcpForce,toolPose,tcpSpeed" << std::endl;
        _recording = true;
    }
}
void Controller::ObslugaPrzyciskuStop()
{
    if (_recording)
    {
        _recording = false;
        Log::infoLog() << "Stop recording" << endl;
        if (_myfile)
        {
            Log::infoLog() << "saving" << endl;
            // _recording = false;
            _myfile << _ss.str(); //rdbuf();
            _myfile.flush();
            _myfile.close();
            _ss.flush();
            // _recording = false;
        }
    }
}
void Controller::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    // Log::infoLog() << "Callback..." << endl;
    for (int i = 0; i < N_AXES; ++i)
    {
        axes[i] = joy->axes[i];
        if (std::abs(axes[i]) < 0.2)
        {
            axes[i] = 0;
        }
        // Log::infoLog() << axes[i] << endl;
    }
    for (int i = 0; i < N_BUTTONS; ++i)
    {
        buttons[i] = joy->buttons[i];
        // Log::infoLog() << buttons[i] << endl;
    }
    if (buttons[6] == 1)
    {
        _steeringMode = 0;
    }
    else if (buttons[7] == 1)
    {
        _steeringMode = 1;
    }
    if (buttons[8] == 1)
    {
        ObslugaPrzyciskuConnect();
        // rwhw::URRTData data = robotUR5->getData();
        // rw::math::Q q = data.qActual;
        // rw::math::Q dq = data.dqActual;

        // if (q.size() == 6)
        // {
        //     _robot->setQ(q, _state);
        //     // getRobWorkStudio()->setState(_state);
        // }

        // robotUR5->move_to_q(rw::math::Q(6, 0, -1.57, 0, -1.57, 0, 0), 100, 3000);
    }
    if (buttons[0] == 1)
    {
        if (_updatingMode == 0)
        {
            _updatingMode = 1;
        }
        else
        {
            // TODO: zastanowaić się tutaj czy nie przenieść do mainLoop i stworzyc flage
            if (_connected)
            {
                _move_robot_to_ghost = true;
                // robotUR5->move_to_q(q_ghost, 100, 3000);
            }
            _updatingMode = 0;
        }

        // robotUR5->move_to_q(q_ghost, 100, 3000);
    }
    else if (buttons[1] == 1)
    {
        if (_updatingMode == 1)
        {
            _back_ghost_to_robot = true;
            _updatingMode = 0;
        }
    }
    // Log::infoLog() << (bool)axes[1] << endl;
}

void Controller::updateGhost()
{
    double scale = 0.001;
    double scaleL = 0.001;
    double scaleR = 0.001;
    if (_ghost)
    {
        if (axes[0] || axes[1] || (axes[2] - axes[5]) || axes[3] || axes[4] || axes[6])
        {
            rw::math::Q q = _ghost->getQ(_state);
            std::vector<rw::math::Q> config;
            switch (_steeringMode)
            {
            case 0:
                // Log::infoLog() << q << endl;
                q[0] += scale * (1 + buttons[4] + 2 * buttons[5]) * axes[0];
                q[1] += scale * (1 + buttons[4] + 2 * buttons[5]) * (-axes[1]);
                q[2] += scale * (1 + buttons[4] + 2 * buttons[5]) * (axes[2] - axes[5]) / 2;
                q[3] += scale * (1 + buttons[4] + 2 * buttons[5]) * axes[3];
                q[4] += scale * (1 + buttons[4] + 2 * buttons[5]) * axes[4];
                q[5] += scale * (1 + buttons[4] + 2 * buttons[5]) * axes[6];

                config.push_back(q);
                config = expandQ(config);
                if (!checkCollision(_ghost, config[0]))
                {
                    _ghost->setQ(q, _state);
                    q_ghost = q;
                }
                else
                {
                    rumble(2);
                }
                break;
            case 1:
                // Transform3D<> t3 = rw::kinematics::Kinematics::frameTframe(_ghost->getBase(), _ghost->getEnd(), _state);
                rw::math::Transform3D<> t3(
                    Vector3D<>(scaleL * (1 + buttons[4] + 2 * buttons[5]) * axes[0],
                               scaleL * (1 + buttons[4] + 2 * buttons[5]) * (-axes[1]),
                               scaleL * (1 + buttons[4] + 2 * buttons[5]) * (axes[2] - axes[5]) / 2),
                    RPY<>(0.0,
                          0.0,
                          0.0)
                        .toRotation3D());
                rw::math::Transform3D<> t4(
                    Vector3D<>(0.0,
                               0.0,
                               0.0),
                    RPY<>(scaleR * (1 + buttons[4] + 2 * buttons[5]) * axes[6],
                          scaleR * (1 + buttons[4] + 2 * buttons[5]) * axes[4],
                          scaleR * (1 + buttons[4] + 2 * buttons[5]) * axes[3])
                        .toRotation3D());
                rw::math::Q q1 = ik(_ghost, t3, t4, _state);
                if (q1.size() == q.size())
                {
                    q = q1;
                }
                _ghost->setQ(q, _state);
                q_ghost = q;
                break;
            }
        }
        if (_back_ghost_to_robot)
        {
            rw::math::Q q = _robot->getQ(_state);
            _ghost->setQ(q, _state);
            q_ghost = q;
            _back_ghost_to_robot = false;
        }

        // rw::math::Q q = _ghost->getQ(_state);
        // Log::infoLog() << q << endl;
        // q[0] += scale * (1 + buttons[4] + 2 * buttons[5]) * axes[0];
        // q[1] += scale * (1 + buttons[4] + 2 * buttons[5]) * (-axes[1]);
        // q[2] += scale * (1 + buttons[4] + 2 * buttons[5]) * (axes[2] - axes[5]);
        // q[3] += scale * (1 + buttons[4] + 2 * buttons[5]) * axes[3];
        // q[4] += scale * (1 + buttons[4] + 2 * buttons[5]) * axes[4];
        // q[5] += scale * (1 + buttons[4] + 2 * buttons[5]) * axes[6];
        // _ghost->setQ(q, _state);

        // TODO: uncomment this \/
        if (_connected)
        {
            rwhw::URRTData data = robotUR5->getData();
            rw::math::Q q = data.qActual;
            rw::math::Q dq = data.dqActual;

            if (q.size() == 6)
            {
                _robot->setQ(q, _state);
            }
        }
        getRobWorkStudio()->setState(_state);
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); //after setState it must be a delay to refresh screen
    }
}

rw::math::Q Controller::ik(rw::models::Device::Ptr robot, const rw::math::Transform3D<> &t_pos, const rw::math::Transform3D<> &t_rot, const rw::kinematics::State &state)
{
    rw::kinematics::Frame *end = robot->getEnd();
    rw::kinematics::Frame *base = robot->getBase();
    rw::math::Transform3D<> toolTend = rw::kinematics::Kinematics::frameTframe(base, end, state);
    rw::math::Transform3D<> endT = t_pos * toolTend * t_rot;

    vector<rw::math::Q> possibleConfigurations, validConfigurations;
    possibleConfigurations = _invkin->solve(endT, state);
    //INFO << "Found " << possibleConfigurations.size() << " possible configurations" << endl;

    for (vector<rw::math::Q>::iterator i = possibleConfigurations.begin(); i != possibleConfigurations.end(); ++i)
    {
        if (!checkCollision(robot, *i))
        {
            validConfigurations.push_back(*i);
        }
        // validConfigurations.push_back(*i);
    }
    validConfigurations = expandQ(validConfigurations);
    //INFO << "Found " << validConfigurations.size() << " valid configurations" << endl;

    rw::math::Q current_q = robot->getQ(state);
    vector<rw::math::Q>::iterator closest_q = validConfigurations.begin();
    double dist = 1000.0;
    for (vector<rw::math::Q>::iterator i = validConfigurations.begin(); i != validConfigurations.end(); ++i)
    {

        Q q1 = *i;
        Q q2 = current_q;

        q1[1] = q1[1] * 2;
        q2[1] = q2[1] * 2;

        double d = rw::math::MetricUtil::dist2<rw::math::Q>(q1, q2);
        if (d < dist)
        {
            dist = d;
            closest_q = i;
        }
    }

    if (validConfigurations.size() == 0)
    {
        INFO << "No valid Q found!" << endl;
        rumble(2);
        return rw::math::Q();
    }
    else
    {
        return *closest_q;
    }
}

bool Controller::checkCollision(rw::models::Device::Ptr robot, const rw::math::Q &q)
{
    bool ret = false;

    rw::kinematics::State testState = _state.clone();
    robot->setQ(q, testState);

    if (_cd->inCollision(testState))
    {
        ret = true;
    }

    return ret;
}

std::vector<rw::math::Q> Controller::expandQ(const std::vector<rw::math::Q> &config)
{
    std::vector<rw::math::Q> res1 = config;
    std::vector<rw::math::Q> res2;
    const double pi2 = 2 * rw::math::Pi;
    rw::math::Q lower, upper;
    std::vector<size_t> indices;
    const std::vector<rw::models::Joint *> &joints = _sdghost->getJoints();
    size_t index = 0;
    BOOST_FOREACH (const rw::models::Joint *joint, joints)
    {
        if (dynamic_cast<rw::models::RevoluteJoint *>(joints[index]))
        {
            for (int i = 0; i < joint->getDOF(); i++)
            {
                indices.push_back(index);
                ++index;
            }
        }
        else
        {
            index += joint->getDOF();
        }
    }

    lower = _sdghost->getBounds().first;
    upper = _sdghost->getBounds().second;

    for (auto index : indices)
    {
        res2.clear();
        //        std::cout<<"Index = "<<index<<std::endl;
        for (auto q : res1)
        {
            //std::cout<<"Input= "<<q<<std::endl;

            double d = q(index);
            while (d > lower(index))
                d -= pi2;
            while (d < lower(index))
                d += pi2;
            while (d < upper(index))
            {
                rw::math::Q tmp(q);
                tmp(index) = d;
                res2.push_back(tmp);
                //  std::cout<<"Output = "<<tmp<<std::endl;
                d += pi2;
            }
        }
        res1 = res2;
        //std::cout<<"Output Count = "<<res1.size()<<std::endl;
    }
    return res1;
}

void Controller::mainLoop()
{
    // initialize collision detector
    _cd = new rw::proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

    bool ready = false;
    while (true)
    {
        // Log::infoLog() << "loop" << endl;
        updateGhost();
        if (_connected && !ready)
        {
            // std::this_thread::sleep_for(std::chrono::milliseconds(5000)); //after setState it must be a delay to refresh screen
            ready = true;
        }
        if (_updatingMode == 0 && ready)
        {
            rwhw::URRTData data = robotUR5->getData();
            // rw::math::Q q = rw::math::Q(6, 0, -1.57, 0, -1.57, 0, 0);
            rw::math::Q q = data.qActual;
            if (q.size() == 6)
            {
                rw::math::Q differneceQ = q - q_ghost;
                if (differneceQ.norm2() > 1 || _updatingMode == 1)
                {
                    robotUR5->move_to_q(q_ghost, 50, 3000);
                    // rw::math::Q differneceQ;
                    // do
                    // {
                    //     data = robotUR5->getData();
                    //     q = data.qActual;
                    //     if (q.size() == 6)
                    //     {
                    //         _robot->setQ(q, _state);
                    //     }
                    //     getRobWorkStudio()->setState(_state);
                    //     differneceQ = q - q_ghost;
                    //     std::this_thread::sleep_for(std::chrono::milliseconds(1)); //after setState it must be a delay to refresh screen
                    // } while (differneceQ.norm2() > 0.01);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    ros::spinOnce();
                }
                else
                {
                    robotUR5->servo_q(q_ghost);
                    data = robotUR5->getData();
                    if (_recording)
                    {
                        _ss << data.controllerTimeStamp << "," << data.qTarget << "," << data.dqTarget << "," << data.ddqTarget << "," << data.iTarget << "," << data.torqueTarget << "," << data.qActual << "," << data.dqActual << "," << data.iActual << "," << data.accValues << "," << data.tcpForce << "," << data.toolPose << "," << data.tcpSpeed << std::endl;
                    }
                }
            }
            if (_move_robot_to_ghost)
            {
                _move_robot_to_ghost = false;
                robotUR5->move_to_q(q_ghost, 50, 3000);
                // rw::math::Q q;
                // rwhw::URRTData data;
                rw::math::Q differneceQ;
                do
                {
                    data = robotUR5->getData();
                    q = data.qActual;
                    if (q.size() == 6)
                    {
                        _robot->setQ(q, _state);
                    }
                    getRobWorkStudio()->setState(_state);
                    differneceQ = q - q_ghost;
                    if (_recording)
                    {
                        // std::string c(",");
                        // _ss << "c";
                        // Log::infoLog() << data.controllerTimeStamp << "," << data.qTarget << "," << data.dqTarget << "," << data.ddqTarget << "," << data.iTarget << "," << data.torqueTarget << "," << data.qActual << "," << data.dqActual << "," << data.iActual << "," << data.accValues << "," << data.tcpForce << "," << data.toolPose << "," << data.tcpSpeed << std::endl;

                        _ss << data.controllerTimeStamp << "," << data.qTarget << "," << data.dqTarget << "," << data.ddqTarget << "," << data.iTarget << "," << data.torqueTarget << "," << data.qActual << "," << data.dqActual << "," << data.iActual << "," << data.accValues << "," << data.tcpForce << "," << data.toolPose << "," << data.tcpSpeed << std::endl;
                        // Log::infoLog() << _ss.rdbuf() << endl;
                    }
                    // _myfile << data.controllerTimeStamp + "," + data.qTarget + "," + data.dqTarget + "," + data.ddqTarget + "," + data.iTarget + "," + data.torqueTarget + "," + data.qActual + "," + data.dqActual + "," + data.iActual + "," + data.accValues + "," + data.tcpForce + "," + data.toolPose + "," + data.tcpSpeed << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(1)); //after setState it must be a delay to refresh screen
                    ros::spinOnce();
                } while (differneceQ.norm2() > 0.01);

                // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            }
            // robotUR5->move_to_q(q_ghost, 100, 3000);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ros::spinOnce();
    }
}

struct ff_effect effects[N_EFFECTS];
struct input_event play, stop, gain;
int fd;

void rumble(int i)
{
    memset(&play, 0, sizeof(play));
    play.type = EV_FF;
    play.code = effects[i].id;
    play.value = 1;

    if (write(fd, (const void *)&play, sizeof(play)) == -1)
    {
        perror("Play effect");
        exit(1);
    }
    // memset(&stop,0,sizeof(stop));
    // stop.type = EV_FF;
    // stop.code =  effects[i].id;
    // stop.value = 0;

    // if (write(fd, (const void*) &stop, sizeof(stop)) == -1) {
    //     perror("");
    //     exit(1);
    // }
}

void initRumble()
{
    const char *device_file_name = "/dev/input/by-id/usb-Microsoft_Controller_3032363030313236363036373439-event-joystick";
    fd = open(device_file_name, O_RDWR);
    if (fd == -1)
    {
        perror("Open device file");
        exit(1);
    }
    /* download a periodic sinusoidal effect */
    memset(&effects[0], 0, sizeof(effects[0]));
    effects[0].type = FF_PERIODIC;
    effects[0].id = -1;
    effects[0].u.periodic.waveform = FF_SINE;
    effects[0].u.periodic.period = 100;       /* 0.1 second */
    effects[0].u.periodic.magnitude = 0x7fff; /* 0.5 * Maximum magnitude */
    effects[0].u.periodic.offset = 0;
    effects[0].u.periodic.phase = 0;
    effects[0].direction = 0x4000; /* Along X axis */
    effects[0].u.periodic.envelope.attack_length = 1000;
    effects[0].u.periodic.envelope.attack_level = 0x7fff;
    effects[0].u.periodic.envelope.fade_length = 1000;
    effects[0].u.periodic.envelope.fade_level = 0x7fff;
    effects[0].trigger.button = 0;
    effects[0].trigger.interval = 0;
    effects[0].replay.length = 20000; /* 20 seconds */
    effects[0].replay.delay = 1000;

    printf("Uploading effect #0 (Periodic sinusoidal) ... ");
    fflush(stdout);
    if (ioctl(fd, EVIOCSFF, &effects[0]) == -1)
    {
        perror("Error:");
    }
    else
    {
        printf("OK (id %d)\n", effects[0].id);
    }

    /* a strong rumbling effect */
    effects[1].type = FF_RUMBLE;
    effects[1].id = -1;
    effects[1].u.rumble.strong_magnitude = 0x8000;
    effects[1].u.rumble.weak_magnitude = 0;
    effects[1].replay.length = 500;
    effects[1].replay.delay = 0;

    printf("Uploading effect #1 (Strong rumble, with heavy motor) ... ");
    fflush(stdout);
    if (ioctl(fd, EVIOCSFF, &effects[1]) == -1)
    {
        perror("Error");
    }
    else
    {
        printf("OK (id %d)\n", effects[1].id);
    }

    /* a weak rumbling effect */
    effects[2].type = FF_RUMBLE;
    effects[2].id = -1;
    effects[2].u.rumble.strong_magnitude = 0;
    effects[2].u.rumble.weak_magnitude = 0xc000;
    effects[2].replay.length = 500;
    effects[2].replay.delay = 0;

    printf("Uploading effect #2 (Weak rumble, with light motor) ... ");
    fflush(stdout);
    if (ioctl(fd, EVIOCSFF, &effects[2]) == -1)
    {
        perror("Error");
    }
    else
    {
        printf("OK (id %d)\n", effects[2].id);
    }
}
