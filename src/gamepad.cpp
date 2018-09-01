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
// using rw::graphics::WorkCellScene;
using rw::loaders::WorkCellFactory;

Controller::Controller() : RobWorkStudioPlugin("Controller", QIcon(":/gamepad.png"))
{
    _ui.setupUi(this);

    connect(_ui.pushButton_2, SIGNAL(clicked()), this, SLOT(ObslugaPrzyciskuConnect()));
    // WorkCellScene::setWorkCell(*wc);
    _wc = WorkCellFactory::load("/home/tomek/Documents/RobWork/workcell/Scene.wc.xml");
    // getRobWorkStudio()->postWorkCell(_wc);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Controller::joyCallback, this);
    initRumble();
    mainLoopThread = std::thread(mainLoop);
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
    getRobWorkStudio()->postWorkCell(_wc);
    //   robotUR5 = new RobotInterface();
    //  Log::infoLog() << "Inicjalizacja";
}

void Controller::open(WorkCell *workcell)
{

    try
    {
        // _wc = workcell;
        _state = getRobWorkStudio()->getState();
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
    Log::infoLog() << "Connecting..." << endl;
    //   if (robotUR5->connect()) {
    //     Log::infoLog() << "Connected" << endl;
    //   }
}
void Controller::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    Log::infoLog() << "Callback..." << endl;
    for (int i = 0; i < 5; ++i)
    {
        twist[i] = a_scale_ * joy->axes[i];
        Log::infoLog() << twist[i] << endl;
    }
}

void mainLoop()
{
    while (true)
    {
        Log::infoLog() << "loop" << endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
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
