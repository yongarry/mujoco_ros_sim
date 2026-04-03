/*  Copyright © 2018, Roboti LLC

    This file is licensed under the MuJoCo Resource License (the "License").
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.roboti.us/resourcelicense.txt
*/

#include "mjros.h"
#include <fstream>
#include <sstream>
#include <map>
#include <cmath>
#include <ros/package.h>

// drop file callback
void drop(GLFWwindow *window, int count, const char **paths)
{
    // make sure list is non-empty
    if (count > 0)
    {
        mju_strncpy(filename, paths[0], 1000);
        settings.loadrequest = 1;
        ROS_INFO("DROP REQUEST");
    }
}

// load mjb or xml model
void loadmodel(void)
{
    // clear request
    settings.loadrequest = 0;

    // make sure filename is not empty
    if (!filename[0])
        return;

    // load and compile
    char error[500] = "";
    mjModel *mnew = 0;
    if (strlen(filename) > 4 && !strcmp(filename + strlen(filename) - 4, ".mjb"))
    {
        mnew = mj_loadModel(filename, NULL);
        if (!mnew)
            strcpy(error, "could not load binary model");
    }
    else
    {
        mnew = mj_loadXML(filename, NULL, error, 500);
    }
    if (!mnew)
    {
        printf("%s\n", error);
        return;
    }

    // compiler warning: print and pause
    if (error[0])
    {
        // mj_forward() below will print the warning message
        printf("Model compiled, but simulation warning (paused):\n  %s\n\n",
               error);
        settings.run = 0;
    }

    // delete old model, assign new
    mj_deleteData(d);
    mj_deleteModel(m);
    m = mnew;
    d = mj_makeData(m);

    int i = settings.key;
    d->time = m->key_time[i];
    mju_copy(d->qpos, m->key_qpos + i * m->nq, m->nq);
    mju_copy(d->qvel, m->key_qvel + i * m->nv, m->nv);
    mju_copy(d->act, m->key_act + i * m->na, m->na);

    if (m->actuator_biastype[0])
    {
        mju_copy(d->ctrl, m->key_qpos + 7 + i * m->nq, m->nu);
    }

    // ── CSV에서 cube 위치/yaw 읽어서 geom_pos/geom_mat 설정 ──
    {
        std::string csv_path = ros::package::getPath("tocabi_cc") + "/cmd/global_command.csv";
        std::ifstream csv_file(csv_path);
        if (!csv_file.is_open())
            ROS_WARN("loadmodel: cannot open CSV: %s", csv_path.c_str());
        else
        {
            std::map<std::string, std::vector<double>> csv;
            std::string line;
            while (std::getline(csv_file, line))
            {
                std::istringstream ss(line);
                std::string token, key;
                std::getline(ss, key, ',');
                while (std::getline(ss, token, ','))
                    if (!token.empty())
                        csv[key].push_back(std::stod(token));
            }

            int n_cubes = (int)csv["posx"].size();
            for (int ci = 0; ci < n_cubes; ci++)
            {
                std::string bname = "cube_" + (ci < 9 ? std::string("0") : std::string("")) + std::to_string(ci + 1);
                int bid = mj_name2id(m, mjOBJ_BODY, bname.c_str());
                if (bid < 0)
                    continue;  // CSV 열이 cube 수보다 많을 경우 (예: 마지막 더미 스텝) 조용히 스킵

                // mocap body 인덱스 = body의 mocap 순서 (m->body_mocapid)
                int mid = m->body_mocapid[bid];
                if (mid < 0)
                {
                    ROS_WARN("loadmodel: body '%s' is not a mocap body", bname.c_str());
                    continue;
                }

                // mocap_pos/mocap_quat은 mjData에 저장 (월드 좌표 직접 지정)
                d->mocap_pos[3*mid + 0] = csv["posx"][ci];
                d->mocap_pos[3*mid + 1] = csv["posy"][ci];
                d->mocap_pos[3*mid + 2] = csv["posz"][ci]-0.05;

                // yaw(roty) → quaternion Rz(yaw): (w, x, y, z) = (cos(yaw/2), 0, 0, sin(yaw/2))
                double yaw = csv["roty"][ci];
                d->mocap_quat[4*mid + 0] = std::cos(yaw * 0.5);  // w
                d->mocap_quat[4*mid + 1] = 0.0;                   // x
                d->mocap_quat[4*mid + 2] = 0.0;                   // y
                d->mocap_quat[4*mid + 3] = std::sin(yaw * 0.5);  // z

                ROS_INFO("loadmodel: %s (mocap %d) -> pos(%.3f, %.3f, %.3f) yaw=%.3f",
                         bname.c_str(), mid, csv["posx"][ci], csv["posy"][ci], csv["posz"][ci], yaw);
            }
        }
    }

    mj_forward(m, d);

    ros_sim_started = true;
    delete[] ctrl_command;
    delete[] ctrl_command2;
    ctrl_command = new mjtNum[m->nu]();
    ctrl_command2 = new mjtNum[m->nbody * 6]();

    // re-create scene and context
    mjv_makeScene(m, &scn, maxgeom);
    mjr_makeContext(m, &con, 50 * (settings.font + 1));

    // clear perturbation state
    pert.active = 0;
    pert.select = 0;
    pert.skinselect = -1;

    // align and scale view, update scene
    alignscale();
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);

    // set window title to model name
    if (window && m->names)
    {
        char title[200] = "Simulate : ";
        strcat(title, m->names);
        strcat(title, ros::this_node::getNamespace().c_str());
        glfwSetWindowTitle(window, title);
    }

    // rebuild UI sections
    makesections();

    // full ui update
    uiModify(window, &ui0, &uistate, &con);
    uiModify(window, &ui1, &uistate, &con);

    updatesettings();
    mujoco_ros_connector_init();
    std::cout << " MODEL LOADED " << std::endl;
}
// run event loop
int main(int argc, char **argv)
{
    // :: ROS CUSTUM :: initialize ros
    ros::init(argc, argv, "mujoco_ros");
    ros::NodeHandle nh("~");
    std::string key_file;
    nh.param<std::string>("license", key_file, "mjkey.txt");

    nh.param("use_shm", use_shm, false);
    sim_command_sub = nh.subscribe<std_msgs::String>("/mujoco_ros_interface/sim_command_con2sim", 100, sim_command_callback);
    sim_command_pub = nh.advertise<std_msgs::String>("/mujoco_ros_interface/sim_command_sim2con", 1);
    force_apply_sub = nh.subscribe("/mujoco_ros_interface/applied_ext_force", 10, &force_apply_callback);

    if (!use_shm)
    {        
        nh.param("pub_mode", pub_total_mode, false);
        std::cout<<"Name Space: " << ros::this_node::getNamespace() << std::endl;

        //register publisher & subscriber
        char prefix[200] = "/mujoco_ros_interface";
        char joint_set_name[200];
        char sim_status_name[200];
        char joint_state_name[200];
        char sim_time_name[200];
        char sensor_state_name[200];

        strcpy(joint_set_name, prefix);
        strcpy(sim_status_name, prefix);
        strcpy(joint_state_name, prefix);
        strcpy(sim_time_name, prefix);
        strcpy(sensor_state_name, prefix);
        if (ros::this_node::getNamespace() != "/")
        {
            strcat(joint_set_name, ros::this_node::getNamespace().c_str());
            strcat(sim_status_name, ros::this_node::getNamespace().c_str());
            strcat(joint_state_name, ros::this_node::getNamespace().c_str());
            strcat(sim_time_name, ros::this_node::getNamespace().c_str());
            strcat(sensor_state_name, ros::this_node::getNamespace().c_str());
        }
        strcat(joint_set_name, "/joint_set");
        strcat(sim_status_name, "/sim_status");
        strcat(joint_state_name, "/joint_states");
        strcat(sim_time_name, "/sim_time");
        strcat(sensor_state_name, "/sensor_states");

        joint_set = nh.subscribe<mujoco_ros_msgs::JointSet>(joint_set_name, 1, jointset_callback, ros::TransportHints().tcpNoDelay(true));

        if (pub_total_mode)
            sim_status_pub = nh.advertise<mujoco_ros_msgs::SimStatus>(sim_status_name, 1);
        else
        {
            joint_state_pub = nh.advertise<sensor_msgs::JointState>(joint_state_name, 1);
            sim_time_pub = nh.advertise<std_msgs::Float32>(sim_time_name, 1);
            sensor_state_pub = nh.advertise<mujoco_ros_msgs::SensorState>(sensor_state_name, 1);
        }
    }
    else
    {
#ifdef COMPILE_SHAREDMEMORY
        init_shm(shm_msg_key, shm_msg_id, &mj_shm_);
#endif
    }

    //ROS_INFO("ROS initialize complete");
    sim_time_ros = ros::Duration(0);
    sim_time_run = ros::Time::now();
    sim_time_now_ros = ros::Duration(0);

    // // print version, check compatibility
    // std::printf("MuJoCo version %s\n", mj_versionString());
    // if (mjVERSION_HEADER!=mj_version()) {
    //     mju_error("Headers and library have different versions");
    // }
    
    // // scan for libraries in the plugin directory to load additional plugins
    // scanPluginLibraries();
    
    // mjvCamera cam;
    // mjv_defaultCamera(&cam);
    
    // mjvOption opt;
    // mjv_defaultOption(&opt);
    
    // mjvPerturb pert;
    // mjv_defaultPerturb(&pert);
    
    // // simulate object encapsulates the UI
    // auto sim = std::make_unique<mj::Simulate>(
    //     std::make_unique<mj::GlfwAdapter>(),
    //     &cam, &opt, &pert, /* is_passive = */ false
    // );
    
    // std::string model_file;
    // if (nh.getParam("model_file", model_file))
    // {
    //     mju_strncpy(filename, model_file.c_str(), 1000);
    //     settings.loadrequest = 2;
    //     ROS_INFO("model is at %s", model_file.c_str());
    // }
    
    // // start physics thread
    // std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);
    
    // // start simulation UI loop (blocking call)
    // sim->RenderLoop();
    // physicsthreadhandle.join();
    
    // initialize everything
    init();

    std::string model_file;
    // request loadmodel if file given (otherwise drag-and-drop)
    if (nh.getParam("model_file", model_file))
    {
        mju_strncpy(filename, model_file.c_str(), 1000);
        settings.loadrequest = 2;
        ROS_INFO("model is at %s", model_file.c_str());
    }

    // start simulation thread
    std::thread simthread(simulate);

    // event loop
    while ((!glfwWindowShouldClose(window) && !settings.exitrequest) && ros::ok())
    {
        // start exclusive access (block simulation thread)
        mtx.lock();
        // load model (not on first pass, to show "loading" label)
        if (settings.loadrequest == 1)
        {
            ROS_INFO("Load Request");
            loadmodel();
        }
        else if (settings.loadrequest > 1)
            settings.loadrequest = 1;

        // handle events (calls all callbacks)
        glfwPollEvents();

        // prepare to render
        prepare();

        // ros events
        rosPollEvents();

        // end exclusive access (allow simulation thread to run)
        mtx.unlock();

        // render while simulation is running
        render(window);
    }

    // stop simulation thread
    settings.exitrequest = 1;
    simthread.join();

    // delete everything we allocated
    uiClearCallback(window);
    mj_deleteData(d);
    mj_deleteModel(m);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    delete[] ctrl_command;
    delete[] ctrl_command2;
    ctrl_command = nullptr;
    ctrl_command2 = nullptr;

    // deactive MuJoCo
    // mj_deactivate();

    std_msgs::String pmsg;
    pmsg.data = std::string("terminate");
    sim_command_pub.publish(pmsg);

#ifdef COMPILE_SHAREDMEMORY
        deleteSharedMemory(shm_msg_id, mj_shm_);
#endif
// terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

    return 0;
}
