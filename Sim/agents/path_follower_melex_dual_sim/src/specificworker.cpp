/*
 *    Copyright (C) 2020 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <cppitertools/zip.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/range.hpp>
#include <algorithm>
#include <QPointF>
#include <ranges>
#include <Eigen/Geometry>
#include <cppitertools/cycle.hpp>

SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    qRegisterMetaType<std::uint64_t>("std::uint64_t");
    this->startup_check_flag = startup_check;
    QLoggingCategory::setFilterRules("*.debug=false\n");
}
/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
    G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    conf_params = std::make_shared<RoboCompCommonBehavior::ParameterList>(params);
    try
    {
        std::setlocale(LC_NUMERIC, "C");
        agent_name = params["agent_name"].value;
        agent_id = stoi(params["agent_id"].value);
        tree_view = params["tree_view"].value == "true";
        graph_view = params["graph_view"].value == "true";
        qscene_2d_view = params["2d_view"].value == "true";
        osg_3d_view = params["3d_view"].value == "true";
        acc_from_pose = params["acc_from_pose"].value == "true";
        robot_name = params["robot_name"].value;
        if (robot_name == "robot_1")

            another_robot = "robot_2";
        else
            another_robot = "robot_1";


        robot_mind_name = params["mind_name"].value;
        current_path_name = params["current_path_name"].value;
        task_name = params["task_name"].value;

        consts.max_adv_speed = stof(params.at("max_advance_speed").value);
        consts.max_rot_speed = stof(params.at("max_rotation_speed").value);
        consts.max_side_speed = stof(params.at("max_side_speed").value);
        consts.robot_length = stof(params.at("robot_length").value);
        consts.robot_width = stof(params.at("robot_width").value);
        consts.robot_radius = stof(params.at("robot_radius").value);
        consts.lateral_correction_gain = stof(params.at("lateral_correction_gain").value);
        consts.lateral_correction_for_side_velocity = stof(params.at("lateral_correction_for_side_velocity").value);
        consts.rotation_gain = std::stof(params.at("rotation_gain").value);
        consts.times_final_distance_to_target_before_zero_rotation = stof(params.at("times_final_distance_to_target_before_zero_rotation").value);
        consts.advance_gaussian_cut_x = stof(params.at("advance_gaussian_out_x").value);
        consts.advance_gaussian_cut_y = stof(params.at("advance_gaussian_out_y").value);
        consts.final_distance_to_target = stof(params.at("final_distance_to_target").value); // mm
        consts.lookahead_distance = stof(params.at("lookahead_distance").value); // mm
    }
    catch (const std::exception &e)
    {
        std::cout << __FUNCTION__ << " Problem reading params" << std::endl;
        std::terminate();
    }
    return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << __FUNCTION__ << std::endl;
    this->Period = period;
    new_intention = false;

    if(this->startup_check_flag)
        this->startup_check();
    else
    {
        // create graph
        G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id); // Init nodes
        std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;

        // Graph viewer
        using opts = DSR::DSRViewer::view;
        int current_opts = 0;
        opts main = opts::none;
        if(tree_view)
            current_opts = current_opts | opts::tree;
        if(graph_view)
            current_opts = current_opts | opts::graph;
        if(qscene_2d_view)
            current_opts = current_opts | opts::scene;
        if(osg_3d_view)
            current_opts = current_opts | opts::osg;
        dsr_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
        setWindowTitle(QString::fromStdString(agent_name + "-" + std::to_string(agent_id)));

        //dsr update signals
        connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::add_or_assign_node_slot, Qt::QueuedConnection);
        //connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::add_or_assign_edge_slot);
        //connect(G.get(), &DSR::DSRGraph::update_attrs_signal, this, &SpecificWorker::add_or_assign_attrs_slot);
        //connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
        connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot, Qt::QueuedConnection);

        //Inner Api
        inner_eigen = G->get_inner_eigen_api();

        // self agent api
        agent_info_api = std::make_unique<DSR::AgentInfoAPI>(G.get());

        // Ignore attributes from G
        G->set_ignored_attributes<cam_rgb_att, cam_depth_att >();

        // Custom widget
        dsr_viewer->add_custom_widget_to_dock("Path follower", &custom_widget);

        widget_2d = qobject_cast<DSR::QScene2dViewer *>(dsr_viewer->get_widget(opts::scene));
        if (widget_2d != nullptr)
            widget_2d->set_draw_laser(false);

        // path planner
        path_follower_initialize();

        // check for existing path node
        if(auto paths = G->get_nodes_by_type(path_to_target_type_name); not paths.empty())
        {
            std::cout << paths.front().name() << paths.front().id() << std::endl;
            this->add_or_assign_node_slot(paths.front().id(), path_to_target_type_name);
        }


        //Initial Pose
        //acc calc
        old_robot_pose = inner_eigen->transform_axis(world_name, robot_name).value();
        last_robot_speed = 0;
        last_robot_ang_speed = 0;
        last_lineal_speed = 0;
        last_angular_speed = 0;

        this->Period = 100;
        std::cout<< __FUNCTION__ << "Initialization finished" << std::endl;
        timer.start(Period);
    }
}

void SpecificWorker::compute() {

    static std::vector<Eigen::Vector2f> path, cyclic_path;
    static Eigen::Vector2f vtp;
    static bool vtp_updated = false;

    check_task_coord_node();
    if( auto r = upflank.try_get(); r.has_value())
        new_intention = true;

    // Check for existing path_to_target_nodes
    if (auto path_o = path_buffer.try_get(); path_o.has_value()) // NEW PATH!
    {
        path.clear();
        path = path_o.value();
        if(new_intention)
        {
            if (is_cyclic)
                path.push_back(path[0]);
            path = path_generation(path, consts.robot_length/4);
            cyclic_path = path;

            vtp_updated = false;
            initial_cyclic_point = path[0];
            current_target = path.back();
            new_intention = false;
        }
        robot_is_active = true;

        if (widget_2d != nullptr){
            draw_path(path, &widget_2d->scene);
        }
    }

    if(auto node_path = G->get_node(current_path_name); node_path.has_value())
    {
        auto nose_3d = inner_eigen->transform(world_name, Mat::Vector3d(0, 500, 0), robot_name).value();
        auto robot_pose_3d = inner_eigen->transform(world_name, robot_name).value();
        auto robot_nose = Eigen::Vector2f(nose_3d.x(), nose_3d.y());
        auto robot_pose = Eigen::Vector2f(robot_pose_3d.x(), robot_pose_3d.y());
        auto speeds = std::make_tuple(0.0, 0.0, 0.0);

        if(!vtp_updated){
            auto closest_point_to_robot = std::ranges::min_element(path, [robot_nose](auto &a, auto &b) {
                return (robot_nose - a).norm() < (robot_nose - b).norm();
            });
            vtp = closest_point_to_robot[0];
            std::cout << "-----VTP UPDATED-------" << std::endl;
            vtp_updated = true;
        }
        if(is_cyclic.load())
        {
            path_point(cyclic_path,robot_nose,consts.lookahead_distance,vtp);
            speeds = update(cyclic_path, QPolygonF(), robot_pose, robot_nose, vtp);
        }
        else
        {
            remove_trailing_path(path, robot_pose);
            path_point(path,robot_nose,consts.lookahead_distance,vtp);
            speeds = update(path, QPolygonF(), robot_pose, robot_nose, vtp);
        }
        auto [adv_, side_, rot_] = speeds;

        auto speeds_sim = sim_factor(std::make_tuple(adv_, side_, rot_));
        //Cálculo de velocidades y envío al robot
        auto[adv, side, rot] =  send_command_to_robot(speeds_sim);
        std::cout << "Adv, Rot: "<< adv << " " << rot << std::endl;

        if(not robot_is_active)  // robot reached the target
        {
            if(is_cyclic.load())
            {
                robot_is_active = true;
            }
            else
            {
                if(auto path_d = G->get_node(current_path_name); path_d.has_value())
                {
                    G->delete_node(path_d.value().id());
                }
            }
        }
    }
    else // stop controlling
    {
        qDebug() << __FUNCTION__ << "No path_node found in G. Stopping the robot";
    }
    fps.print("FPS: ", [this](auto x){ dsr_viewer->set_external_hz(x);});

}

void SpecificWorker::check_task_coord_node()
{
    if(auto robot_node = G->get_node(robot_name); robot_node.has_value())
    {
        if (auto mind = G->get_node(robot_mind_name); mind.has_value())
        {
            if (auto task = G->get_node(task_name); task.has_value()) {
                if (auto task_dest = G->get_attrib_by_name<task_destination_values_att>(
                            task.value()); not task_dest.has_value()) {
                    task_point.x() = G->get_attrib_by_name<task_pickup_values_att>(task.value()).value().get()[0];

                    task_point.y() = G->get_attrib_by_name<task_pickup_values_att>(task.value()).value().get()[1];
                    cout << "TASK_POINTX" << task_point.x() << "TASK_POINTY" << task_point.y() << endl;
                }
            }
            else
            {
                if (auto occupied = G->get_attrib_by_name<robot_occupied_att>(robot_node.value()); !occupied.value() ) {
                    task_point.x() = 100000;
                    task_point.y() = 100000;
                }
            }
        }
    }
}

void SpecificWorker::path_follower_initialize()
{
    qDebug() << __FUNCTION__ << "Controller - ";
    robotBottomLeft = Mat::Vector3d(-consts.robot_width / 2, -consts.robot_length / 2, 0);
    robotBottomRight = Mat::Vector3d(consts.robot_width / 2, -consts.robot_length / 2, 0);
    robotTopRight = Mat::Vector3d(consts.robot_width / 2, consts.robot_length / 2, 0);
    robotTopLeft = Mat::Vector3d(-consts.robot_width / 2, consts.robot_length / 2, 0);
    qInfo() << __FUNCTION__ << "CONTROLLER: Params from config:" << consts.max_adv_speed << consts.max_rot_speed << consts.max_side_speed << consts.max_lag
            << consts.robot_radius;
}

void SpecificWorker::remove_trailing_path(const std::vector<Eigen::Vector2f> &path, const Eigen::Vector2f &robot_pose)
{
    // closest point to robot nose in path
    if(path.size() == 1) return;

    auto closest_point_to_robot = std::ranges::min_element(path, [robot_pose](auto &a, auto &b){ return (robot_pose - a).norm() < (robot_pose - b).norm();});
    std::vector<float> x_values;  x_values.reserve(path.size());
    std::transform(closest_point_to_robot, path.cend(), std::back_inserter(x_values),
                   [](const auto &value) { return value.x(); });
    std::vector<float> y_values;  y_values.reserve(path.size());
    std::transform(closest_point_to_robot, path.cend(), std::back_inserter(y_values),
                   [](const auto &value) { return value.y(); });


    if (auto node_path = G->get_node(current_path_name); node_path.has_value())
    {
        G->add_or_modify_attrib_local<path_x_values_att>(node_path.value(), x_values);
        G->add_or_modify_attrib_local<path_y_values_att>(node_path.value(), y_values);
        G->update_node(node_path.value());
    }
    else
        std::cout << __FUNCTION__ << "No path target " << std::endl;
}

std::vector<Eigen::Vector2f> SpecificWorker::cyclic_trailing_path(const std::vector<Eigen::Vector2f> &path, const Eigen::Vector2f &robot_pose)
{
    std::vector<Eigen::Vector2f> path_edited, short_path;
    std::vector<float> x_values;  x_values.reserve(path.size());
    std::vector<float> y_values;  y_values.reserve(path.size());

    for (int i=0 ; i<=10 ; i++){
        short_path.push_back(path[i]);
    }
    /////ESCRIBIR PATH CICLICO &path -> x_values, y_values
    for (auto &&i : iter::range(path.size())){
        Eigen::Matrix<float, 2, 1> a(x_values[i],y_values[i]);
        path_edited.push_back(a);
    }
    return path_edited;
}

float SpecificWorker::path_distance(const std::vector<Eigen::Vector2f> &path,const Eigen::Vector2f &robot_pose){

    auto pt1_x = path.cbegin()->x();
    auto pt1_y = path.cbegin()->y();
    auto pt2_x = path[1].x();
    auto pt2_y = path[1].y();
    Eigen::Vector2f v1(pt1_x, pt1_y);
    Eigen::Vector2f v2(pt2_x, pt2_y);
    auto dist = 0;
    if(path.size()>=2)
    {
        Eigen::ParametrizedLine<float, 2> line(v1, (v2-v1)/(v2 -v1).norm());
        dist = line.distance(robot_pose);
    }
    return dist;
}

float SpecificWorker::dist_along_path(const std::vector<Eigen::Vector2f> &path)
{
    float len = 0.0;
    for(auto &&p : iter::sliding_window(path, 2))
        len += (p[1]-p[0]).norm();
    return len;
}



std::tuple<float, float, float> SpecificWorker::update(const std::vector<Eigen::Vector2f> &path, const QPolygonF &laser_poly, const Eigen::Vector2f &robot_pose,
                                                       const Eigen::Vector2f &robot_nose, const Eigen::Vector2f &target)
{
    auto vtp = target;
    float advVel = 0.f, sideVel = 0.f, rotVel = 0.f;
    qDebug() << " Controller - "<< __FUNCTION__;
    static QGraphicsEllipseItem *target_scene;

    if(path.size() < 2 or (dist_along_path(path) < consts.final_distance_to_target))
    {
        qInfo() << __FUNCTION__ << " -------------- Target achieved -----------------";
        return std::make_tuple(0.0, 0.0, 0.0);
    }

    // Compute euclidean distance to target
    float euc_dist_to_target = (robot_nose - target).norm();

    //Paint VTP
    widget_2d->scene.removeItem(target_scene);
    target_scene = widget_2d->scene.addEllipse(-100, -100, 200, 200, QPen(QColor("Yellow")), QBrush(QColor("Yellow")));
    target_scene->setPos(vtp[0],vtp[1]);
    target_scene->setZValue(100);

    //Transform vtp to vehicle coordinate
    Eigen::Vector3d aux(vtp[0], vtp[1], 0.f);

    auto vtp_robot = inner_eigen->transform(robot_name, aux , world_name);


    qInfo() << __FUNCTION__ << "Robot VTP (x,y,z):" << vtp_robot.value().x() <<" "<< vtp_robot.value().y() ;
    // Compute wheels speed

    float DeltaX =vtp_robot.value().x()/1000;
    float DeltaY =vtp_robot.value().y()/1000;

    if(DeltaY<0)
        DeltaY=0;

    float LA2 = pow(DeltaX, 2.0) + pow(DeltaY, 2.0);
//    float Gamma = (2*DeltaX)/LA2; //coppelia robot
    float Gamma = -(2*DeltaX)/LA2; //real robot

    float Vd = (1+consts.robot_width*Gamma*0.5);
    float Vi = (1-consts.robot_width*Gamma*0.5);

    rotVel = (Vi-Vd)*consts.rotation_gain/consts.robot_width;

    /// limit angular  values to physical limits
    rotVel = std::clamp(rotVel, -consts.max_rot_speed, consts.max_rot_speed);
    /// Compute advance speed
    advVel = consts.max_adv_speed * (Vd + Vi) ;
    advVel = std::min(advVel *
                      exponentialFunction(rotVel, consts.advance_gaussian_cut_x, consts.advance_gaussian_cut_y, 0),
                      euc_dist_to_target);
    advVel = std::clamp(advVel, -consts.max_adv_speed, consts.max_adv_speed);

    // Bumper between cars
    cout << "DISTANCIA A PUNTO " << (robot_nose-vtp).norm() << "umbral " << consts.final_distance_to_target*2.5 << endl;
    auto robot_to_robot_distance = inner_eigen->transform(robot_name, another_robot).value();
    cout << "DE " + robot_name + "A " + another_robot + " COORD X " << robot_to_robot_distance.x() << " COOORD Y" << robot_to_robot_distance.y() << endl;
    auto distance_between_robots_v = Eigen::Vector2f(robot_to_robot_distance.x(), robot_to_robot_distance.y());
    auto distance_robots = distance_between_robots_v.norm();
    auto angle_between_robots = atan2(robot_to_robot_distance.x(), robot_to_robot_distance.y());
    cout << "ANGULO ENTRE ROBOTS" << angle_between_robots << endl;
    cout << "DISTANCIA ENTRE ROBOTS: " << distance_robots << endl;

    //Bumper for people
    auto person_nodes = G->get_nodes_by_type("person");
    for(const auto &person : person_nodes){
        if(auto person_car_pos_diff = inner_eigen->transform(world_name, person.name()); person_car_pos_diff.has_value()){
            if(person_car_pos_diff.value().norm() > 2000)
            {
                qInfo() << "PÊRSON EN EL MEDIO:", person_car_pos_diff.value().norm();
                break;
            }
        }
    }

    if ((robot_nose-vtp).norm()< consts.final_distance_to_target*3.5)  { //real robot
//    if ((robot_nose-vtp).norm()< consts.final_distance_to_target) { //coppelia robot
        std::cout << "---------------VTP REACHED------------------: " << (robot_nose-vtp).norm() <<std::endl;
        check_task_att( false);
        return std::make_tuple(0.0, 0.0, 0.0);
    }
    else if ( distance_robots <= 8000.0 and abs(angle_between_robots) < M_PI_4){
        cout << "ROBOT STOPED BECAUSE COLLISION COULD ----------------" << endl;
        check_task_att(true);
        //return std::make_tuple(0.0, 0.0, 0.0);
    }
    else{
        check_task_att(true);
    }

    return std::make_tuple(advVel, sideVel, rotVel);
}


std::tuple<float,float,float> SpecificWorker::sim_factor(std::tuple<float,float,float> speeds)
{
    auto &[adv_, side_, rot_] = speeds;
    float k_percentage = 1;
    float k_time = 1;
    float k_brake = 1;
    if (auto collision_edge = G->get_edges_by_type("virtual_collision"); not collision_edge.empty())
    {
        for (const auto &edge: collision_edge) {
            auto attr = edge.attrs();
            for (auto &[k, v]: attr) {
                if (k == "collision") {
//                    std::cout<<v.value().index()<<endl;
                    auto collision_percentage = std::get<8>(v.value());
                    k_percentage = exponentialFunction(collision_percentage, 0.6, 0.4, 0);
                } else if (k == "time_collision") {
                    auto collision_time = std::get<3>(v.value());
                    auto min_time = *std::min_element(collision_time.begin(), collision_time.end());
                    std::cout<<collision_time.size()<<min_time<<endl;
                    k_time = 1- exponentialFunction(min_time ,10,0.4,0);
                }

            }
        }
    }
    if (auto brake_edges = G->get_edges_by_type("virtual_brake"); not brake_edges.empty()) {
        auto brake_edge = brake_edges[0];
        auto brake_attr = brake_edge.attrs();
        for (auto &[k, v]: brake_attr) {
            if (k == "brake") {
//                    std::cout<<v.value().index()<<endl;
                auto brake_percentage = std::get<8>(v.value());
                k_brake = exponentialFunction(brake_percentage, 0.3, 0.9, 0);
            }
        }
    }
    auto min = std::min({k_percentage, k_time, k_brake});
    std::cout << "Simulation factor" << std::min({k_percentage, k_time, k_brake}) << endl;
    adv_ *= min;
    std::cout << "PERC " << k_percentage << " TIME " << k_time << " BRAKE " << k_brake << endl;
    return std::make_tuple(adv_, side_, rot_);
}


std::tuple<float, float, float> SpecificWorker::send_command_to_robot(const std::tuple<float, float, float> &speeds) //adv, side, rot
{
    auto &[adv_, side_, rot_] = speeds;
    if(auto robot_node = G->get_node(robot_name); robot_node.has_value())
    {
        G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node.value(), (float) adv_);
        G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot_node.value(), (float) rot_);
        G->add_or_modify_attrib_local<robot_ref_side_speed_att>(robot_node.value(), (float) side_);
        G->update_node(robot_node.value());
    }
    else qWarning() << __FUNCTION__ << "No robot node found";
    return std::make_tuple(adv_, side_, rot_);
}

// compute max de gauss(value) where gauss(x)=y  y min
float SpecificWorker::exponentialFunction(float value, float xValue, float yValue, float min)
{
    if (yValue <= 0)
        return 1.f;
    float landa = -fabs(xValue) / log(yValue);
    float res = exp(-fabs(value) / landa);
    //std::cout << "lambda: " << landa << "res" << std::endl;
    return std::max(res, min);
}

float SpecificWorker::rewrapAngleRestricted(const float angle)
{
    if (angle > M_PI)
        return angle - M_PI * 2;
    else if (angle < -M_PI)
        return angle + M_PI * 2;
    else
        return angle;
}

void SpecificWorker::print_current_state(const std::vector<Eigen::Vector2f> &path, Eigen::Matrix<float, 2, 1> robot_pose, float adv, float side, float rot)
{
    std::cout << "---------------------------" << std::endl;
    std::cout << "Robot position: " << std::endl;
    std::cout << "\t " << robot_pose.x() << ", " << robot_pose.y() << std::endl;
    std::cout << "Target position: " << std::endl;
    std::cout << "\t " << path.back().x() << ", " << path.back().y() << std::endl;
    std::cout << "Dist to target: " << std::endl;
    std::cout << "\t " << dist_along_path(path) << std::endl;
    std::cout << "Ref speeds:  " << std::endl;
    std::cout << "\t Advance-> " << adv << std::endl;
    std::cout << "\t Side -> " << side << std::endl;
    std::cout << "\t Rotate -> " << rot << std::endl;
    std::cout << "\tRobot_is_active -> " << std::boolalpha << robot_is_active << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
/// Asynchronous changes on G nodes from G signals
/////////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::add_or_assign_node_slot(const std::uint64_t id, const std::string &type)
{
    //check node type
    if (type == path_to_target_type_name)
    {
        if( auto node = G->get_node(id); node.has_value())
        {
            auto x_values = G->get_attrib_by_name<path_x_values_att>(node.value());
            auto y_values = G->get_attrib_by_name<path_y_values_att>(node.value());
            auto is_cyclic_o = G->get_attrib_by_name<path_is_cyclic_att>(node.value());
            if(is_cyclic_o.has_value())
                is_cyclic.store(is_cyclic_o.value());
            else is_cyclic.store(false);
            if(x_values.has_value() and y_values.has_value())
            {
                auto x = x_values.value().get();
                auto y = y_values.value().get();
                std::vector<Eigen::Vector2f> path; path.reserve(x.size());
                for (auto &&[x, y] : iter::zip(x, y))
                    path.push_back(Eigen::Vector2f(x, y));

                path_buffer.put(std::move(path));
                auto t_x = G->get_attrib_by_name<path_target_x_att>(node.value()); //
                auto t_y = G->get_attrib_by_name<path_target_y_att>(node.value());
                if(t_x.has_value() and t_y.has_value())
                    current_target = Eigen::Vector2f(t_x.value(), t_y.value());
            }
        }
    }
    if(type == intention_type_name)
    {
        if( auto node = G->get_node(id); node.has_value())
        {
            upflank.put(true);
        }
    }
}

void SpecificWorker::del_node_slot(std::uint64_t from)
{
    if( auto node = G->get_node(current_path_name); not node.has_value())
    {
        qInfo() << __FUNCTION__ << "Path node deleter. Aborting control";
        send_command_to_robot(std::make_tuple(0.0,0.0,0.0));
    }
    if ( auto node = G->get_node(current_intention_name); not node.has_value())
    {
        downflank.put(true);
    }
}

///////////////////////////////////////////////////
/// GUI
//////////////////////////////////////////////////

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}




void SpecificWorker::draw_path(std::vector<Eigen::Vector2f> &path, QGraphicsScene* viewer_2d)
{
    static std::vector<QGraphicsLineItem *> scene_road_points;

    //clear previous points

    for (QGraphicsLineItem* item : scene_road_points)
    {
        viewer_2d->removeItem(item);
        delete item;
    }
    scene_road_points.clear();

    /// Draw all points
    QGraphicsLineItem *line1, *line2;
    std::string color;
    for(auto &&p_pair : iter::sliding_window(path, 2))
    {
        Mat::Vector2d a_point(p_pair[0].x(), p_pair[0].y());
        Mat::Vector2d b_point(p_pair[1].x(), p_pair[1].y());
        Mat::Vector2d dir = a_point - b_point;
        Mat::Vector2d dir_perp = dir.unitOrthogonal();
        Eigen::ParametrizedLine segment = Eigen::ParametrizedLine<double, 2>::Through(a_point, b_point);
        Eigen::ParametrizedLine<double, 2> segment_perp((a_point+b_point)/2, dir_perp);
        auto left = segment_perp.pointAt(50);
        auto right = segment_perp.pointAt(-50);
        QLineF qsegment(QPointF(a_point.x(), a_point.y()), QPointF(b_point.x(), b_point.y()));
        QLineF qsegment_perp(QPointF(left.x(), left.y()), QPointF(right.x(), right.y()));

        line1 = viewer_2d->addLine(qsegment, QPen(QBrush(QColor(QString::fromStdString(color))), 20));
        line2 = viewer_2d->addLine(qsegment_perp, QPen(QBrush(QColor(QString::fromStdString("#F0FF00"))), 20));

        scene_road_points.push_back(line1);
        scene_road_points.push_back(line2);
    }
}

void SpecificWorker::path_point(std::vector<Eigen::Vector2f> &path, Eigen::Vector2f robot_nose, float distance, Eigen::Vector2f &vtp) {

    bool check = false;
    if (dist_along_path(path)>distance)
    {
        for (auto &&p : iter::cycle(path))
        {
            qInfo() << "VTP" << vtp.x() << vtp.y();
            qInfo() << "p" << p.x() << p.y();
            qInfo() << "CYVLE";
            if (p==vtp || check)
            {
                qInfo() << "p==vtp || check";
                check = true;

                if ((p - task_point).norm()<500)// cambiar aqui esto
                {
                    vtp = task_point;
                    std::cout << "-----TASK POINT-----: " << vtp << std::endl;
                    break;
                }
                else if ((robot_nose - p).norm()>=distance)
                {
                    vtp = p;
                    std::cout << "-----NEW VTP-----: " << vtp << std::endl;
                    break;
                }
                qInfo() << "NOT BReAKED";
            }
        }
    }
    else
        vtp = path[-1];
}

vector<Eigen::Vector2f> SpecificWorker::path_generation(const vector<Eigen::Vector2f> &path, float resolution){
    vector<Eigen::Vector2f> new_path;
    for (auto &&p : iter::sliding_window(path, 2))
    {
        auto n_points = (p[1] - p[0]).norm()/resolution;
        auto lambda = 1/n_points;
        for(float i=0.0 ; i <= 1.0; i+=lambda){
            auto r = (1 - i)*p[0]+ i * p[1];
            new_path.push_back(r);
        }
    }
    return new_path;
}

void SpecificWorker::check_task_att( bool movement)
{
    if (auto task = G->get_node(task_name); task.has_value())
    {
        G->add_or_modify_attrib_local<task_movement_att>(task.value(), movement);

        if (auto task_dest = G->get_attrib_by_name<task_destination_values_att>(task.value()); task_dest.has_value() and task_dest.value().get()[0] != task_point.x()){
            cout << "ENTRA" << endl;
            task_point.x() = G->get_attrib_by_name<task_destination_values_att>(task.value()).value().get()[0];
            cout << "PUNTO DESTINO X" <<  task_point.x() << endl;
            task_point.y()= G->get_attrib_by_name<task_destination_values_att>(task.value()).value().get()[1];
            cout << "PUNTO DESTINO Y" <<  task_point.y() << endl;

        }
        else if (auto task_dest = G->get_attrib_by_name<task_destination_values_att>(task.value()); task_dest.has_value() and task_dest.value().get()[0] == task_point.x())
        {
            if(auto on_movement = G->get_attrib_by_name<task_movement_att>(task.value()); on_movement.has_value() and on_movement==false) {
                G->add_or_modify_attrib_local<task_completed_att>(task.value(), true);
                G->update_node(task.value());
                cout << "---------------------------------UPDATE_NODE_TASK------------------------------" + task_name << endl;
            }
        }
        G->update_node(task.value());
    }
}
