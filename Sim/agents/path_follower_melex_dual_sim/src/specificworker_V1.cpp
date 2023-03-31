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
using json = nlohmann::json;


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
    last_error = 0;
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

    //Opt Variables
    std::string variable_file = "opt_variable_file";
    try
    {
        ifstream f_ent;
        f_ent.open("etc/" + variable_file +".json");
        f_ent >> json_file;
        f_ent.close(); ///Cerramos el .json

        //Opt variables load from json
        json_load("rotation_gain",json_file);
        json_load("advance_gaussian_cut_x",json_file);
        json_load("advance_gaussian_cut_y",json_file);
        json_load("lookahead_distance",json_file);


        for(std::map<string, OPT_PARAMS>::iterator it = opt_variables.begin(); it != opt_variables.end(); ++it)
            variable_vector.push_back(it->first);
        //std::sample(variable_vector.begin(), variable_vector.end(), std::back_inserter(variable_vector_random),variable_vector.size(), std::mt19937{std::random_device{}()});
        //std::random_shuffle(variable_vector.begin(), variable_vector.end());

        auto rd = std::random_device {};
        auto rng = std::default_random_engine { rd() };
        std::shuffle(std::begin(variable_vector), std::end(variable_vector), rng);


        std::cout << "FIRST OPT VARIABLE: "<<variable_vector[0] << std::endl;

        error_data.traveled_distance = json_file["error"]["traveled_distance"];
        error_data.accumulated_path_distance = json_file["error"]["accumulated_path_distance"];
        error_data.accumulated_lineal_acceleration = json_file["error"]["accumulated_lineal_acceleration"];
        error_data.accumulated_angular_acceleration = json_file["error"]["accumulated_angular_acceleration"];

    }
    catch(const std::exception& e)
    {
        std::cout << e.what() << std::endl;
        qFatal("Unable to open file, please check config file");
    }
    reset_error(error_data);
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


        // timeseries
        customPlot = new QCustomPlot(custom_widget.frame_plot);
        customPlot->resize(custom_widget.frame_plot->size());
        customPlot->addGraph(); // blue line
        customPlot->graph(0)->setPen(QPen(QColor(0, 0, 255)));

        customPlot->xAxis->setRange(0, 10000);
        customPlot->graph(0)->rescaleAxes();
        customPlot->graph(0)->setName("Distance to path");

        //customPlot->axisRect()->setupFullAxesBox();
        customPlot->yAxis->setRange(0, 1500);
        customPlot->legend->setVisible(true);
        customPlot->legend->setFont(QFont("Helvetica",9));
        connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
        connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));
        customPlot->show();

        // timeseries Acceleration
        customPlot1 = new QCustomPlot(custom_widget.frame_acc);
        customPlot1->resize(custom_widget.frame_plot->size());
        customPlot1->addGraph(); // blue line
        customPlot1->graph(0)->setPen(QPen(QColor(255, 0, 0)));
        customPlot1->addGraph(); // xColor line
        customPlot1->graph(1)->setPen(QPen(QColor(0, 255, 0)));
        customPlot1->xAxis->setRange(0, 10000);

        customPlot1->graph(0)->rescaleAxes();
        customPlot1->graph(0)->setName("Aceleration");
        customPlot1->graph(1)->rescaleAxes();
        customPlot1->graph(1)->setName("Angular aceleration");
        //customPlot->axisRect()->setupFullAxesBox();
        customPlot1->yAxis->setRange(-10, 10);
        customPlot1->legend->setVisible(true);
        customPlot1->legend->setFont(QFont("Helvetica",9));
        connect(customPlot1->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot1->xAxis2, SLOT(setRange(QCPRange)));
        connect(customPlot1->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot1->yAxis2, SLOT(setRange(QCPRange)));
        customPlot1->show();

        //Initial Pose
        //acc calc
        old_robot_pose = inner_eigen->transform_axis(world_name, robot_name).value();
        last_robot_speed = 0;
        last_robot_ang_speed = 0;
        last_lineal_speed = 0;
        last_angular_speed = 0;

        this->Period = 200;
        std::cout<< __FUNCTION__ << "Initialization finished" << std::endl;
        timer.start(Period);
    }
}

void SpecificWorker::compute() {

    static std::vector<Eigen::Vector2f> path, cyclic_path;

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
            path = path_generation(path, consts.robot_length/2);
            cyclic_path = path;

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
        auto robot_pose_matrix = inner_eigen->transform_axis(world_name, robot_name).value();
        auto speeds = std::make_tuple(0.0, 0.0, 0.0);


        if(is_cyclic.load())
        {
            cyclic_path = cyclic_trailing_path(cyclic_path,robot_nose);
            Eigen::Matrix<float, 2, 1> vtp = path_point(cyclic_path,robot_nose,consts.lookahead_distance);
            speeds = update(cyclic_path, QPolygonF(), robot_pose, robot_nose, vtp);
        }
        else
        {
            remove_trailing_path(path, robot_pose);
            Eigen::Matrix<float, 2, 1> vtp = path_point(path,robot_nose,consts.lookahead_distance);
            speeds = update(path, QPolygonF(), robot_pose, robot_nose, vtp);
        }
        //Cálculo de velocidades y envío al robot

        auto[adv, side, rot] =  send_command_to_robot(speeds);
        std::cout << "Adv, Rot: "<< adv << " " << rot << std::endl;

//        //Acc estimation
//        Eigen::Vector2f pose_prueba = {old_robot_pose.x(),old_robot_pose.y()};
//        last_robot_speed = ((robot_pose - pose_prueba).norm())/ this->Period;
//        last_robot_ang_speed = (robot_pose_matrix[5]-old_robot_pose[5])*1000/ this->Period;
//
//        auto lineal_acc = (last_robot_speed - last_lineal_speed)/ this->Period;
//        auto ang_acc =  (last_robot_ang_speed-last_angular_speed)*1000/ this->Period;
//
//        //Update error
//        auto error = update_error_values(path, robot_pose, speeds, lineal_acc, ang_acc);

//        old_robot_pose = robot_pose_matrix;
//        last_lineal_speed = last_robot_speed;
//        last_angular_speed = last_robot_ang_speed;

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
        //if (auto node_paths = G->get_nodes_by_type(path_to_target_type_name); not node_paths.empty())
    {
        //auto path_to_target_node = node_paths.front();
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

    auto closest_point_to_robot = std::ranges::min_element(short_path, [robot_pose](auto &a, auto &b){ return (robot_pose - a).norm() < (robot_pose - b).norm();});


//    if(path_distance(path,robot_pose)>=consts.lookahead_distance or closest_point_to_robot==path.cbegin() or closest_point_to_robot==path.cbegin()+1)
    if(1)
    {
//        std::transform(closest_point_to_robot, path.cend(), std::back_inserter(x_values),
//                       [](const auto &value) { return value.x(); });
//        std::transform(closest_point_to_robot, path.cend(), std::back_inserter(y_values),
//                       [](const auto &value) { return value.y(); });
//
//        std::transform(path.begin(), closest_point_to_robot, std::back_inserter(x_values),
//                       [](const auto &value) { return value.x(); });
//        std::transform(path.cbegin(), closest_point_to_robot, std::back_inserter(y_values),
//                       [](const auto &value) { return value.y(); });
        crossroad = false;
    }
    else
    {
        if(0)
        {
            std::cout << "----------Crossroad----------" << std::endl;

            std::transform(path.cbegin() + 1, path.cend(), std::back_inserter(x_values),
                           [](const auto &value) { return value.x(); });
            std::transform(path.cbegin() + 1, path.cend(), std::back_inserter(y_values),
                           [](const auto &value) { return value.y(); });
            std::transform(path.begin(), path.cbegin() + 1, std::back_inserter(x_values),
                           [](const auto &value) { return value.x(); });
            std::transform(path.cbegin(), path.cbegin() + 1, std::back_inserter(y_values),
                           [](const auto &value) { return value.y(); });
            crossroad = true;
        }
        else
        {
            std::transform(path.cbegin(), path.cend(), std::back_inserter(x_values),
                           [](const auto &value) { return value.x(); });
            std::transform(path.cbegin(), path.cend(), std::back_inserter(y_values),
                           [](const auto &value) { return value.y(); });
            std::transform(path.begin(), path.cbegin(), std::back_inserter(x_values),
                           [](const auto &value) { return value.x(); });
            std::transform(path.cbegin(), path.cbegin(), std::back_inserter(y_values),
                           [](const auto &value) { return value.y(); });
        }
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

void polyfit(	const std::vector<double> &t, const std::vector<double> &v, std::vector<double> &coeff, int order)
{
    // Create Matrix Placeholder of size n x k, n= number of datapoints, k = order of polynomial, for exame k = 3 for cubic polynomial
    Eigen::MatrixXd T(t.size(), order + 1);
    Eigen::VectorXd V = Eigen::VectorXd::Map(&v.front(), v.size());
    Eigen::VectorXd result;

    // check to make sure inputs are correct
    assert(t.size() == v.size());
    assert(t.size() >= order + 1);
    // Populate the matrix
    for(size_t i = 0 ; i < t.size(); ++i)
    {
        for(size_t j = 0; j < order + 1; ++j)
        {
            T(i, j) = pow(t.at(i), j);
        }
    }
    std::cout<<T<<std::endl;

    // Solve for linear least square fit
    result  = T.householderQr().solve(V);
    coeff.resize(order+1);
    for (int k = 0; k < order+1; k++)
    {
        coeff[k] = result[k];
    }

}


void SpecificWorker::realtime_data_slot(double value1, double value2, double value3)
{
    static double index = 0;
    customPlot->graph(0)->addData(index++, value1);
    customPlot1->graph(0)->addData(index++, value2);
    customPlot1->graph(1)->addData(index++, value3);
    customPlot->xAxis->setRange(index, 400, Qt::AlignRight);
    customPlot->replot();

    customPlot1->xAxis->setRange(index, 400, Qt::AlignRight);
    customPlot1->replot();
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
    float Gamma = -(2*DeltaX)/LA2;

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

//    std::cout << " DeltaY " << DeltaY << " DeltaX: " << DeltaX << std::endl;
//    std::cout << " Vd " << Vd << " Vi: " << Vi << std::endl;
//    std::cout << " Adv: " << advVel << " rotVel: " << rotVel << std::endl;

    return std::make_tuple(advVel, sideVel, rotVel);
}


std::tuple<float, float, float> SpecificWorker::update_error_values(const std::vector<Eigen::Vector2f> &path, const Eigen::Vector2f &robot_pose, const std::tuple<float, float, float> speeds, const float acc, const float acc_ang) {

    auto robot_Adv = 0.0;
    auto robot_Rot = 0.0;
    auto error=std::make_tuple(0.0, 0.0, 0.0);
    float distance = dist_along_path(path);

    if(acc_from_pose){
        if(auto robot_node = G->get_node(robot_name); robot_node.has_value()){
            float dist = path_distance(path, robot_pose);
            realtime_data_slot(dist, acc*1000, acc_ang);
            error=std::make_tuple(dist,acc*1000,acc_ang);
            update_error(error,distance);
        }
    }
    else{
        if(auto robot_node = G->get_node(robot_name); robot_node.has_value()){
            auto robot_Adv_k = G->get_attrib_by_name<robot_ref_adv_speed_att>(robot_node.value());
            robot_Adv =  robot_Adv_k.value();
            auto robot_Rot_k = G->get_attrib_by_name<robot_ref_rot_speed_att>(robot_node.value());
            robot_Rot = robot_Rot_k.value();
            //float robot_Side = G->get_attrib_by_name<robot_ref_side_speed_att>(robot_node.value());
        }

        float dist = path_distance(path, robot_pose);
        float acceleration = ((get<0>(speeds) - robot_Adv)/ this->Period);
        float ang_acceleration = ((get<1>(speeds) - (float)robot_Rot) * 1000 / this->Period);

        realtime_data_slot(dist, acceleration, ang_acceleration);
        error=std::make_tuple(dist, acceleration, ang_acceleration);
        update_error(error,distance);
    }
    return error;
}

std::vector<QPointF> SpecificWorker::get_points_along_extended_robot_polygon(int offset, int chunck)
{
    static QGraphicsRectItem *poly_draw = nullptr;
    std::vector<QPointF> poly;
    QRectF rp(QPointF(robotTopLeft.x(),robotTopLeft.y()), QPointF(robotBottomRight.x(), robotBottomRight.y()));
    rp.adjust(-offset, offset, offset, -offset);
    QLineF bottom(rp.bottomLeft(), rp.bottomRight());
    QLineF left(rp.topLeft(), rp.bottomLeft());
    QLineF top(rp.topLeft(), rp.topRight());
    QLineF right(rp.topRight(), rp.bottomRight());
    for(auto i : iter::range(0.f, 1.f, (float)(1.f/(top.length()/chunck))))
    {
        poly.push_back(top.pointAt(i));
        poly.push_back(bottom.pointAt(i));
        //auto aux = inner_eigen->transform(world_name, Eigen::Vector3d{point.x(), point.y(), 0.f}, robot_name);
        //poly << QPointF(aux.value().x(), aux.value().y());
        //point =  bottom.pointAt(i);
        //aux = inner_eigen->transform(world_name, Eigen::Vector3d{point.x(), point.y(), 0.f}, robot_name);
        //poly << QPointF(aux.value().x(), aux.value().y());
    }
    for(auto i : iter::range(0.f, 1.f, (float)(1.f/(left.length()/chunck))))
    {
        poly.push_back(left.pointAt(i));
        poly.push_back(right.pointAt(i));
        //        auto point =  left.pointAt(i);
        //        auto aux = inner_eigen->transform(world_name, Eigen::Vector3d{point.x(), point.y(), 0.f}, robot_name);
        //        poly << QPointF(aux.value().x(), aux.value().y());
        //        point =  right.pointAt(i);
        //        aux = inner_eigen->transform(world_name, Eigen::Vector3d{point.x(), point.y(), 0.f}, robot_name);
        //        poly << QPointF(aux.value().x(), aux.value().y());
    }
    if(poly_draw != nullptr)
    {
        widget_2d->scene.removeItem(poly_draw);
        delete poly_draw;
    }
    poly_draw = widget_2d->scene.addRect(rp, QPen(QColor("blue"), 10));
    auto robot_pos = inner_eigen->transform_axis(world_name, robot_name).value();
    poly_draw->setRotation(qRadiansToDegrees(robot_pos[5]));
    poly_draw->setPos(QPointF(robot_pos[0], robot_pos[1]));
    return poly;
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

///////////////////////////////////////////////////////
//// Check new target from mouse
///////////////////////////////////////////////////////
void SpecificWorker::new_target_from_mouse(int pos_x, int pos_y, int id)
{
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

                auto cyclic = G->get_attrib_by_name<path_is_cyclic_att>(node.value()); //
            }
        }
    }
    if(type == intention_type_name)
    {
        if( auto node = G->get_node(id); node.has_value())
        {
            upflank.put(true);
            std::cout << "NEW INTENTION NODE" << std::endl;
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
/**************************************/

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

/// Compute bumper-away speed
//QVector2D total{0, 0};
//    const auto &[angles, dists] = laser_data;
//    for (const auto &[angle, dist] : iter::zip(angles, dists))
//    {
//        float limit = (fabs(ROBOT_LENGTH / 2.f * sin(angle)) + fabs(ROBOT_LENGTH / 2.f * cos(angle))) + 200;
//        float diff = limit - dist;
//        if (diff >= 0)
//            total = total + QVector2D(-diff * cos(angle), -diff * sin(angle));
//    }

/// Compute bumper away speed for rectangular shape
// get extendedrobot polygon in worlds's coordinate frame
//    std::vector<QPointF> rp = get_points_along_extended_robot_polygon(200, 40);
//    for (const auto &p : rp)
//        if(not laser_poly.containsPoint(p, Qt::OddEvenFill))
//            total = total + QVector2D(p);
//    qInfo() << __FUNCTION__ << total;
//    sideVel = std::clamp(total.y(), -MAX_SIDE_SPEED, MAX_SIDE_SPEED);

void SpecificWorker::update_error(const std::tuple<float,float,float> error_inst, float dist_along_path)
{
    error_data.accumulated_path_distance = error_data.accumulated_path_distance + std::get<0>(error_inst);
    error_data.accumulated_lineal_acceleration = error_data.accumulated_lineal_acceleration + std::abs(std::get<1>(error_inst));
    error_data.accumulated_angular_acceleration = error_data.accumulated_angular_acceleration + std::abs(std::get<2>(error_inst));
}

void SpecificWorker::reset_error(ERROR_DATA &error_data){

    error_data.accumulated_path_distance = 0.0;
    error_data.accumulated_lineal_acceleration = 0.0;
    error_data.accumulated_angular_acceleration = 0.0;

    std::cout << "RESET ERROR_DATA" << std::endl;
}

void SpecificWorker::dump_error(){
    std::string variable_file = "opt_variable_file";

    json_dump("rotation_gain",json_file);
    json_dump("advance_gaussian_cut_x",json_file);
    json_dump("advance_gaussian_cut_y",json_file);
    json_dump("lookahead_distance",json_file);

    //Error dump
    json_file["error"]["traveled_distance"] =  dist_along_path(intention_path);
    json_file["error"]["accumulated_path_distance"] = error_data.accumulated_path_distance;
    json_file["error"]["accumulated_lineal_acceleration"] = error_data.accumulated_lineal_acceleration;
    json_file["error"]["accumulated_angular_acceleration"] = error_data.accumulated_angular_acceleration;

    //Json write
    std::ofstream output("etc/" + variable_file +".json");
    output << std::setw(4) << json_file << std::endl;
    output.close();
}

Eigen::Matrix<float, 2, 1> SpecificWorker::path_point(std::vector<Eigen::Vector2f> &path, Eigen::Vector2f robot_nose, float distance) {

    auto vtp = path[0];
    bool aux_bool = false;
    float len = 0.0;
    std::vector<float> x_values;  x_values.reserve(path.size());
    std::vector<float> y_values;  y_values.reserve(path.size());

    // closest point to robot nose in path
    auto closest_point_to_nose = std::ranges::min_element(path, [robot_nose](auto &a, auto &b) {
        return (robot_nose - a).norm() < (robot_nose - b).norm();
    });


//    if (dist_along_path(path) >= distance) {
//        if (path_distance(path, robot_nose) >= distance) {
//            for (auto &&p : iter::sliding_window(path, 2)) {
//                if ((p[0].y() == closest_point_to_nose->y() && p[0].x() == closest_point_to_nose->x()) || aux_bool) {
//                    aux_bool = true;
//                    len += (p[1] - p[0]).norm();
//                    vtp = p[1];
//                    if (len >= distance && (p[1] - robot_nose).norm()>=distance) {
//                        vtp = p[1];
//                        break;
//                    }
//                }
//            }
//        }
//        else
//        {
//            for (auto &&p : iter::sliding_window(path, 2)) {
//                len += (p[1] - p[0]).norm();
//                vtp = p[1];
//                if (len >= distance && (p[1] - robot_nose).norm()>=distance) {
//                    vtp = p[1];
//                    break;
//                }
//            }
//        }
//    }
//    else
//        vtp = path[-1];

    if (dist_along_path(path) >= distance) {
        if (path_distance(path, robot_nose) >= distance) {
            for (auto &&p : iter::sliding_window(path, 2)) {
                if ((p[0].y() == closest_point_to_nose->y() && p[0].x() == closest_point_to_nose->x()) || aux_bool) {
                    aux_bool = true;
                    len += (p[1] - p[0]).norm();
                    vtp = p[1];
                    if (len >= distance && (p[1] - robot_nose).norm()>=distance) {
                        vtp = p[1];
                        break;
                    }
                }
            }
        }
        else
        {
            for (auto &&p : iter::sliding_window(path, 1)) {
                len = (robot_nose - p[0]).norm();
                if (len >= distance) {
                    vtp = p[0];
                    break;
                }
            }
        }
    }
    else
        vtp = path[-1];


    return vtp;
}

void SpecificWorker::json_load(std::string variable_name, json j){
    OPT_PARAMS variable_data;

    variable_data.min = j[variable_name]["min"];
    variable_data.max = j[variable_name]["max"];
    variable_data.step = j[variable_name]["step"];
    variable_data.initial_value = j[variable_name]["initial_value"];
    variable_data.last_value = j[variable_name]["last_value"];
    variable_data.last_step = j[variable_name]["last_step"];

    opt_variables.emplace(variable_name, variable_data);
}

void SpecificWorker::json_dump(std::string variable_name, json &j){
    j[variable_name]["min"] = opt_variables[variable_name].min;
    j[variable_name]["max"] = opt_variables[variable_name].max;
    j[variable_name]["step"] = opt_variables[variable_name].step;
    j[variable_name]["initial_value"] = opt_variables[variable_name].initial_value;
    j[variable_name]["last_value"] = opt_variables[variable_name].last_value;
    j[variable_name]["last_step"] = opt_variables[variable_name].last_step;
}

void SpecificWorker::variable_load(std::string variable_name){

    std::map<string, string> m;
    std::vector<string> v;

//    for(map<int,int>::iterator it = m.begin(); it != m.end(); ++it) {
//        v.Push_back(it->first);
//        cout << it->first << "\n";
//    }
    consts.advance_gaussian_cut_x = opt_variables[variable_name].last_value;
}

//bool SpecificWorker::optimizer(const std::vector<Eigen::Vector2f> &path){//, Eigen::Matrix<float, 2, 1> robot_pose){
//auto optimized_variable = false;
//
////    opt_variables["advance_gaussian_cut_x"].
//    if(is_cyclic && path[0]==initial_cyclic_point)
//    {
//        if(reset_intention)
//        {
//            optimized_variable =true;
//            float A=1;
//            float B,C = 0;
//            auto delta_error = (error_data.accumulated_path_distance*A + error_data.accumulated_angular_acceleration*B + error_data.accumulated_lineal_acceleration*C)  - last_error;
//
//
//            if (delta_error < 0){
//                opt_variables[variable_vector[0]].last_value = opt_variables[variable_vector[0]].last_value+opt_variables[variable_vector[0]].last_step;
//                opt_variables[variable_vector[0]].last_value = std::clamp(opt_variables[variable_vector[0]].last_value, opt_variables[variable_vector[0]].min, opt_variables[variable_vector[0]].max);
//            }
//            else
//            {
//                opt_variables[variable_vector[0]].last_value = opt_variables[variable_vector[0]].last_value-opt_variables[variable_vector[0]].last_step;
//                opt_variables[variable_vector[0]].last_value = std::clamp(opt_variables[variable_vector[0]].last_value, opt_variables[variable_vector[0]].min, opt_variables[variable_vector[0]].max);
//                opt_variables[variable_vector[0]].last_step = -opt_variables[variable_vector[0]].last_step;
//            }
//
//            last_error = (error_data.accumulated_path_distance*A + error_data.accumulated_angular_acceleration*B + error_data.accumulated_lineal_acceleration*C);
//            error_vector.insert(error_vector.cbegin(),last_error);
//
//            dump_error();
//            reset_error(error_data);
//            reset_intention = false;
//
//            std::cout << "-----------OPTIMIZATION---------------"<<std::endl;
//            std::cout << "Variable = " << variable_vector[0] << std::endl;
//            std::cout << "Delta error = "<< delta_error <<std::endl;
//            std::cout << "Value = "<< opt_variables[variable_vector[0]].last_value << std::endl;
//            std::cout << "Last step = " << opt_variables[variable_vector[0]].last_step << std::endl;
//        }
//        if(optimized_variable){
//            variable_vector.push_back(variable_vector[0]);
//            variable_vector.erase(variable_vector.cbegin());
//            std::cout<<"NEW OPT VARIABLE -> "<<variable_vector[0]<< std::endl;
//
//            //reset error_vector
//        }
//
//
//    }
//    else if (is_cyclic && initial_cyclic_point!=path[0])
//    {
//        reset_intention=true;
//    }
//
//    return optimizated_var;
//}

bool SpecificWorker::optimizer(const std::vector<Eigen::Vector2f> &path){//, Eigen::Matrix<float, 2, 1> robot_pose){
    auto optimized_variable = false;

//    opt_variables["advance_gaussian_cut_x"].
    if(is_cyclic && path[0]==initial_cyclic_point)
    {
        if(reset_intention)
        {
            //optimized_variable =true;
            float A=1;
            float B,C = 0;
            auto delta_error = (error_data.accumulated_path_distance*A + error_data.accumulated_angular_acceleration*B + error_data.accumulated_lineal_acceleration*C)  - last_error;

            if (delta_error < 0){
                opt_variables[variable_vector[0]].last_value = opt_variables[variable_vector[0]].last_value+opt_variables[variable_vector[0]].last_step;
                opt_variables[variable_vector[0]].last_value = std::clamp(opt_variables[variable_vector[0]].last_value, opt_variables[variable_vector[0]].min, opt_variables[variable_vector[0]].max);
            }
            else
            {
                opt_variables[variable_vector[0]].last_value = opt_variables[variable_vector[0]].last_value-opt_variables[variable_vector[0]].last_step;
                opt_variables[variable_vector[0]].last_value = std::clamp(opt_variables[variable_vector[0]].last_value, opt_variables[variable_vector[0]].min, opt_variables[variable_vector[0]].max);
                opt_variables[variable_vector[0]].last_step = -opt_variables[variable_vector[0]].last_step;
            }

            last_error = (error_data.accumulated_path_distance*A + error_data.accumulated_angular_acceleration*B + error_data.accumulated_lineal_acceleration*C);
            error_vector.insert(error_vector.cbegin(),last_error);

            dump_error();
            reset_error(error_data);
            reset_intention = false;

            std::cout << "-----------OPTIMIZATION---------------"<<std::endl;
            std::cout << "Variable = " << variable_vector[0] << std::endl;
            std::cout << "Delta error = "<< delta_error <<std::endl;
            std::cout << "Value = "<< opt_variables[variable_vector[0]].last_value << std::endl;
            std::cout << "Last step = " << opt_variables[variable_vector[0]].last_step << std::endl;
        }
        if(optimized_variable){
            variable_vector.push_back(variable_vector[0]);
            variable_vector.erase(variable_vector.cbegin());
            std::cout<<"NEW OPT VARIABLE -> "<<variable_vector[0]<< std::endl;

            //reset error_vector
        }
    }
    else if (is_cyclic && initial_cyclic_point!=path[0])
    {
        reset_intention=true;
    }

    return optimizated_var;
}

//SpecificWorker::Grad_Stochastic(const Eigen::MatrixX3d &points, const std::vector<double> &params,
//                                        const  std::vector<double> &deltas, unsigned int max_iter, double mean_error_to_leave, double huber)
//{
//    static std::random_device rd;
//    static std::mt19937 mt(rd());
//    static std::uniform_int_distribution<int> params_selector(0, params.size()-1);
//    static std::uniform_int_distribution<int> delta_selector(0, deltas.size()-1);
//
//    std::vector<double> new_params = params;
//    float e_ant = std::numeric_limits<double>::max();
//    int idx = params_selector(mt);
//    int step_index = delta_selector(mt);
//    double step = deltas.at(step_index);
//    std::tuple<double, Eigen::ArrayXXd> res;
//    size_t loops = 0;
//    double gradient = 0.0;
//    std::vector<double> gradients;
//    for(auto &&i : iter::range(max_iter))
//    {
//        //qInfo() << FUNCTION << idx << new_params[0] << " " << new_params[1] << " " << new_params[2] << " " << new_params[3];
//        new_params.at(idx) += step;
//        res = error(new_params, points, huber);
//        const auto [e, _] = res;
//        if( e < mean_error_to_leave )
//            break;
//        if( e >= e_ant)  // time to change param and delta
//        {
//            idx = params_selector(mt);
//            step_index = delta_selector(mt);
//            step = deltas[step_index];
//            e_ant = std::numeric_limits<double>::max();
//        }
//        e_ant = e;
//        loops = i;
//        gradient = (e_ant - e) / step;
//        //gradients.push_back(fabs(gradient));
//    }
//    //qInfo() << FUNCTION << std::ranges::max(gradients) << std::ranges::min(gradients);
//    return std::make_tuple(new_params, std::get<0>(res), loops, std::get<1>(res));
//}

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