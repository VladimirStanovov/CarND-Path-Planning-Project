#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

vector<int> getStates(int currentState,bool lane_changed_recently)
{
    vector<int> successor_states;
    if(currentState == 0)
    {
        successor_states.push_back(0);  //KL
        successor_states.push_back(1);  //PLCL
        successor_states.push_back(2);  //PLCR
    }
    else if(currentState == 1)
    {
        successor_states.push_back(0);  //KL
        successor_states.push_back(1);  //PLCL
        successor_states.push_back(2);  //PLCR
        if(!lane_changed_recently)
            successor_states.push_back(3);  //LCL
    }
    else if(currentState == 2)
    {
        successor_states.push_back(0);  //KL
        successor_states.push_back(2);  //PLCR
        successor_states.push_back(1);  //PLCL
        if(!lane_changed_recently)
            successor_states.push_back(4);  //LCR
    }
    else if(currentState == 3)
    {
        successor_states.push_back(0);  //KL
    }
    else if(currentState == 4)
    {
        successor_states.push_back(0);  //KL
    }
    return successor_states;
}

struct Trajectory
{
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    double ref_vel;
    double project_speed;
    int lane;
};
double LANE_CHANGE_COST = 0.5;
double OFF_TRACK_COST = 100;
double PREPARE_COST = 1.0;
double COLLISION_SAFE_DISTANCE = 10;
vector<string> STATE_NAMES = {"KL","PLCL","PLCR","LCL","LCR"};
vector<vector<double> > getTrajectoryForState(json j, int newstate, int &lane, double &ref_vel, double &project_speed, double max_speed,
                                                vector<double> map_waypoints_x,  vector<double> map_waypoints_y,  vector<double> map_waypoints_s,
                                                vector<double> map_waypoints_dx,  vector<double> map_waypoints_dy)
{
    double car_x = j[1]["x"];
    double car_y = j[1]["y"];
    double car_s = j[1]["s"];
    double car_d = j[1]["d"];
    double car_yaw = j[1]["yaw"];
    double car_speed = j[1]["speed"];

    // Previous path data given to the Planner
    auto previous_path_x = j[1]["previous_path_x"];
    auto previous_path_y = j[1]["previous_path_y"];
    // Previous path's end s and d values
    double end_path_s = j[1]["end_path_s"];
    double end_path_d = j[1]["end_path_d"];

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    auto sensor_fusion = j[1]["sensor_fusion"];

    int prev_size = previous_path_x.size();
    int prev_lane = lane;
    double desired_speed = max_speed;
    project_speed = 0;
    double future_car_s = car_s;
    if(newstate == 0 || newstate == 1 || newstate == 2)   //KL/PLCL/PLCR
    {
        //cout<<"KL|PLCL|PLCR\n";
        if(prev_size > 0)
        {
            future_car_s = end_path_s;
        }
        double min_s_front;
        bool front_exists = false;
        project_speed = max_speed;
        for(int i=0;i!=sensor_fusion.size();i++)
        {
            float d = sensor_fusion[i][6];
            if(d < (2 + 4*lane + 2) && d > (2 + 4*lane-2))
            {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];
                check_car_s += ((double)prev_size*0.02*check_speed);
                double dist_check = check_car_s - future_car_s;
                if(dist_check > 0 && !front_exists)
                {
                    front_exists = true;
                    min_s_front = dist_check;
                    //cout<<min_s_front<<"\t";
                }
                if(dist_check > 0 && front_exists)
                {
                    if(dist_check <= min_s_front)
                    {
                        min_s_front = dist_check;
                        //cout<<min_s_front<<"\n";
                        double dist_coeff = (1./(1.+exp((min_s_front-16)*0.4)));
                        //cout<<dist_coeff<<endl;
                        desired_speed = (1.-dist_coeff)*max_speed + dist_coeff*check_speed;
                        project_speed = desired_speed;
                    }
                }
            }
        }
    }
    if((newstate == 1 && lane > 0) || (newstate == 2 && lane < 2))   //PLCL/PLCR
    {
        int temp_lane;
        if(newstate == 1)
            temp_lane = lane - 1;
        else
            temp_lane = lane + 1;
        if(prev_size > 0)
        {
            future_car_s = end_path_s;
        }
        double min_s_front;
        bool front_exists = false;
        project_speed = max_speed;
        for(int i=0;i!=sensor_fusion.size();i++)
        {
            float d = sensor_fusion[i][6];
            if(d < (2 + 4*temp_lane + 2) && d > (2 + 4*temp_lane-2))
            {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];
                check_car_s += ((double)prev_size*0.02*check_speed);
                double dist_check = check_car_s - future_car_s;
                if(dist_check > 0 && !front_exists)
                {
                    front_exists = true;
                    min_s_front = dist_check;
                    //cout<<min_s_front<<"\t";
                }
                if(dist_check > 0 && front_exists)
                {
                    if(dist_check <= min_s_front)
                    {
                        min_s_front = dist_check;
                        //cout<<min_s_front<<"\n";
                        double dist_coeff = (1./(1.+exp((min_s_front-16)*0.4)));
                        //cout<<dist_coeff<<endl;
                        project_speed = (1.-dist_coeff)*max_speed + dist_coeff*check_speed;
                    }
                }
            }
        }
    }
    if((newstate == 3 && lane > 0) || (newstate == 4 && lane < 2))  //LCL/LCR
    {
        if(newstate == 3)
            lane--;
        else
            lane++;
        if(prev_size > 0)
        {
            future_car_s = end_path_s;
        }
        double min_s_front;
        bool front_exists = false;
        project_speed = max_speed;
        bool unsafe = false;
        for(int i=0;i!=sensor_fusion.size();i++)
        {
            float d = sensor_fusion[i][6];
            if(d < (2 + 4*lane + 2) && d > (2 + 4*lane-2))
            {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];
                check_car_s += ((double)prev_size*0.02*check_speed);
                double dist_check = check_car_s - future_car_s;
                if(dist_check > 0 && !front_exists)
                {
                    front_exists = true;
                    min_s_front = dist_check;
                    //cout<<min_s_front<<"\t";
                }
                if(dist_check > 0 && front_exists)
                {
                    if(dist_check <= min_s_front)
                    {
                        min_s_front = dist_check;
                        //cout<<min_s_front<<"\n";
                        double dist_coeff = (1./(1.+exp((min_s_front-16)*0.4)));
                        //cout<<dist_coeff<<endl;
                        desired_speed = (1.-dist_coeff)*max_speed + dist_coeff*check_speed;
                        project_speed = desired_speed;
                    }
                }
                if(dist_check < COLLISION_SAFE_DISTANCE && dist_check > -COLLISION_SAFE_DISTANCE)
                {
                    unsafe = true; //We will collide! (probably...)
                }
            }
        }
        if(unsafe)
            project_speed = 0;
    }

    vector<double> ptsx;
    vector<double> ptsy;

    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    if(prev_size < 2)
    {
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    }
    else
    {
        ref_x = previous_path_x[prev_size-1];
        ref_y = previous_path_y[prev_size-1];

        double ref_x_prev = previous_path_x[prev_size-2];
        double ref_y_prev = previous_path_y[prev_size-2];

        ref_yaw = atan2(ref_y - ref_y_prev,ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    vector<double> next_wp0;
    vector<double> next_wp1;
    vector<double> next_wp2;

    if(prev_lane != lane)
    {
        next_wp0 = getXY(car_s + 50,(2+4*prev_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
        next_wp1 = getXY(car_s + 100,(2+2*(prev_lane+lane)),map_waypoints_s,map_waypoints_x,map_waypoints_y);
        next_wp2 = getXY(car_s + 150,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
    }
    else
    {
        next_wp0 = getXY(car_s + 50,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
        next_wp1 = getXY(car_s + 100,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
        next_wp2 = getXY(car_s + 150,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
    }

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for(int i=0; i < ptsx.size(); i++)
    {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
        ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
    }

    tk::spline s;

    s.set_points(ptsx,ptsy);

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    for(int i=0;i!=previous_path_x.size();i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    double target_x = 60.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));

    double x_add_on = 0;

    for(int i=1;i <= 50 - previous_path_x.size(); i++)
    {
        double vel_difference = desired_speed - ref_vel;
        if(vel_difference > 0.224)
            vel_difference = 0.224;
        if(vel_difference < -0.224)
            vel_difference = -0.224;
        ref_vel += vel_difference;
        /*if(vel_difference <= 0)
        {
            ref_vel -= .224;
        }
        else if (ref_vel < desired_speed)
        {
            ref_vel += .224;
        }*/
        //cout<<"generating point "<<i<<endl;
        double N = (target_dist/(0.02*ref_vel/2.24));
        double x_point = x_add_on + (target_x)/N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
    return {next_x_vals,next_y_vals};
}
double calculate_cost(int lane,double ref_vel,double max_speed,Trajectory newTraj,int state)
{
    double cost = 0;
    if(lane != newTraj.lane)
    {
        cost = cost + LANE_CHANGE_COST;
        //cout<<"LANE_CHANGE:"<<lane<<" new: "<<newTraj.lane<<endl;
    }
    if(lane == 0 && state == 3)
    {
        cost += OFF_TRACK_COST;
        //cout<<"OFF_TRACK_COST"<<endl;
    }
    if(lane == 2 && state == 4)
    {
        cost += OFF_TRACK_COST;
        //cout<<"OFF_TRACK_COST"<<endl;
    }
    if(state == 1 || state == 2)
    {
        cost += max_speed+0.5+PREPARE_COST - newTraj.project_speed;
        //cout<<"project_speed = "<<newTraj.project_speed<<"\tcost = "<<cost<<endl;
    }
    else
    {
        cost += max_speed+0.5 - newTraj.project_speed;
        //cout<<"project_speed = "<<newTraj.project_speed<<"\tcost = "<<cost<<endl;
    }

    return cost;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  int lane = 1;
  double ref_vel = 0.0;
  double max_speed = 49.5;
  int state = 0; //KL
  double last_lane_change_s = 0;
  double lane_change_period = 100;

  h.onMessage([&lane,&ref_vel,&max_speed,&state,&last_lane_change_s,&lane_change_period,&max_s,
                &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,
                &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
            // j[1] is the data JSON object

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
          	double car_s = j[1]["s"];
          	if(car_s < last_lane_change_s)
                last_lane_change_s = last_lane_change_s - max_s;
            auto next_states = getStates(state,(car_s-last_lane_change_s) < lane_change_period);
            cout<<"car_s = "<<car_s<<endl;
            cout<<"Current state: "<<STATE_NAMES[state]<<endl;
            double newcost;
            double best_cost;
            double best_state = -1;
            int best_state_num;
            vector<Trajectory> newTraj(next_states.size());
            for(int i=0;i!=next_states.size();i++)
            {
                newTraj[i].lane = lane;
                newTraj[i].ref_vel = ref_vel;
                auto new_trajectory = getTrajectoryForState(j,next_states[i],newTraj[i].lane,newTraj[i].ref_vel,newTraj[i].project_speed,max_speed,map_waypoints_x,map_waypoints_y,map_waypoints_s,map_waypoints_dx,map_waypoints_dy);
                newTraj[i].next_x_vals = new_trajectory[0];
                newTraj[i].next_y_vals = new_trajectory[1];
                newcost = calculate_cost(lane,ref_vel,max_speed,newTraj[i],next_states[i]);
                if(best_state == -1)
                {
                    best_cost = newcost;
                    best_state = next_states[i];
                    best_state_num = i;
                }
                else if (newcost < best_cost)
                {
                    best_cost = newcost;
                    best_state = next_states[i];
                    best_state_num = i;
                }
                cout<<STATE_NAMES[next_states[i]]<<"\t cost = "<<newcost<<"\n";
            }
            //cout<<best_state<<endl;
            next_x_vals = newTraj[best_state_num].next_x_vals;
            next_y_vals = newTraj[best_state_num].next_y_vals;
            ref_vel = newTraj[best_state_num].ref_vel;
            if(lane != newTraj[best_state_num].lane)
            {
                last_lane_change_s = car_s;
            }
            //cout<<"last_lane_change_s = "<<last_lane_change_s<<endl;

            lane = newTraj[best_state_num].lane;
            if(state != best_state)
            {
                cout<<"STATE CHANGED TO "<<STATE_NAMES[best_state];
            }
            state = best_state;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

          	//cout<<"Sending data"<<endl;

            json msgJson;

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
