#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include "spline.h"
// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace std;
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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

////////////////////////Dynamic optimal trej ///////////////////////////////////////////////////////////////////////////////////////
          
	  const int d=3, S=100;
 	  double J[S][d],speed_cost[S][d];

      double next_opt_s[S][d];
 	  double next_opt_d[S][d];
         
      vector <double> rel_dist_s;
      vector <double> d_c;
	  vector <double> speed_o;
	  double speed;

           if(sensor_fusion.size() > 0) {
            for(int i=0; i<sensor_fusion.size(); i++){
              double vxo= sensor_fusion[i][3];
              double vyo= sensor_fusion[i][4];
              double s_o= sensor_fusion[i][5];
              double d_o= sensor_fusion[i][6];
	      if ((s_o > end_path_s) && (floor(s_o-end_path_s) < S) && (floor(d_o)>=0)){
	    	rel_dist_s.push_back(s_o);
            	d_c.push_back(d_o);
		speed_o.push_back(sqrt(vxo*vxo + vyo*vyo));
	      }
            }
           }
         	 
     	 for(int i=0; i<S;i++){
    	   for(int j=0; j<d ; j++){
               J[i][j] = 0;
	       speed_cost[i][j]=0;
           }
         }

//////////////////////////////Grid///////////////////////////////////////////////////////////////////
 	for(int i=0;i<d_c.size();i++){
		
		int tmp = floor(rel_dist_s[i] - end_path_s);
	        
		if (tmp>=0) {		
   			int temp_d = floor(d_c[i]/4);
    			J[S-1-tmp][temp_d] = 99;
			J[S-1-tmp+1][temp_d] = (fabs(22.35 - speed_o[i]));
		}
 	}

	vector<double> add_cost{0,0,0};

	 for(int i=0; i<S;i++){
   		 for(int j=0; j<d ; j++){
        		double temp11 = add_cost[j] ;
        		if(J[i][j]==0){
                		J[i][j] = J[i][j]+temp11;	;
        		}else if (J[i][j]==99){
            			add_cost[j] = J[i+1][j] ;
        		}
    		}
 	}

	double J_opt[d];
	double turn;

	for (int i=0;i<d;i++){
    		J_opt[i] = J[0][i] ;
	}

	for(int s = 1 ; s < S; s++){

		double cost_act[d][3];
    		int    nxt_s_act[d][3];
    		int    nxt_d_act[d][3];

  		for(int i=0; i<d ; i++){
        		for (int j=0; j<d; j++){

          			int nxt_s_tmp = (s-1);
              			    nxt_s_tmp = max(0, nxt_s_tmp);
          			int nxt_d_tmp = j + i - 1 ;

          			if ((nxt_d_tmp >=0) and (nxt_d_tmp < d)){

              				cost_act[i][j]  = J[nxt_s_tmp][nxt_d_tmp];   ///cost of action + cost of lane
                      
          			}else if((nxt_d_tmp>d-1) or (nxt_d_tmp<0)){
             				cost_act[i][j]  = 99;
          			}         			
				nxt_s_act[i][j] = nxt_s_tmp;
          			nxt_d_act[i][j] = nxt_d_tmp;
          		}
    		}

		

    		for (int i=0; i<d;i++){

        		double J_opt_temp =99;
        		double temp1 =99999;
        		double next_s_temp = -1;
        		double next_d_temp = -1;

        		for (int j=0; j<d ;j++) {

          			J_opt_temp = J_opt[i] + cost_act[i][j];

          			if (J_opt_temp < temp1){
              				next_d_temp = nxt_d_act[i][j] ;
              				next_s_temp = nxt_s_act[i][j] ; 
              				temp1 = J_opt_temp;
           			}
        		}
        
			J_opt[i]= temp1;
        		next_opt_s[s][i] = next_s_temp ;
       			next_opt_d[s][i] = next_d_temp ;
		}   	
	}
	
/////////////////////////////////////////////
	int start_d= floor(end_path_d/4) ;
	
	double trej[S][d];
	for(int i=0; i<S;i++){
    		for(int j=0; j<d ; j++){
			trej[i][j]=0;
		}
    	}
	
	for(int i=S-1;i>=0;i--){
	
		////////////////////////////////////////////////
		
	//	if( (start_d==0 and next_opt_d[i][1]==2) or (start_d==2 and next_opt_d[i][1] ==0)){
	//		start_d = 1;
	//	}

		///////////////////////////////////////////////
    	
		trej[i][start_d] = 1;
    		start_d = next_opt_d[i][start_d];
	}

/////////////////////////////////////////////
    vector<int> trej_D;
	bool flag=false; 
	
        for(int j=0;j<d;j++){
		int temp = trej[S-2][j];	  
		if((temp==1)){
	      		turn = j;
	      		flag = true;
			break;
           	}
	  }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
           
	   int path_size = previous_path_x.size();          
	   
	   double inc ;  
        
	   double front_S = S;
           double lane = car_d;
	   double ref_vel=car_speed;
	   double speed_diff =0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	   
	   
	   if(path_size>3){

		bool inc_flag=true;
                double dy11 = (previous_path_y[path_size-1]);
                double dy21 = (previous_path_y[path_size-2]);
                double dx11 = (previous_path_x[path_size-1]);
                double dx21 = (previous_path_x[path_size-2]);
                double theta11 = atan2(dy11-dy21,dx11-dx21);

                vector<double> sd_3 = getFrenet(previous_path_x[path_size-1], previous_path_y[path_size-1], theta11, map_waypoints_x, map_waypoints_y);
                
		if(car_speed>30){	
			lane = 2 + 4*turn;
		}
	 
          	if(sensor_fusion.size() > 0) {
	   		for(int i=0; i<sensor_fusion.size(); i++){
	    
            			double s_o= sensor_fusion[i][5];
            			double d_o= sensor_fusion[i][6];
          			double vxo= sensor_fusion[i][3];
              			double vyo= sensor_fusion[i][4];	
				double xo = sensor_fusion[i][1];
			        double yo = sensor_fusion[i][2];

	    		        //////Brake//////
				if( (floor(car_d/4)==floor(d_o/4)) and (s_o - sd_3[0] <=100 ) and ( s_o >= sd_3[0]) ) {
					front_S = s_o;
					if( (s_o - sd_3[0] )<20){
						speed = sqrt(vxo*vxo + vyo*vyo);
                        			inc_flag=false;
						break;
			                }
	   			}	
			}
		} 
	   
////////////////////////////////////Lane change decision///////////////////////////////////

		if(sensor_fusion.size() > 0) {
                        for(int i=0; i<sensor_fusion.size(); i++){

                                double s_o= sensor_fusion[i][5];
                                double d_o= sensor_fusion[i][6];
                                double vxo= sensor_fusion[i][3];
                                double vyo= sensor_fusion[i][4];
                                
                                if (((floor(lane/4) == floor(d_o/4)) and (  ( (s_o>car_s) and  (s_o - car_s ) < 20) or ( (car_s>s_o) and (car_s - s_o ) < 10 )))){

				 	lane = 2+4*floor(car_d/4);
					break;
				}
			}
		}			
	
		double final_lane = lane ;
		if(!inc_flag and (ref_vel*0.447 > speed)){
	
			speed_diff = 0 - 3*0.224; //Brakes

		}else if(inc_flag and ref_vel<49.5){
			speed_diff = 0.224;		
		}
	   }
//////////////////////////////////////////////////////////////////////////////////////////////

	    vector<double> X;
            vector<double> Y;

            double x0 = car_x;
            double y0 = car_y;
            double yaw0 = deg2rad(car_yaw);

            if ( path_size < 2 ) {
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                X.push_back(prev_car_x);
                X.push_back(car_x);

                Y.push_back(prev_car_y);
                Y.push_back(car_y);

            } else {
                x0 = previous_path_x[path_size - 1];
                y0 = previous_path_y[path_size - 1];

                double x0_prev = previous_path_x[path_size - 2];
                double y0_prev = previous_path_y[path_size - 2];
                yaw0 = atan2(y0-y0_prev, x0-x0_prev);

                X.push_back(x0_prev);
                X.push_back(x0);

                Y.push_back(y0_prev);
                Y.push_back(y0);
            }

            vector<double> p0 = getXY(car_s + 40, lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> p1 = getXY(car_s + 70, lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> p2 = getXY(car_s + 90, lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            X.push_back(p0[0]);
            X.push_back(p1[0]);
            X.push_back(p2[0]);

            Y.push_back(p0[1]);
            Y.push_back(p1[1]);
            Y.push_back(p2[1]);

            for ( int i = 0; i < X.size(); i++ ) {
              double shift_x = X[i] - x0;
              double shift_y = Y[i] - y0;

              X[i] = shift_x * cos(0 - yaw0) - shift_y * sin(0 - yaw0);
              Y[i] = shift_x * sin(0 - yaw0) + shift_y * cos(0 - yaw0);
            }

            tk::spline s;
            s.set_points(X,Y);

            for ( int i = 0; i < path_size; i++ ) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(900 + target_y*target_y);

            double x_inc = 0;

            for( int i = 1; i < 25 - path_size; i++ ) {
                
				ref_vel += speed_diff;
                
				ref_vel = min(49.5, max(2.24,ref_vel)) ;

              double N = target_dist/(0.02*ref_vel/2.24);
              double x = x_inc + target_x/N;
              double y = s(x);

              x_inc = x;

              double x_temp = x;
              double y_temp = y;

              x = x_temp * cos(yaw0) - y_temp * sin(yaw0);
              y = x_temp * sin(yaw0) + y_temp * cos(yaw0);

              x += x0;
              y += y0;
 
              next_x_vals.push_back(x);
              next_y_vals.push_back(y);
            }

 ///////////////////////////////////////////////////////////////////////////////////////////////////	  

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
