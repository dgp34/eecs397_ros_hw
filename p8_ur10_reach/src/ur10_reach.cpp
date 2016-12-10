// ur10_reach by dgp34
// search for reachability for flange over x_range = [-1.5,1.5] , y_range= [-1.5,1.5] at z_range = ...

#include <ur_fk_ik/ur_kin.h> 
#include <fstream>
#include <string>
using namespace std;

// define all magic numbers
#define GRND 0.0
#define BIN_TOP 0.724275
#define AGV_TOP 0.750201
#define CONV_TOP 0.903960
#define TRAY_TOP 0.950316
#define ARM_BASE 1.099893

// define XY domains and tolerances
#define X_MIN -1.5
#define X_MAX 1.5
#define Y_MIN -1.5
#define Y_MAX 1.5
#define DX 0.05
#define DY 0.05

int main(int argc, char **argv) {
    ros::init(argc, argv, "ur10_reach");

    int nsolns;

    double x_des,y_des,z_des;

    std::vector<double> z_heights;
    z_heights.push_back(GRND - ARM_BASE);
    z_heights.push_back(BIN_TOP - ARM_BASE);
    z_heights.push_back(AGV_TOP - ARM_BASE);
    z_heights.push_back(CONV_TOP - ARM_BASE);
    z_heights.push_back(TRAY_TOP - ARM_BASE);

    Eigen::Vector3d p_des;

    Eigen::Vector3d b_des, n_des,t_des;
    b_des<<0,0,-1; //tool flange pointing down
    n_des<<0,0,1; //x-axis pointing forward...arbitrary
    t_des = b_des.cross(n_des); //consistent right-hand frame
    
    Eigen::Matrix3d R_des;
    R_des.col(0) = n_des;
    R_des.col(1) = t_des;
    R_des.col(2) = b_des;

    Eigen::Affine3d A_fwd_DH;
    A_fwd_DH.linear() = R_des;

    std::vector<Eigen::Vector3d> reachable;
    std::vector<Eigen::VectorXd> q6dof_solns;

    std::string namelist[] = {"Ground", "Bin Top", "Tray Top", "Conveyor Top", "AGV"};

    UR10IkSolver ik_solver;

    for (int i = 0; i<z_heights.size();i++) {
	reachable.clear();

        for (double x_des = X_MIN; x_des < X_MAX; x_des += DX) {

            for (double y_des = Y_MIN; y_des < Y_MAX; y_des += DY) {
	        p_des[0] = x_des;
	        p_des[1] = y_des;
	        p_des[2] = z_heights[i];
	        A_fwd_DH.translation() = p_des;

	        nsolns = ik_solver.ik_solve(A_fwd_DH, q6dof_solns);
	        if (nsolns > 0) {
                    ROS_INFO("soln at x,y = %f, %f",p_des[0],p_des[1]);
                    reachable.push_back(p_des);
                }
            }
        }

        ROS_INFO("Saving these results...");

    	int temp_n = reachable.size();
        const char* name = namelist[i].c_str();
    	ofstream outfile;

    	outfile.open(name);

    	for (int j=0; j<temp_n; j++) {
            p_des = reachable[j];
            outfile << p_des[0] << ", " << p_des[1] << ", " << p_des[2] << endl;
    	}

    	outfile.close(); 

    	ROS_INFO("Saved.");
    }

    ROS_INFO("Reachability has been computed.");
    return 0;
}
