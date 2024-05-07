#include "ltl/LTLBeliefDecomposition.h"

double square(double value){
    return value*value;
}

ompl::control::BeliefDecomposition::BeliefDecomposition(int dim, const base::RealVectorBounds &b, const Scene scene, const double p_safe) : Decomposition(dim, b){

    setNumRegions(5);
	p_threshold_ = p_safe;
    dimensions_ = dim;

    n_regions_ = scene.n_regions_;
    A_list_.resize(n_regions_); A_list_ = scene.A_list_regions_;
    B_list_.resize(n_regions_); B_list_ = scene.B_list_regions_;

    erf_inv_result_ = boost::math::erf_inv(1 - 2 * p_threshold_/n_regions_);
}

ompl::control::BeliefDecomposition::BeliefDecomposition(int dim, const base::RealVectorBounds &b) : Decomposition(dim, b){

    setNumRegions(1);
}

int ompl::control::BeliefDecomposition::coordToRegion(std::vector<double> coord) const{
    int rid = 0;

    for (int i = 0; i < n_regions_; ++i){
        if (coord.size() == 3){
            if (coord[0] >= proposition_regions_[i][0] && coord[0] <= proposition_regions_[i][1] && coord[1] >= proposition_regions_[i][2] && coord[1] <= proposition_regions_[i][3] && coord[2] >= proposition_regions_[i][4] && coord[2] <= proposition_regions_[i][5])
            {
                rid = i+1;
            }
        }
        else
            if (coord[0] >= proposition_regions_[i][0] && coord[0] <= proposition_regions_[i][1] && coord[1] >= proposition_regions_[i][2] && coord[1] <= proposition_regions_[i][3])
            {
                rid = i+1;
            }
    }
    return rid;
}

double ompl::control::BeliefDecomposition::getRegionVolume(int id)
{
    return 1.0;
}

void ompl::control::BeliefDecomposition::getNeighbors(int rid, std::vector<int> &neighbors) const
{

    if (rid == 0){
        for (int i = 0; i < n_regions_; ++i)
        {
            neighbors.push_back(i+1);
        }
    }
    else {
        neighbors.push_back(0);
    }

}

int ompl::control::BeliefDecomposition::locateRegion(const ompl::base::State *s) const
{
    std::vector<double> coord(6); //x, y, z, covx, covy, covz
    project(s, coord);

    if (coord.size() == 2){
        coord[2] = 4.0;
        coord[5] = 0.000000001;
    }

    double x = coord[0];
    double y = coord[1];
    double z = coord[2];

    Eigen::MatrixXf PX(3, 3); PX.setZero();

    PX(0,0) = coord[3];
    PX(1,1) = coord[4];
    PX(2,2) = coord[5];

    //=========================================================================
    // Probabilistic region checker
    //=========================================================================

    for (int o = 0; o < n_regions_; o++) {
        if (HyperplaneCCValidityChecker(A_list_.at(o), B_list_.at(o), x, y, z, PX)) {
            return o+1;
        }
    }
    return 0;
}

bool ompl::control::BeliefDecomposition::HyperplaneCCValidityChecker(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const double &x_pose, const double &y_pose, const double &z_pose, const Eigen::MatrixXf &PX) const {
	
    bool valid = true;
	double PV, b_bar;
	Eigen::MatrixXf Pv_2;

	for (int i = 0; i < 6; i++) {
		Pv_2 = A.row(i) * PX * A.row(i).transpose();
		PV = sqrt(Pv_2(0, 0));

		b_bar = sqrt(2) * PV * erf_inv_result_;

		if(B(i, 0) - (x_pose * A(i, 0) + y_pose * A(i, 1) + z_pose * A(i, 2)) >= b_bar) {
            valid = true;
		}
        else{
            valid = false;
            return false;
        }
	}
	return valid;
}

void ompl::control::BeliefDecomposition::sampleFromRegion(int rid, ompl::RNG &rng, std::vector<double> &coord) const{

    auto bounds_ = getBounds();

    while (true){
        coord[0] = rng.uniformReal(bounds_.low[0], bounds_.high[0]);
        coord[1] = rng.uniformReal(bounds_.low[1], bounds_.high[1]);
        if (coordToRegion(coord) == rid){
            break;
        }
    }
}