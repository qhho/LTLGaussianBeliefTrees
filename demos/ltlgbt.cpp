#include <filesystem>
#include "ltlgbt.hpp"

namespace ob = ompl::base;
namespace oc = ompl::control;

using Polygon = oc::PropositionalTriangularDecomposition::Polygon;
using Vertex = oc::PropositionalTriangularDecomposition::Vertex;

class TheDecomposition : public oc::BeliefDecomposition
{
public:

    TheDecomposition(int dim, const ob::RealVectorBounds &b, int discretization_level, double p_threshold) : oc::BeliefDecomposition(dim,b, discretization_level, p_threshold)
    {

    }
    
    virtual void project(const ob::State *s, std::vector<double> &coord) const
    {
        coord.resize(4);
        coord[0] = s->as<R2BeliefSpace::StateType>()->getX();
        coord[1] = s->as<R2BeliefSpace::StateType>()->getY();
        coord[2] = s->as<R2BeliefSpace::StateType>()->getCovariance()(0,0);
        coord[3] = s->as<R2BeliefSpace::StateType>()->getCovariance()(1,1);
    }
    
    void sampleFullState(const ob::StateSamplerPtr& sampler, const std::vector<double>& coord, ob::State* s) const override
    {
       sampler->sampleUniform(s);
       auto* ws = s->as<ob::CompoundStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0);
       ws->setXY(coord[0], coord[1]);
    }

    int dim_;
};

class TheDecompositionUnicycle : public oc::BeliefDecomposition
{
public:

    TheDecompositionUnicycle(int dim, const ob::RealVectorBounds &b, int discretization_level, double p_threshold) : oc::BeliefDecomposition(dim,b, discretization_level, p_threshold)
    {
        dim_ = dim;
    }
    
    virtual void project(const ob::State *s, std::vector<double> &coord) const
    {
        coord.resize(4);
        coord[0] = s->as<ob::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getX();
        coord[1] = s->as<ob::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getY();
        coord[2] = s->as<ob::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getCovariance()(0,0);
        coord[3] = s->as<ob::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getCovariance()(1,1);
    }
    
    void sampleFullState(const ob::StateSamplerPtr& sampler, const std::vector<double>& coord, ob::State* s) const override
    {
       sampler->sampleUniform(s);
       auto* ws = s->as<ob::CompoundStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0);
       ws->setXY(coord[0], coord[1]);
    }

    int dim_;
};

class MyPropDecomposition : public oc::PropositionalDecomposition
{
public:

    MyPropDecomposition(const oc::DecompositionPtr &decomp, int num_regions) : oc::PropositionalDecomposition(decomp)
    {
        num_regions_ = num_regions;
    }
    
    virtual oc::World worldAtRegion(int rid)
    {
        oc::World world(num_regions_);
        world[0] = false;
        for (int i = 0; i < num_regions_; ++i)
        {
            world[i+1] = false;
        }
        world[rid] = true;
        return world;
    }
    
    virtual int getNumProps() const
    {
        return num_regions_;
    }

    int num_regions_;
};

ob::StateSpacePtr constructStateSpace(double cov)
{
    ob::StateSpacePtr state_space = ob::StateSpacePtr(new R2BeliefSpace(cov));
    return state_space;
}

ob::StateSpacePtr constructStateSpaceUnicycle(double cov)
{
    ob::StateSpacePtr c_space = ob::StateSpacePtr(new ob::CompoundStateSpace());
    c_space->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new R2BeliefSpace(cov)), 1.0); // x, y, P
    c_space->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 0.0); // yaw
    c_space->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), 0.0); // surge
    c_space->as<ob::CompoundStateSpace>()->lock();
    return c_space;
}


oc::ControlSpacePtr constructCtrlSpaceUnicycle(ob::StateSpacePtr c_space) {
	oc::ControlSpacePtr ctrl_space = oc::ControlSpacePtr(new oc::CompoundControlSpace(c_space));
	ctrl_space->as<oc::CompoundControlSpace>()->addSubspace(oc::ControlSpacePtr(new oc::RealVectorControlSpace(c_space, 2))); // x y
	ctrl_space->as<oc::CompoundControlSpace>()->addSubspace(oc::ControlSpacePtr(new oc::RealVectorControlSpace(c_space, 1))); // yaw
	ctrl_space->as<oc::CompoundControlSpace>()->addSubspace(oc::ControlSpacePtr(new oc::RealVectorControlSpace(c_space, 1))); // surge
	ctrl_space->as<oc::CompoundControlSpace>()->lock();
	return ctrl_space;
}

void boundCtrlSpaceUnicycle(oc::ControlSpacePtr ctrl_space) {
	ob::RealVectorBounds bounds_xy(2), bounds_yaw(1), bounds_surge(1), bounds_dot_x_dot_y(2), bounds_covariance(4), bounds_cd(1);

	bounds_xy.setLow(0, 0);
	bounds_xy.setHigh(0, 100);
	bounds_xy.setLow(1, -60);
	bounds_xy.setHigh(1, 100);
	ctrl_space->as<oc::CompoundControlSpace>()->as<oc::RealVectorControlSpace>(0)->setBounds(bounds_xy);

	bounds_yaw.setLow(-M_PI);
	bounds_yaw.setHigh(M_PI);
	ctrl_space->as<oc::CompoundControlSpace>()->as<oc::RealVectorControlSpace>(1)->setBounds(bounds_yaw);

	bounds_surge.setLow(0.1);
	bounds_surge.setHigh(5.0);
	ctrl_space->as<oc::CompoundControlSpace>()->as<oc::RealVectorControlSpace>(2)->setBounds(bounds_surge);
}

oc::SpaceInformationPtr scoutPlanner()
{
    ob::StateSpacePtr space(constructStateSpace(5.0)); //space should probably be a class attribute
    // set the bounds for the R^2 part of R2BeliefSpace();
    ob::RealVectorBounds bounds_se2(2);
    bounds_se2.setLow(0, 0.0);
    bounds_se2.setHigh(0, 100.0);
    bounds_se2.setLow(1, -60.0);
    bounds_se2.setHigh(1, 100.0);
    
    space->as<R2BeliefSpace>()->setBounds(bounds_se2);

    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-60.0);
    cbounds.setHigh(100.0);

    cspace->setBounds(cbounds);

    oc::SpaceInformationPtr si(std::make_shared<oc::SpaceInformation>(space, cspace));

    ob::StateValidityCheckerPtr om_stat_val_check;
    om_stat_val_check = ob::StateValidityCheckerPtr(new StateValidityCheckerPCCBlackmore("scene3", si, 0.95));
    si->setStateValidityChecker(om_stat_val_check);
    si->setStatePropagator(oc::StatePropagatorPtr(new SimpleStatePropagator(si)));
    si->setPropagationStepSize(0.1);
    si->setMinMaxControlDuration(1, 40);
    return si;
}

void LTLGaussianBeliefTrees::plan(bool use_scout, double scout_process_noise, std::string outputfile)
{
    ompl::RNG rng = ompl::RNG();
    std::string filename = "results/" + outputfile;
    std::fstream file1(filename, std::fstream::out | std::fstream::app);
    ompl::time::point timeStart = ompl::time::now();
    double scout_initial_cov = initial_cov;
    if (scout_process_noise == 0.0){
        scout_initial_cov = 0.001;
    }
    
    ob::StateSpacePtr space(constructStateSpace(scout_initial_cov)); //space should probably be a class attribute
    // set the bounds for the R^2 part of R2BeliefSpace();
    ob::RealVectorBounds bounds_se2(2);
    bounds_se2.setLow(0, 0.0);
    bounds_se2.setHigh(0, 100.0);
    bounds_se2.setLow(1, -60.0);
    bounds_se2.setHigh(1, 100.0);
    
    space->as<R2BeliefSpace>()->setBounds(bounds_se2);
    // create a control space
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(0, 0.0);
    cbounds.setHigh(0, 100.0);
    cbounds.setLow(1, -60.0);
    cbounds.setHigh(1, 100.0);

    cspace->setBounds(cbounds);


    // double process_noise = 0.1; //0.05*rng.uniform01();

    oc::SpaceInformationPtr si(std::make_shared<oc::SpaceInformation>(space, cspace));
    ob::StateValidityCheckerPtr om_stat_val_check;
    om_stat_val_check = ob::StateValidityCheckerPtr(new StateValidityCheckerPCCBlackmoreSimple("scene3", si, 0.95));
    si->setStateValidityChecker(om_stat_val_check);
    si->setPropagationStepSize(0.25);
    si->setStatePropagator(oc::StatePropagatorPtr(new SimpleStatePropagator(si, scout_process_noise)));
    
    si->setMinMaxControlDuration(1, 20);
    si->setup();
    ob::StateSpacePtr space_unicycle(constructStateSpaceUnicycle(initial_cov)); //space should probably be a class attribute
    space_unicycle->as<ob::CompoundStateSpace>()->as<R2BeliefSpace>(0)->setBounds(bounds_se2);
    ob::RealVectorBounds bounds_surge(1);
    bounds_surge.setLow(0.1);
    bounds_surge.setHigh(5.);
    space_unicycle->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(2)->setBounds(bounds_surge);
    oc::ControlSpacePtr ctrl_space_unicycle(constructCtrlSpaceUnicycle(space_unicycle));
    boundCtrlSpaceUnicycle(ctrl_space_unicycle);
    oc::SpaceInformationPtr unicycle_si(new oc::SpaceInformation(space_unicycle, ctrl_space_unicycle));
    unicycle_si->setPropagationStepSize(0.25);
    unicycle_si->setStatePropagator(oc::StatePropagatorPtr(new DynUnicycleControlSpace(unicycle_si, process_noise)));
    ob::StateValidityCheckerPtr val_check_unicycle;
    val_check_unicycle = ob::StateValidityCheckerPtr(new StateValidityCheckerPCCBlackmore("scene3", unicycle_si, 0.95));
    unicycle_si->setStateValidityChecker(val_check_unicycle);
    unicycle_si->setMinMaxControlDuration(1, 45);
    
    unicycle_si->setup();

    ob::RealVectorBounds decomp_bounds(2);
    decomp_bounds.setLow(0, 0.0);
    decomp_bounds.setLow(1, -60.0);
    decomp_bounds.setHigh(0, 100.0);
    decomp_bounds.setHigh(1, 100.0);

    auto decomp = TheDecomposition(2, decomp_bounds, 2, 0.99); //set decomposition
    oc::DecompositionPtr dec_p(&decomp);
    auto prop_dec = MyPropDecomposition(dec_p);
    oc::PropositionalDecompositionPtr ptd(&prop_dec);

    auto decomp_unicycle = TheDecompositionUnicycle(2, decomp_bounds, 2, 0.99); //set decomposition
    oc::DecompositionPtr dec_p_unicycle(&decomp_unicycle); 
    auto prop_dec_unicycle = MyPropDecomposition(dec_p_unicycle);
    oc::PropositionalDecompositionPtr ptd_unicycle(&prop_dec_unicycle);

    // auto cosafety = oc::Automaton::SequenceAutomaton(3, {1});
    auto cosafety = oc::Automaton::MyAutomatonPhi2();
    auto safety = oc::Automaton::AcceptingAutomaton(3);
    // auto safety = oc::Automaton::AvoidanceAutomaton(3, {2});
    cosafety->print(std::cout);
    
    auto product(std::make_shared<oc::ProductGraph>(ptd, cosafety, safety));
    auto product_unicycle(std::make_shared<oc::ProductGraph>(ptd_unicycle, cosafety, safety));

    // oc::SpaceInformationPtr scoutsi = scoutPlanner();
    auto scoutstlsi(std::make_shared<oc::STLSpaceInformation>(si, product));
    auto scoutpdef(std::make_shared<oc::STLProblemDefinition>(scoutstlsi));

    // The robot state space, given by SpaceInformation, is referred to as the "lower space".
    auto stlsi(std::make_shared<oc::STLSpaceInformation>(unicycle_si, product_unicycle));
    auto pdef(std::make_shared<oc::STLProblemDefinition>(stlsi));
   
    // create a start state
    ob::ScopedState<> scoutstart(space);
    scoutstart[0] = 10.0; //x
    scoutstart[1] = 0.0; //y
    
    ob::ScopedState<> start(space_unicycle);
    start[0] = 10.0; //x
    start[1] = 0.0; //y
    start[2] = 0.0; //double(0.850977); //
    start[3] = 1.0;

    scoutpdef->addLowerStartState(scoutstart.get());
    pdef->addLowerStartState(start.get());
    
    //STL planner (input: STL space information, product automaton)
    oc::CosafeSTLRRTUnicycle stlPlanner(stlsi, scoutstlsi, product_unicycle, product);
    stlPlanner.setProblemDefinition(pdef);
    stlPlanner.scoutplanner.setProblemDefinition(scoutpdef);
    stlPlanner.setUseScout(use_scout);
    ob::PlannerStatus solved = stlPlanner.ob::Planner::solve(120.0);
    if (solved)
    {
        double timeUsed = ompl::time::seconds(ompl::time::now() - timeStart);
        file1 << timeUsed << "  1" << std::endl;
        std::cout << "Found solution" << std::endl;
        // The path returned by STLProblemDefinition is through hybrid space.
        // getLowerSolutionPath() projects it down into the original robot state space
        // that we handed to STLSpaceInformation.
        static_cast<oc::PathControl &>(*pdef->getLowerSolutionPath()).printAsMatrix(std::cout);
    }
    else
    {
        std::cout << "no solution" << std::endl;
        double timeUsed = ompl::time::seconds(ompl::time::now() - timeStart);
        file1 << timeUsed << "  0" << std::endl;
    }
}

int main(int argc, char ** argv)
{
    // bool a = false;
    // if (std::stoi(argv[2]) == 1){
    //     a = true;
    // }

    double initial_cov
    
    bool use_scout = true;
    double scout_noise = 0.0001;

    std::filesystem::path currentPath = std::filesystem::current_path();
    std::filesystem::path scene_config = currentPath  / ".." / "configurations" / "scenes" / "scene5.yaml";
    std::filesystem::path system_config = currentPath  / ".." / "configurations" / "systems" / "system1.yaml";
    GaussianBeliefTrees GaussianBeliefTrees(scene_config, system_config);

    plan(use_scout, scout_noise, "ltlgbt.txt");

    return 0;
}