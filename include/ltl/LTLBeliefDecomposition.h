/* Author: Qi Heng Ho */

#ifndef OMPL_CONTROL_PLANNERS_BeliefDecomposition_
#define OMPL_CONTROL_PLANNERS_BeliefDecomposition_

#include <cstdlib>
#include <memory>
#include <unordered_map>
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/State.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/util/RandomNumbers.h"
#include <iostream>
#include "Spaces/R3BeliefSpace.h"
#include "Spaces/R2BeliefSpace.h"
#include <boost/math/special_functions/erf.hpp>
#include "Scene/Scene.h"
#include "System/System.h"

namespace ompl
{
    namespace control
    {
        /** \brief A GridDecomposition is a Decomposition implemented using a grid. */
        class BeliefDecomposition : public Decomposition
        {
        public:
            /** \brief Constructor. Creates a GridDecomposition as a hypercube with a given dimension, side length, and
               bounds.
                The cells of the hypercube are referenced by integer coordinates of the form
                \f$(r_1,\ldots,r_k)\f$, where \f$ 0 \leq r_i < \texttt{len}\f$. */
            BeliefDecomposition(int dim, const base::RealVectorBounds &b);

            BeliefDecomposition(int dim, const base::RealVectorBounds &b, const Scene scene, const double p_threshold = 0.95);

            ~BeliefDecomposition() override = default;

            int NumRegions;

            int getNumRegions() const override
            {
                return NumRegions;
            }

            void setNumRegions(int num)
            {
                NumRegions = num;
            }

            int coordToRegion(std::vector<double> coord) const;

            double getRegionVolume(int rid) override;

            void getNeighbors(int rid, std::vector<int> &neighbors) const override;

            int locateRegion(const base::State *s) const override;

            void sampleFromRegion(int rid, RNG &rng, std::vector<double> &coord) const override;

            bool HyperplaneCCValidityChecker(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const double &x_pose, const double &y_pose, const double &z_pose, const Eigen::MatrixXf &PX) const;
            bool HyperplaneCCValidityCheckerAvoid(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const double &x_pose, const double &y_pose, const double &z_pose, const Eigen::MatrixXf &PX) const;

        protected:
            // /** \brief Helper method to return the bounds of a given region. */
            // virtual const base::RealVectorBounds &getRegionBounds(int rid) const;
            int discretization_level_{1};
            double p_threshold_, erf_inv_result_, erf_inv_result_region_oneminusalpha_;

            std::vector<std::vector<double>> bounds_;

            unsigned int n_regions_;
            std::vector<Eigen::Matrix<float, 6, 3> > A_list_;
            std::vector<Eigen::Matrix<float, 6, 1> > B_list_;
            int dimensions_;

            std::vector<std::vector<double>> proposition_regions_;
        };
    }
}
#endif
