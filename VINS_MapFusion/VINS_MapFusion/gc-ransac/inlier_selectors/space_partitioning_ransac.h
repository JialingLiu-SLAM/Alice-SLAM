// Copyright (C) 2021 ETH Zurich.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of 2021 ETH Zurich nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Daniel Barath (barath.daniel@sztaki.mta.hu)
#pragma once 

#include <vector>
#include <opencv2/core.hpp>
#include "model.h"
#include "inlier_selector.h"
#include "grid_neighborhood_graph.h"

namespace gcransac
{
	namespace inlier_selector
	{
        template <
            typename _Estimator,
            typename _NeighborhoodStructure>
        class SpacePartitioningRANSAC : public AbstractInlierSelector<_Estimator, _NeighborhoodStructure>
        {
        protected:
            std::vector<bool> gridCornerMask;
            std::vector<std::tuple<int, int, double, double>> gridCornerCoordinatesH;
            std::vector<double> additionalParameters;

        public:
            static constexpr bool doesSomething() { return true; }

            explicit SpacePartitioningRANSAC(const _NeighborhoodStructure *kNeighborhoodGraph_) : 
                AbstractInlierSelector<_Estimator, _NeighborhoodStructure>(kNeighborhoodGraph_)
            {
                // The number cells filled in the grid
                const size_t &kCellNumber = kNeighborhoodGraph_->filledCellNumber();
                const size_t &kDivisionNumber = kNeighborhoodGraph_->getDivisionNumber();
                const size_t kMaximumCellNumber = std::pow(kDivisionNumber, 4);

                // Initialize the structures speeding up the selection by caching data
                gridCornerMask.resize(kMaximumCellNumber, false);
                gridCornerCoordinatesH.resize(kMaximumCellNumber);

                // Save additional info needed for the selection
                const auto &sizes = kNeighborhoodGraph_->getCellSizes();

                additionalParameters.resize(5);
                additionalParameters[0] = sizes[0]; // The width of the source image
                additionalParameters[1] = sizes[1]; // The height of the source image
                additionalParameters[2] = sizes[2]; // The width of the destination image
                additionalParameters[3] = sizes[3]; // The height of the destination image
                additionalParameters[4] = kDivisionNumber; // The number of cells along an axis   
            }

            // The function that runs the model-based inlier selector
            void run(
                const cv::Mat& kCorrespondences_, // All point correspondences
                const gcransac::Model& kModel_, // The model parameters
                const _NeighborhoodStructure& kNeighborhood_, // The neighborhood structure. This probably will be a GridNeighborhood currently.
                const double& inlierOutlierThreshold_,
                std::vector<const std::vector<size_t>*>& selectedCells_, // The indices of the points selected
                size_t& pointNumber_); 

        
        };

        // The function that runs the model-based inlier selector
        template <
            typename _Estimator,
            typename _NeighborhoodStructure>
        void SpacePartitioningRANSAC<_Estimator, _NeighborhoodStructure>::run(
            const cv::Mat& kCorrespondences_, // All point correspondences
            const gcransac::Model& kModel_, // The model parameters
            const _NeighborhoodStructure& kNeighborhood_, // The neighborhood structure. This probably will be a GridNeighborhood currently.
            const double& inlierOutlierThreshold_,
            std::vector<const std::vector<size_t>*>& selectedCells_, // The indices of the points selected
            size_t& pointNumber_)
        {
            // Initializing the selected point number to zero
            pointNumber_ = 0;

//            if constexpr (std::is_same<_Estimator, gcransac::utils::DefaultHomographyEstimator>())
//                runHomography(
//                    kCorrespondences_,
//                    kModel_,
//                    kNeighborhood_,
//                    selectedCells_,
//                    pointNumber_,
//                    inlierOutlierThreshold_);
//            else  // Other then homography estimation is not implemented yet.
//            {
                // Return all points
                const auto& cellMap = kNeighborhood_.getCells();
                selectedCells_.reserve(cellMap.size());
                for (const auto& [cell, value] : cellMap)
                {
                    const auto &points = value.first;
                    selectedCells_.emplace_back(&points);
                    pointNumber_ += points.size();
                }
//            }
        }
        
      
    }
}
