//
//  utils.hpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2023/4/26.
//  Copyright © 2023 zx. All rights reserved.
//

#ifndef utils_compress_hpp
#define utils_compress_hpp

#pragma once


#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

namespace LBFC2
{


class Utils
{
public:
    static bool writeMatrix(const cv::Mat &mat, const std::string &path);
    static bool readMatrix(cv::Mat &mat, const std::string &path);

    static bool writeMatrix(const cv::Mat &mat, FILE *f);
    static bool readMatrix(cv::Mat &mat, FILE *f);

    static bool writeVectorOfMatrix(const std::vector<cv::Mat> &mat, FILE *f);
    static bool readVectorOfMatrix(std::vector<cv::Mat> &mat, FILE *f);


    static void bin2mat( cv::InputArray _src, cv::OutputArray _dst, int type = CV_8U, bool norm = false  );
    static void mat2bin( cv::InputArray _src, cv::OutputArray _dst );


    template<typename T>
    static size_t findNearestNeighbourIndex(const T &value, const std::vector<T> &x)
    {
        typename std::vector<T>::const_iterator first = x.begin();
        typename std::vector<T>::const_iterator last = x.end();
        typename std::vector<T>::const_iterator before = std::lower_bound(first, last, value);
        typename std::vector<T>::const_iterator it;

        if (before == first)
            it = first;
        else if (before == last)
            it = --last;
        else
        {
            typename std::vector<T>::const_iterator after = before;
            --before;
            it = (*after - value) < (value - *before) ? after : before;
        }

        return std::distance(first, it);
    }

};

} // END NAMESPACE



#endif /* utils_hpp */
