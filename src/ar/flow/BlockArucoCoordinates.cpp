//---------------------------------------------------------------------------------------------------------------------
//  AR MICO plugin
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------


#include <mico/ar/flow/BlockArucoCoordinates.h>
#include <flow/Outpipe.h>
#include <flow/Policy.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

namespace mico{
        BlockArucoCoordinates::BlockArucoCoordinates(){
            createPipe<cv::Mat>("image");

            createPolicy({  flow::makeInput<Eigen::Matrix4f>("coordinates") });

            registerCallback({"image"}, 
                [&](flow::DataFlow _data){
                    auto image = _data.get<cv::Mat>("image");
                    Eigen::Matrix4f coordinates = Eigen::Matrix4f::Identity();
                    if(getPipe("coordinates")->registrations()){
                        getPipe("coordinates")->flush(coordinates);
                    }
                }
            );

            
            registerCallback({"input"}, 
                [&](flow::DataFlow _data){
                    auto u_ = _data.get<float>("input");
                }
            );
        }

        bool BlockArucoCoordinates::configure(std::vector<flow::ConfigParameterDef> _params) {
            if(auto param = getParamByName(_params, "id"); param){
                id_ = param.value().asInteger();
            }

            return true;
        }
        
        std::vector<flow::ConfigParameterDef> BlockArucoCoordinates::parameters(){
            return {
                {"id", flow::ConfigParameterDef::eParameterType::INTEGER, 1}
            };
        }


}